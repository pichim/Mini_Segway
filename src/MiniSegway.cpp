#include "MiniSegway.h"

MiniSegway::MiniSegway(RC& rc, IMU& imu) : _rc(rc)
                                         , _imu(imu)
                                         , _button(MINI_SEGWAY_BUTTON, PullUp)
                                         , _Thread(osPriorityHigh, 4096)
{
    _button.fall(callback(this, &MiniSegway::toggleDoExecute));

    _Thread.start(callback(this, &MiniSegway::threadTask));
    _Ticker.attach(callback(this, &MiniSegway::sendThreadFlag), microseconds{MINI_SEGWAY_PERIOD_US});
}

MiniSegway::~MiniSegway()
{
    _Ticker.detach();
    _Thread.terminate();
}

void MiniSegway::toggleDoExecute()
{
    _do_execute = !_do_execute;
    if (_do_execute)
        _do_reset = true;
}

void MiniSegway::threadTask()
{
    // additional LED
    DigitalOut led(MINI_SEGWAY_LED);
    led = 0;

    // timer to measure delta time
    Timer timer;
    timer.start();
    microseconds time_previous_us{0};

    // processed rc package
    RC::rc_pkg_t rc_pkg;

    // serial stream either to matlab or to the openlager
    SerialStream serialStream(MINI_SEGWAY_NUM_OF_FLOATS,
                              MINI_SEGWAY_TX,
                              MINI_SEGWAY_RX,
                              MINI_SEGWAY_BAUDRATE);
    
    // encoders
    Encoder encoder_M1(MINI_SEGWAY_ENCA_M1,
                       MINI_SEGWAY_ENCB_M1,
                       MINI_SEGWAY_COUNTS_PER_TURN,
                       MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY,
                       MINI_SEGWAY_TS);
    Encoder encoder_M2(MINI_SEGWAY_ENCA_M2,
                       MINI_SEGWAY_ENCB_M2,
                       MINI_SEGWAY_COUNTS_PER_TURN,
                       MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY,
                       MINI_SEGWAY_TS);
    Encoder::encoder_signals_t encoder_signals_M1 = encoder_M1.read();
    Encoder::encoder_signals_t encoder_signals_M2 = encoder_M2.read();

    // motors
    DigitalOut enable_motor_driver(MINI_SEGWAY_ENABLE_MOTOR_DRIVER);
    Motor motor_M1(MINI_SEGWAY_PWM_M1, MINI_SEGWAY_VOLTAGE_MAX);
    Motor motor_M2(MINI_SEGWAY_PWM_M2, MINI_SEGWAY_VOLTAGE_MAX);
    float voltage_M1{0.0f};
    float voltage_M2{0.0f};

    // robot geometry for kinematics
    Eigen::Matrix2f Cwheel2robot; // transform wheel to robot
    Cwheel2robot <<  R_WHEEL / 2.0f   ,  R_WHEEL / 2.0f   ,
                     R_WHEEL / L_WHEEL, -R_WHEEL / L_WHEEL;

    Eigen::Vector2f robot_coord = {0.0f, 0.0f};  // contains v and w (robot translational and rotational velocities)
    Eigen::Vector2f wheel_speed = {0.0f, 0.0f};  // w1 w2 (wheel speed)

    // imu
    IMU::ImuData imu_data;

    // chirp generator
    Chirp chirp(MINI_SEGWAY_CHIRP_F0,
                MINI_SEGWAY_CHIRP_F1,
                MINI_SEGWAY_CHIRP_T1,
                MINI_SEGWAY_TS);
    float voltage = MINI_SEGWAY_CHIRP_OFFSET;
    float sinarg = 0.0f;

    // // current sensor
    // AnalogIn analog_in_M1(MINI_SEGWAY_AIN1);
    // AnalogIn analog_in_M2(MINI_SEGWAY_AIN2);
    // float current_M1;
    // float current_M2;


    // give the openLager 1000 msec time to start
    thread_sleep_for(1000);

    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        // logic so that _do_execute can also be triggered by the start byte via matlab
        static bool is_start_byte_received = false;
        if (!is_start_byte_received && serialStream.startByteReceived()) {
            is_start_byte_received = true;
            toggleDoExecute();
        }

// only process the sbus pkg if sbus is used and is running as an own thread
#if !DO_USE_PPM_IN && !SBUS_DO_RUN_AS_THREAD
        // sbus pkg needs to be processed to update the rc_pkg
        // whereas when its ppm in, the rc_pkg is updated via ISR
        _rc.processReceivedData();
#endif
        // arm is only true if receiver data is valid and arm button is pressed
        rc_pkg = _rc.update();

        // read motor signals
        encoder_signals_M1 = encoder_M1.read();
        encoder_signals_M2 = encoder_M2.read();

        // read imu data
        imu_data = _imu.update();

        // // read current sensor
        // current_M1 = 0.8f * (analog_in_M1.read() * 3.3f) - 2.5f;
        // current_M2 = 0.8f * (analog_in_M2.read() * 3.3f) - 2.5f;

        // measure delta time
        const microseconds time_us = timer.elapsed_time();
        const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
        time_previous_us = time_us;

        // TODO: Use rc_pkg.armed
        // here lifes the main logic of the mini segway
        if (_do_execute ) { //&& rc_pkg.armed) {

            // enable motor drivers
            if (enable_motor_driver == 0)
                enable_motor_driver = 1;

            // // apply robot kinematics
            // robot_coord(1) = TURN_RATIO * rc_pkg.turn_rate;
            // robot_coord(0) = vel_cntrl_v2_fcn(rc_pkg.forward_speed * MINI_SEGWAY_VEL_MAX_RADS,
            //                                   B_TURN,
            //                                   robot_coord(1),
            //                                   Cwheel2robot);
            // wheel_speed = (Cwheel2robot.inverse() * robot_coord) / MINI_SEGWAY_VEL_MAX_RADS;
            
            // set voltage to the motors
            // voltage_M1 = wheel_speed(0) * MINI_SEGWAY_VOLTAGE_MAX;
            // voltage_M2 = wheel_speed(1) * MINI_SEGWAY_VOLTAGE_MAX;

            // perform frequency response measurement
            if (chirp.update()) {
                const float exc = chirp.getExc();
                // const float fchirp = chirp.getFreq();
                sinarg = chirp.getSinarg();
                voltage = MINI_SEGWAY_CHIRP_AMPLITUDE * exc + MINI_SEGWAY_CHIRP_OFFSET;
            } else {
                // toggleDoExecute();
                _do_execute = false;
            }
            // apply voltage to the motors
            voltage_M1 = voltage;
            voltage_M2 = voltage;

            // write voltage to motors
            motor_M1.setVoltage(voltage_M1);
            motor_M2.setVoltage(voltage_M2);

            // send data to serial stream (openlager or laptop / pc)
            serialStream.write( dtime_us );                      //  0 
            serialStream.write( rc_pkg.forward_speed );          //  1 
            serialStream.write( rc_pkg.turn_rate );              //  2
            serialStream.write( (rc_pkg.armed ? 1.0f : 0.0f) );  //  3
#if DO_USE_PPM_IN
            serialStream.write( _rc.getPeriod() * 1.0e-4f );     //  4 
#else
            serialStream.write( _rc.getPeriod() * 2.2222e-04f ); //  4
#endif
            serialStream.write( encoder_signals_M1.velocity );   //  5
            serialStream.write( encoder_signals_M2.velocity );   //  6
            serialStream.write( encoder_signals_M1.rotations );  //  7
            serialStream.write( encoder_signals_M2.rotations );  //  8
            serialStream.write( imu_data.gyro(0) );              //  9
            serialStream.write( imu_data.gyro(1) );              // 10
            serialStream.write( imu_data.gyro(2) );              // 11
            serialStream.write( imu_data.acc(0) );               // 12
            serialStream.write( imu_data.acc(1) );               // 13
            serialStream.write( imu_data.acc(2) );               // 14
            serialStream.write( imu_data.rpy(0) );               // 15
            serialStream.write( imu_data.rpy(1) );               // 16
            serialStream.write( imu_data.rpy(2) );               // 17
            serialStream.write( voltage_M1 );                    // 18
            serialStream.write( voltage_M2 );                    // 19
            serialStream.write( sinarg );                        // 20
            // serialStream.write( current_M1);                     // 21
            // serialStream.write( current_M2);                     // 22
            serialStream.send();

            led = 1;
        } else {
            // if (_do_reset) {
            //     _do_reset = false;

            //     enable_motor_driver = 0;
            //     serialStream.reset();
            //     encoder_M1.reset();
            //     encoder_M2.reset();
            //     voltage_M1 = voltage_M2 = 0.0f;
            //     motor_M1.reset(); // TODO: check if this is necessary even when we set voltage_Mi to zero
            //     motor_M2.reset();

            //     robot_coord = {0.0f, 0.0f};
            //     wheel_speed = {0.0f, 0.0f};

            //     led = 0;
            // }

            // TODO: remove this after chirp experimetns are finished
            enable_motor_driver = 1;
            voltage_M1 = voltage_M2 = MINI_SEGWAY_CHIRP_OFFSET;
            motor_M1.setVoltage(voltage_M1);
            motor_M2.setVoltage(voltage_M2);
        }
    }
}

float MiniSegway::vel_cntrl_v2_fcn(const float& set_wheel_speed,
                                   const float& b,
                                   const float& robot_omega,
                                   const Eigen::Matrix2f& Cwheel2robot)
{
    // function to determine the velocity of the wheels
    // set_wheel_speed is the velocity in rad/s set by radio
    Eigen::Vector2f wheel_speed = {set_wheel_speed + 2.0f * b * robot_omega,  // -> RIGHT WHEEL
                                   set_wheel_speed - 2.0f * b * robot_omega}; // -> LEFT WHEEL
    return Cwheel2robot.block<1, 2>(0, 0) * wheel_speed;
}

void MiniSegway::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}
