#include "MiniSegway.h"

MiniSegway::MiniSegway(RC& rc, IMU& imu) : _Thread(osPriorityHigh, 4096)
                                         , _rc(rc)
                                         , _imu(imu)
                                         , _button(MINI_SEGWAY_BUTTON, PullUp)
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

    // // motors
    // DCMotor motor_M1(MINI_SEGWAY_PWM_M1_POS,
    //                  MINI_SEGWAY_PWM_M1_NEG,
    //                  MINI_SEGWAY_ENCA_M1,
    //                  MINI_SEGWAY_ENCB_M1,
    //                  MINI_SEGWAY_GEAR_RATIO,
    //                  MINI_SEGWAY_KN,
    //                  MINI_SEGWAY_VOLTAGE_MAX);
    // DCMotor motor_M2(MINI_SEGWAY_PWM_M2_POS,
    //                  MINI_SEGWAY_PWM_M1_NEG,
    //                  MINI_SEGWAY_ENCA_M2,
    //                  MINI_SEGWAY_ENCB_M2,
    //                  MINI_SEGWAY_GEAR_RATIO,
    //                  MINI_SEGWAY_KN,
    //                  MINI_SEGWAY_VOLTAGE_MAX);
    // motor_M1.setVelocityCntrlIntegratorLimitsPercent(80.0f);
    // motor_M2.setVelocityCntrlIntegratorLimitsPercent(80.0f);
    // DCMotor::motor_signals_t motor_signals_M1;
    // DCMotor::motor_signals_t motor_signals_M2;

    // encoders
    Encoder encoder_M1(MINI_SEGWAY_ENCA_M1,
                       MINI_SEGWAY_ENCB_M1,
                       MINI_SEGWAY_COUNTS_PER_TURN,
                       MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY_RAD_SEC,
                       MINI_SEGWAY_VELOCITY_FILTER_DAMPING,
                       MINI_SEGWAY_TS);
    Encoder encoder_M2(MINI_SEGWAY_ENCA_M2,
                       MINI_SEGWAY_ENCB_M2,
                       MINI_SEGWAY_COUNTS_PER_TURN,
                       MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY_RAD_SEC,
                       MINI_SEGWAY_VELOCITY_FILTER_DAMPING,
                       MINI_SEGWAY_TS);
    Encoder::encoder_signals_t encoder_signals_M1 = encoder_M1.read();
    Encoder::encoder_signals_t encoder_signals_M2 = encoder_M2.read();

    // motors
    Motor motor_M1(MINI_SEGWAY_PWM_M1,
                   MINI_SEGWAY_PWM_DIR_M1,
                   MINI_SEGWAY_VOLTAGE_MAX);
    Motor motor_M2(MINI_SEGWAY_PWM_M2,
                   MINI_SEGWAY_PWM_DIR_M2,
                   MINI_SEGWAY_VOLTAGE_MAX);
    float voltage_M1{0.0f};
    float voltage_M2{0.0f};

    // imu
    IMU::ImuData imu_data;

    // differential drive kinematic
    // q = Cwheel2robot * s
    // q = (v, w)    v : forward speed in m/sec
    //               w : turn rate in rad/sec
    // w = (w1, w2)  w1: right wheel speed in rad/sec
    //               w2: left  wheel speed in rad/sec
    const Eigen::Matrix2f Cwheel2robot = (Eigen::Matrix2f() << R_WHEEL / 2.0f   ,  R_WHEEL / 2.0f   ,
                                                               R_WHEEL / B_WHEEL, -R_WHEEL / B_WHEEL).finished();
    // const float wheel_speed_max = 2.0f * M_PIf * motor_M1.getMaxVelocity();
    const float wheel_speed_max = 2.0f * M_PIf * MINI_SEGWAY_KN / 60.0f * MINI_SEGWAY_VOLTAGE_MAX;

    const float forward_speed_max = Cwheel2robot.row(0) * (Eigen::Vector2f() <<  wheel_speed_max,
                                                                                 wheel_speed_max).finished();
    const float turn_rate_max     = Cwheel2robot.row(1) * (Eigen::Vector2f() <<  wheel_speed_max,
                                                                                -wheel_speed_max).finished();
    // forward speed to turn rate mix gain, mix_gain in (0.0, 1.0)
    const float mixer_gain = 0.7f;
    // flip_sign = 1.0 -> mini segway, flip_sign = -1.0 -> car on ground
    const float flip_mixer_sign = -1.0f;
    Eigen::Vector2f wheel_speed = {0.0f, 0.0f};
    Eigen::Vector2f robot_coord = {0.0f, 0.0f};

#if MINI_SEGWAY_CHIRP_USE_CHIRP
    // chirp generator
    Chirp chirp(MINI_SEGWAY_CHIRP_F0,
                MINI_SEGWAY_CHIRP_F1,
                MINI_SEGWAY_CHIRP_T1,
                MINI_SEGWAY_TS);
    float voltage = MINI_SEGWAY_CHIRP_OFFSET;
    float sinarg = 0.0f;
#endif

#if MINI_SEGWAY_AIN_USE_CURRENT_SENSOR
    // current sensor
    AnalogIn analog_in_M1(MINI_SEGWAY_AIN_1);
    AnalogIn analog_in_M2(MINI_SEGWAY_AIN_2);
    float current_M1;
    float current_M2;
#endif

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
        rc_pkg = _rc.update();

        // read motor signals
        // motor_signals_M1 = motor_M1.read();
        // motor_signals_M2 = motor_M2.read();

        // read motor signals
        encoder_signals_M1 = encoder_M1.read();
        encoder_signals_M2 = encoder_M2.read();

        // read imu data
        imu_data = _imu.update();

#if MINI_SEGWAY_AIN_USE_CURRENT_SENSOR
        // read current sensor
        current_M1 = 0.8f * (analog_in_M1.read() * 3.3f) - 2.5f;
        current_M2 = 0.8f * (analog_in_M2.read() * 3.3f) - 2.5f;
#endif

        // measure delta time
        const microseconds time_us = timer.elapsed_time();
        const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
        time_previous_us = time_us;

        // arm is only true if receiver data is valid and arm button is pressed
        if (_do_execute) { // && rc_pkg.armed) {
            
            // mix wheel speed based on rc input
            robot_coord << flip_mixer_sign * mixer_gain * forward_speed_max * rc_pkg.forward_speed, 
                           flip_mixer_sign * (1.0f - mixer_gain) * turn_rate_max * rc_pkg.turn_rate;
            wheel_speed = Cwheel2robot.inverse() * robot_coord;
            // motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
            // motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));

            // 2.0f * M_PIf * MINI_SEGWAY_KN / 60.0f * MINI_SEGWAY_VOLTAGE_MAX
            voltage_M1 = (wheel_speed(0) / (2.0f * M_PIf)) / (MINI_SEGWAY_KN / 60.0f);
            voltage_M2 = (wheel_speed(1) / (2.0f * M_PIf)) / (MINI_SEGWAY_KN / 60.0f);

            // write voltage to motors
            motor_M1.setVoltage(voltage_M1);
            motor_M2.setVoltage(voltage_M2);

// TODO: adjust code if you want to use the following section again
#if MINI_SEGWAY_CHIRP_USE_CHIRP
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
#endif

            // send data to serial stream (openlager or laptop / pc)
            serialStream.write( dtime_us );                      //  0 
            serialStream.write( rc_pkg.turn_rate );              //  1 
            serialStream.write( rc_pkg.forward_speed );          //  2
            serialStream.write( (rc_pkg.armed ? 1.0f : 0.0f) );  //  3
#if DO_USE_PPM_IN
            serialStream.write( _rc.getPeriod() * 1.0e-4f );     //  4 
#else
            serialStream.write( _rc.getPeriod() * 2.2222e-04f ); //  4
#endif
            // serialStream.write( motor_signals_M1.velocity );     //  5
            // serialStream.write( motor_signals_M2.velocity );     //  6
            serialStream.write( encoder_signals_M1.velocity );     //  5
            serialStream.write( encoder_signals_M2.velocity );     //  6
            // serialStream.write( motor_signals_M1.rotations );    //  7
            // serialStream.write( motor_signals_M2.rotations );    //  8
            serialStream.write( encoder_signals_M1.rotations );    //  7
            serialStream.write( encoder_signals_M2.rotations );    //  8
            serialStream.write( imu_data.gyro(0) );              //  9
            serialStream.write( imu_data.gyro(1) );              // 10
            serialStream.write( imu_data.gyro(2) );              // 11
            serialStream.write( imu_data.acc(0) );               // 12
            serialStream.write( imu_data.acc(1) );               // 13
            serialStream.write( imu_data.acc(2) );               // 14
            serialStream.write( imu_data.rpy(0) );               // 15
            serialStream.write( imu_data.rpy(1) );               // 16
            serialStream.write( imu_data.rpy(2) );               // 17
            // serialStream.write( motor_signals_M1.voltage );      // 18
            // serialStream.write( motor_signals_M2.voltage );      // 19
            serialStream.write( voltage_M1 );      // 18
            serialStream.write( voltage_M2 );      // 19
            // serialStream.write( motor_M1.getVelocitySetpoint()); // 20
            // serialStream.write( motor_M2.getVelocitySetpoint()); // 21
            serialStream.send();

            led = 1;
        } else {
            if (_do_reset) {
                _do_reset = false;
                led = 0;
                // enable_motor_driver = 0;
                serialStream.reset();
                // motor_M1.setVelocity(0.0f);
                // motor_M2.setVelocity(0.0f);
                motor_M1.setVoltage(0.0f);
                motor_M2.setVoltage(0.0f);
            }
        }
    }
}

void MiniSegway::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}

void MiniSegway::toggleDoExecute()
{
    _do_execute = !_do_execute;
    if (_do_execute)
        _do_reset = true;
}