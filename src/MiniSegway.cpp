#include "MiniSegway.h"

MiniSegway::MiniSegway(RC& rc) : _Thread(osPriorityHigh, 4096)
                               , _rc(rc)
                               , _imu(MINI_SEGWAY_IMU_MOSI,
                                      MINI_SEGWAY_IMU_MISO,
                                      MINI_SEGWAY_IMU_CLK,
                                      MINI_SEGWAY_IMU_CS)
                               , _button(MINI_SEGWAY_BLUE_BUTTON, PullUp)
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
    DigitalOut led2(MINI_SEGWAY_LED);

    // timer to measure delta time
    Timer timer;
    timer.start();
    microseconds time_previous_us{0};

    // serial stream either to matlab or to the openlager
    SerialStream serialStream(MINI_SEGWAY_NUM_OF_FLOATS,
                              MINI_SEGWAY_TX,
                              MINI_SEGWAY_RX,
                              MINI_SEGWAY_BAUDRATE);

    // motor driver enable
    DigitalOut enable_motor_driver(MINI_SEGWAY_ENABLE_MOTOR_DRIVER);

    // encoders
    Encoder encoder_M1(MINI_SEGWAY_ENCA_M1,
                       MINI_SEGWAY_ENCB_M1,
                       MINI_SEGWAY_MOTOR_COUNTS_PER_TURN,
                       MINI_SEGWAY_MOTOR_VELOCITY_FILTER_FREQUENCY_HZ,
                       MINI_SEGWAY_MOTOR_VELOCITY_FILTER_DAMPING,
                       MINI_SEGWAY_TS);
    Encoder encoder_M2(MINI_SEGWAY_ENCA_M2,
                       MINI_SEGWAY_ENCB_M2,
                       MINI_SEGWAY_MOTOR_COUNTS_PER_TURN,
                       MINI_SEGWAY_MOTOR_VELOCITY_FILTER_FREQUENCY_HZ,
                       MINI_SEGWAY_MOTOR_VELOCITY_FILTER_DAMPING,
                       MINI_SEGWAY_TS);

    // motors
    Motor motor_M1(MINI_SEGWAY_PWM_M1,
                   MINI_SEGWAY_PWM_DIR_M1,
                   MINI_SEGWAY_MOTOR_VOLTAGE_MAX);
    Motor motor_M2(MINI_SEGWAY_PWM_M2,
                   MINI_SEGWAY_PWM_DIR_M2,
                   MINI_SEGWAY_MOTOR_VOLTAGE_MAX);

    // current sensor
    AnalogIn analog_in_M1(MINI_SEGWAY_CURRENT_AIN_M1);
    AnalogIn analog_in_M2(MINI_SEGWAY_CURRENT_AIN_M2);
    IIRFilter currentLowPass2[2];
    currentLowPass2[0].lowPass2Init(MINI_SEGWAY_CURRENT_FILTER_FREQUENCY_HZ,
                                    MINI_SEGWAY_CURRENT_FILTER_DAMPING,
                                    MINI_SEGWAY_TS);
    currentLowPass2[1].lowPass2Init(MINI_SEGWAY_CURRENT_FILTER_FREQUENCY_HZ,
                                    MINI_SEGWAY_CURRENT_FILTER_DAMPING,
                                    MINI_SEGWAY_TS);

    // differential drive kinematic
    // q = Cwheel2robot * w
    // q = (v, w)    v : forward speed in m/sec
    //               w : turn rate in rad/sec
    // w = (w1, w2)  w1: right wheel speed in rad/sec
    //               w2: left  wheel speed in rad/sec
    const Eigen::Matrix2f Cwheel2robot = (Eigen::Matrix2f() << R_WHEEL / 2.0f   ,  R_WHEEL / 2.0f   ,
                                                               R_WHEEL / B_WHEEL, -R_WHEEL / B_WHEEL).finished();

    // max forward speed and turn rate
    const float k_voltage2wheel_speed = 2.0f * M_PIf * (MINI_SEGWAY_MOTOR_KN / 60.0f);
    const float wheel_speed_max = k_voltage2wheel_speed * MINI_SEGWAY_MOTOR_VOLTAGE_MAX;
    const float forward_speed_max = Cwheel2robot.row(0) * (Eigen::Vector2f() <<  wheel_speed_max,
                                                                                 wheel_speed_max).finished();
    const float turn_rate_max     = Cwheel2robot.row(1) * (Eigen::Vector2f() <<  wheel_speed_max,
                                                                                -wheel_speed_max).finished();

    // PID controllers
    PIDController Cpi_vel;
    Cpi_vel.piControllerInit(MINI_SEGWAY_CPI_VEL_KP,
                             MINI_SEGWAY_CPI_VEL_KI,
                             MINI_SEGWAY_TS);
    // PIDController Cpd_ang;
    // Cpd_ang.pdt1ControllerInit(MINI_SEGWAY_CPD_ANG_KP,
    //                            MINI_SEGWAY_CPD_ANG_KD,
    //                            MINI_SEGWAY_CPD_ANG_FCUT_D,
    //                            MINI_SEGWAY_TS);

    // states for state machine
    enum RobotState {
        CAR,     // 0
        SEGWAY,  // 1
    } robot_state = RobotState::CAR;

#if MINI_SEGWAY_CHIRP_USE_CHIRP
    // chirp generator
    Chirp chirp(MINI_SEGWAY_CHIRP_F0,
                MINI_SEGWAY_CHIRP_F1,
                MINI_SEGWAY_CHIRP_T1,
                MINI_SEGWAY_TS);
    float voltage = MINI_SEGWAY_CHIRP_OFFSET;
    float sinarg = 0.0f;
#endif

#if MINI_SEGWAY_AIN_USE_ADDITIONAL_CURRENT_SENSOR
    // additional current sensor
    AnalogIn analog_additional_in_M1(MINI_SEGWAY_AIN_ADDITIONAL_M1);
    AnalogIn analog_additional_in_M2(MINI_SEGWAY_AIN_ADDITIONAL_M2);
    float current_additional_M1;
    float current_additional_M2;
#endif

    // give the system some time to start, e.g. opelager and imu angle estimate (openLager needs 1 second)
    thread_sleep_for(MINI_SEGWAY_IMU_NUM_FOR_AVERAGE);
    printf("MiniSegway running...\n");


    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        // logic so that _do_execute can also be triggered by the start byte via matlab
        static bool is_start_byte_received = false;
        if (!is_start_byte_received && serialStream.startByteReceived()) {
            is_start_byte_received = true;
            toggleDoExecute();
        }

        // sbus pkg needs to be processed to update the rc_pkg
        _rc.processReceivedData();

        // read processed rc package
        const RC::rc_pkg_t rc_pkg = _rc.update();

        // read imu data
        const IMU::ImuData imu_data = _imu.update();

        // TODO: Implement I2T protection (slow lowpass applied to current sensor signal, if current is too high, disable motor driver)
        // read current sensor from motor driver (really ruff estimation of the current scaling, current is always positive)
        const float current_M1 = currentLowPass2[0].apply(analog_in_M1.read() * 3.3f * 5.5f);
        const float current_M2 = currentLowPass2[1].apply(analog_in_M2.read() * 3.3f * 5.5f);

        // measure delta time
        const microseconds time_us = timer.elapsed_time();
        const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
        time_previous_us = time_us;

        // arm is only true if receiver data is valid and arm button is pressed
        if (_do_execute && rc_pkg.armed) {

            // enable motor driver
            if (enable_motor_driver == 0) {
                enable_motor_driver = 1;
                led2 = 1;
            }

            // read encoder signals
            const Encoder::encoder_signals_t encoder_signals_M1 = encoder_M1.read();
            const Encoder::encoder_signals_t encoder_signals_M2 = encoder_M2.read();
            

#if MINI_SEGWAY_AIN_USE_ADDITIONAL_CURRENT_SENSOR
            // read additional current sensor
            current_additional_M1 = (analog_additional_in_M1.read() * 3.3f - 2.5f) / 0.8f;
            current_additional_M2 = (analog_additional_in_M2.read() * 3.3f - 2.5f) / 0.8f;
#endif

            // calculate robot velocities from wheel speeds and gyro data
            Eigen::Vector2f wheel_speed{encoder_signals_M1.velocity + imu_data.gyro(0),
                                        encoder_signals_M2.velocity + imu_data.gyro(0)};
            Eigen::Vector2f robot_coord = Cwheel2robot * wheel_speed;

            // state machine mainly calculates the setpoints for the motors
            Eigen::Vector2f robot_coord_setpoint{0.0f, 0.0f};

#if MINI_SEGWAY_CHIRP_USE_CHIRP
            float sinarg = 0.0f;
#endif

            switch (robot_state) {
                case RobotState::CAR: {
                    
                    // mix wheel speed based on rc input
                    float flip_mixer_sign = 1.0f;
                    if (imu_data.rpy(0) < 0.0f)
                        flip_mixer_sign = -1.0f;
                    robot_coord_setpoint << flip_mixer_sign * MINI_SEGWAY_CAR_MIXER_GAIN * forward_speed_max * rc_pkg.forward_speed, 
                                            flip_mixer_sign * (1.0f - MINI_SEGWAY_CAR_MIXER_GAIN) * turn_rate_max * rc_pkg.turn_rate;

                    // if the absolute angle is small enough we switch to segway mode
                    if (fabs(imu_data.rpy(0)) < MINI_SEGWAY_ABS_ANGLE_START_BALANCE_RAD)
                        robot_state = RobotState::SEGWAY;

                    break;
                }
                case RobotState::SEGWAY: {

                    // mix wheel speed based on rc input
                    robot_coord_setpoint << MINI_SEGWAY_CAR_MIXER_GAIN * forward_speed_max * rc_pkg.forward_speed, 
                                            -1.0f * (1.0f - MINI_SEGWAY_CAR_MIXER_GAIN) * turn_rate_max * rc_pkg.turn_rate;

#if MINI_SEGWAY_CHIRP_USE_CHIRP
                    float exc = MINI_SEGWAY_CHIRP_OFFSET;
                    if (rc_pkg.armed) {
                        if (chirp.update()) {
                            exc = MINI_SEGWAY_CHIRP_AMPLITUDE * chirp.getExc() + MINI_SEGWAY_CHIRP_OFFSET;
                            sinarg = chirp.getSinarg();
                        } 
                    }
                    robot_coord_setpoint(0) += exc;
#endif

                    // cascaded pid controller for velocity and angle
                    const float u_pi_vel = Cpi_vel.applyConstrained(robot_coord_setpoint(0) - robot_coord(0),
                                                                    -MINI_SEGWAY_CAR_MIXER_GAIN * forward_speed_max,
                                                                    MINI_SEGWAY_CAR_MIXER_GAIN * forward_speed_max);
                    const float u_pd_ang = MINI_SEGWAY_CPD_ANG_KP * imu_data.rpy(0) + MINI_SEGWAY_CPD_ANG_KD * imu_data.gyro(0);
                    // TODO: implement saturation
                    robot_coord_setpoint(0) = -1.0f * (u_pi_vel - u_pd_ang);

                    // TODO: implement angle controller
                    // robot_coord_setpoint(1) = ...;

                    // if the absolute angle is bigger than a certain threshold we switch to reset mode
                    if (fabs(imu_data.rpy(0)) > MINI_SEGWAY_ABS_ANGLE_STOP_BALANCE_RAD) {
                        robot_state = RobotState::CAR;
                        Cpi_vel.reset(0.0f);
                    }

                    break;
                }
            }

            // write velocity setpoints to motors
            const Eigen::Vector2f voltage = Cwheel2robot.inverse() * robot_coord_setpoint / k_voltage2wheel_speed;
            motor_M1.setVoltage(voltage(0));
            motor_M2.setVoltage(voltage(1));

            // send data to serial stream (openlager or laptop / pc)
            serialStream.write( dtime_us );                        //  0 micro seconds

            serialStream.write( rc_pkg.turn_rate );                //  1 normalized between -1.0 and 1.0
            serialStream.write( rc_pkg.forward_speed );            //  2 normalized between -1.0 and 1.0
            serialStream.write( (rc_pkg.armed ? 1.0f : 0.0f) );    //  3 arming switch (1.0f (armed) or 0.0f)
            serialStream.write( _rc.getPeriod() * 2.2222e-04f );   //  4 microseconds

            serialStream.write( encoder_signals_M1.velocity );     //  5 rad/sec
            serialStream.write( encoder_signals_M2.velocity );     //  6 rad/sec

            serialStream.write( encoder_signals_M1.rotations );    //  7 rad
            serialStream.write( encoder_signals_M2.rotations );    //  8 rad

            serialStream.write( imu_data.gyro(0) );                //  9 rad/sec
            serialStream.write( imu_data.gyro(1) );                // 10 rad/sec
            serialStream.write( imu_data.gyro(2) );                // 11 rad/sec

            serialStream.write( imu_data.acc(0) );                 // 12 m/sec^2
            serialStream.write( imu_data.acc(1) );                 // 13 m/sec^2
            serialStream.write( imu_data.acc(2) );                 // 14 m/sec^2

            serialStream.write( imu_data.rpy(0) );                 // 15 rad
            serialStream.write( imu_data.rpy(1) );                 // 16 rad
            serialStream.write( imu_data.rpy(2) );                 // 17 rad

            serialStream.write( voltage(0) );                      // 18 voltage
            serialStream.write( voltage(1) );                      // 19 voltage

            serialStream.write( current_M1 );                      // 20 amps (or at least approximately)
            serialStream.write( current_M2 );                      // 21 amps (or at least approximately)

            serialStream.write( robot_coord(0) );                  // 22 forward speed in m/sec
            serialStream.write( robot_coord(1) );                  // 23 turn rate in rad/sec

            serialStream.write( robot_coord_setpoint(0) );         // 24 forward speed setpoint in m/sec
            serialStream.write( robot_coord_setpoint(1) );         // 25 turn rate setpoint in rad/sec

            serialStream.write( static_cast<float>(robot_state) ); // 26 robot state

#if MINI_SEGWAY_CHIRP_USE_CHIRP
            serialStream.write( sinarg );                          // 27 (rad)
#endif

            serialStream.send();

        } else {
            // reset the system once
            if (_do_reset) {
                _do_reset = false;

                enable_motor_driver = 0;
                led2 = 0;
                serialStream.reset();
                motor_M1.setVoltage(0.0f);
                motor_M2.setVoltage(0.0f);
                robot_state = RobotState::CAR;
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