#include "MiniSegway.h"

MiniSegway::MiniSegway(RC& rc) : _Thread(osPriorityHigh, 4096)
                               , _rc(rc)
                               , _imu(MINI_SEGWAY_IMU_MOSI,
                                      MINI_SEGWAY_IMU_MISO,
                                      MINI_SEGWAY_IMU_CLK,
                                      MINI_SEGWAY_IMU_CS)
                               , _button(MINI_SEGWAY_BLUE_BUTTON, PullUp)
                               , _additional_button(MINI_SEGWAY_ADD_BLUE_BUTTON, PullUp)
{
    _button.fall(callback(this, &MiniSegway::toggleDoExecute));
    _additional_button.fall(callback(this, &MiniSegway::toggleDoExecute));

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
    // additional leds
    Led led1(MINI_SEGWAY_LED1);
    Led led2(MINI_SEGWAY_LED2);

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
    const Eigen::Matrix2f Cwheel2robot = (Eigen::Matrix2f() << MINI_SEGWAY_R_WHEEL / 2.0f   ,  MINI_SEGWAY_R_WHEEL / 2.0f   ,
                                                               MINI_SEGWAY_R_WHEEL / MINI_SEGWAY_B_WHEEL, -MINI_SEGWAY_R_WHEEL / MINI_SEGWAY_B_WHEEL).finished();

    // max forward speed and turn rate
    const float k_voltage2wheel_speed = 2.0f * M_PIf * (MINI_SEGWAY_MOTOR_KN / 60.0f);
    const float wheel_speed_max = k_voltage2wheel_speed * MINI_SEGWAY_MOTOR_VOLTAGE_MAX;
    const float forward_speed_max = MINI_SEGWAY_R_WHEEL * wheel_speed_max;
    const float turn_rate_max = 2.0f * MINI_SEGWAY_R_WHEEL / MINI_SEGWAY_B_WHEEL * wheel_speed_max;

    // pid controllers and filter for pid controllers
    IIRFilter robotSetPointIntegrator[2];
    robotSetPointIntegrator[0].integratorInit(MINI_SEGWAY_TS);
    robotSetPointIntegrator[1].integratorInit(MINI_SEGWAY_TS);
    IIRFilter accLowPass1;
    accLowPass1.lowPass1Init(MINI_SEGWAY_FORWARD_CPD_VEL_FCUT_D,
                             MINI_SEGWAY_TS);
    IIRFilter gyroLowPass1;
    gyroLowPass1.lowPass1Init(MINI_SEGWAY_FORWARD_CPD_ANG_FCUT_D,
                              MINI_SEGWAY_TS);

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
 
    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        // read imu data
        const IMU::ImuData imu_data = _imu.update();

        if (_imu.isCalibrated()) {

            // inform user that the system is running
            static bool is_first_run = true;
            if (is_first_run) {
                is_first_run = false;
                printf("MiniSegway running...\n");
            }

            // lowpass filter for gyro x
            const float gyro_theta_filtered = gyroLowPass1.apply(imu_data.gyro(0));
            const float acc_x_filtered = accLowPass1.apply(-imu_data.acc(1) * cosf(imu_data.rpy(0))
                                                           +imu_data.acc(2) * sinf(imu_data.rpy(0)));

            // logic so that _do_execute can also be triggered by the start byte via matlab
            static bool is_start_byte_received = false;
            if (!is_start_byte_received && serialStream.startByteReceived()) {
                is_start_byte_received = true;
                toggleDoExecute();
            }
            // led1 shows the status of user button
            if (_do_execute)
                led1.on();
            else
                led1.blink();


            // sbus pkg needs to be processed to update the rc_pkg
            _rc.processReceivedData();
            const RC::rc_pkg_t rc_pkg = _rc.update();           
            // led2 shows the status of the arming switch
            if (rc_pkg.armed)
                led2.on();
            else
                led2.blink();


            // TODO: Implement I2T protection (slow lowpass applied to current sensor signal, if current is too high, disable motor driver)
            // read current sensor from motor driver (really ruff estimation of the current scaling, current is always positive)
            const float current_M1 = currentLowPass2[0].apply(analog_in_M1.read() * 3.3f * 5.5f);
            const float current_M2 = currentLowPass2[1].apply(analog_in_M2.read() * 3.3f * 5.5f);


            // measure delta time
            const microseconds time_us = timer.elapsed_time();
            const float dtime_us = duration_cast<microseconds>(time_us - time_previous_us).count();
            time_previous_us = time_us;


            // user button was pressed and systems is armed
            if (_do_execute && rc_pkg.armed) {

                // enable h-bridges
                if (enable_motor_driver == 0)
                    enable_motor_driver = 1;

                // read encoder signals
                const Encoder::encoder_signals_t encoder_signals_M1 = encoder_M1.read();
                const Encoder::encoder_signals_t encoder_signals_M2 = encoder_M2.read();

#if MINI_SEGWAY_AIN_USE_ADDITIONAL_CURRENT_SENSOR
                // read additional current sensor
                current_additional_M1 = (analog_additional_in_M1.read() * 3.3f - 2.5f) / 0.8f;
                current_additional_M2 = (analog_additional_in_M2.read() * 3.3f - 2.5f) / 0.8f;
#endif

                // calculate robot velocities from wheel speeds and gyro data
                Eigen::Vector2f wheel_speed{encoder_signals_M1.velocity + gyro_theta_filtered,
                                            encoder_signals_M2.velocity + gyro_theta_filtered};
                Eigen::Vector2f robot_vel = Cwheel2robot * wheel_speed;

                // calculate robot coordinates from wheel angles
                Eigen::Vector2f wheel_angle{encoder_signals_M1.rotations + 0.0f * imu_data.rpy(0),
                                            encoder_signals_M2.rotations + 0.0f * imu_data.rpy(0)};
                Eigen::Vector2f robot_pos = Cwheel2robot * wheel_angle;

                // state machine mainly calculates the setpoints for the motors
                Eigen::Vector2f robot_vel_setpoint{0.0f, 0.0f};
                Eigen::Vector2f robot_vel_input{0.0f, 0.0f};

                // calculate max forward speed and turn rate based on mode switch
                float forward_speed_max_scaled = 0.0f;
                float turn_rate_max_scaled = 0.0f;
                if (rc_pkg.mode) {
                    forward_speed_max_scaled = forward_speed_max * MINI_SEGWAY_SCALE_SPEED_MAX_FAST;
                    turn_rate_max_scaled = turn_rate_max * MINI_SEGWAY_SCALE_SPEED_MAX_FAST;
                } else {
                    forward_speed_max_scaled = forward_speed_max * MINI_SEGWAY_SCALE_SPEED_MAX_SLOW;
                    turn_rate_max_scaled = turn_rate_max * MINI_SEGWAY_SCALE_SPEED_MAX_SLOW;
                }
                
                // control logic
                switch (robot_state) {
                    case RobotState::CAR: {
                        // mix wheel speed based on rc input
                        float flip_mixer_sign = 1.0f;
                        if (imu_data.rpy(0) < 0.0f)
                            flip_mixer_sign = -1.0f;
                        robot_vel_input << flip_mixer_sign *         MINI_SEGWAY_MIXER_GAIN  * rc_pkg.forward_speed * forward_speed_max_scaled, 
                                                      -1.0 * (1.0f - MINI_SEGWAY_MIXER_GAIN) * rc_pkg.turn_rate * turn_rate_max_scaled;

                        // if the absolute angle is small enough we switch to segway mode
                        if (fabs(imu_data.rpy(0)) < MINI_SEGWAY_ABS_ANGLE_START_BALANCE_RAD) {
                            encoder_M1.reset();
                            encoder_M2.reset();
                            robotSetPointIntegrator[0].reset(0.0f);
                            robotSetPointIntegrator[1].reset(0.0f);
                            robot_state = RobotState::SEGWAY;
                        }
                        break;
                    }
                    case RobotState::SEGWAY: {
                        // mix wheel speed based on rc input
                        robot_vel_setpoint <<                 MINI_SEGWAY_MIXER_GAIN * rc_pkg.forward_speed * forward_speed_max_scaled, 
                                              -1.0f * (1.0f - MINI_SEGWAY_MIXER_GAIN) * rc_pkg.turn_rate * turn_rate_max_scaled;

#if MINI_SEGWAY_CHIRP_USE_CHIRP
                        float exc = MINI_SEGWAY_CHIRP_OFFSET;
                        // just change if (_do_execute) { // && rc_pkg.armed) {
                        // so that you can start the chirp generator with the arming switch
                        if (rc_pkg.armed) {
                            if (chirp.update()) {
                                exc = MINI_SEGWAY_CHIRP_AMPLITUDE * chirp.getExc() + MINI_SEGWAY_CHIRP_OFFSET;
                            } 
                        } else {
                            chirp.reset();
                        }
                        robot_vel_setpoint(0) += exc;
#endif

                        // state space controller with additional d part on velociity
                        const float u_p_pos = MINI_SEGWAY_FORWARD_CP_POS_KP * (robotSetPointIntegrator[0].apply(robot_vel_setpoint(0)) - robot_pos(0));
                        const float u_p_vel = MINI_SEGWAY_FORWARD_CPD_VEL_KP * robot_vel(0);
                        const float u_d_vel = MINI_SEGWAY_FORWARD_CPD_VEL_KD * acc_x_filtered;
                        const float u_p_ang = MINI_SEGWAY_FORWARD_CPD_ANG_KP * imu_data.rpy(0);
                        const float u_d_ang = MINI_SEGWAY_FORWARD_CPD_ANG_KD * gyro_theta_filtered;
                        robot_vel_input(0) = -1.0f * (u_p_pos - (u_p_vel + u_d_vel + u_p_ang + u_d_ang));

                        // proportional controller
                        robot_vel_input(1) = MINI_SEGWAY_TURN_CP_POS_KP * (robotSetPointIntegrator[1].apply(robot_vel_setpoint(1)) - robot_pos(1));

                        // if the absolute angle is bigger than a certain threshold we switch back car mode
                        if (fabs(imu_data.rpy(0)) > MINI_SEGWAY_ABS_ANGLE_STOP_BALANCE_RAD) {
                            robot_state = RobotState::CAR;
                        }
                        break;
                    }
                }

                // write velocity setpoints to motors
                const Eigen::Vector2f voltage = Cwheel2robot.inverse() * robot_vel_input / k_voltage2wheel_speed;
                motor_M1.setVoltage(voltage(0));
                motor_M2.setVoltage(voltage(1));

                // send data to serial stream (openlager or laptop / pc)
                serialStream.write( dtime_us );                      //  0 micro seconds

                serialStream.write( rc_pkg.turn_rate );              //  1 normalized between -1.0 and 1.0
                serialStream.write( rc_pkg.forward_speed );          //  2 normalized between -1.0 and 1.0
                serialStream.write( (rc_pkg.armed ? 1.0f : 0.0f) );  //  3 arming switch (1.0f (armed) or 0.0f)
                serialStream.write( _rc.getPeriod() * 2.2222e-04f ); //  4 microseconds

                serialStream.write( encoder_signals_M1.velocity );   //  5 rad/sec
                serialStream.write( encoder_signals_M2.velocity );   //  6 rad/sec
                serialStream.write( encoder_signals_M1.rotations );  //  7 rad
                serialStream.write( encoder_signals_M2.rotations );  //  8 rad

                serialStream.write( imu_data.gyro(0) );              //  9 rad/sec
                serialStream.write( imu_data.gyro(1) );              // 10 rad/sec
                serialStream.write( imu_data.gyro(2) );              // 11 rad/sec
                serialStream.write( imu_data.acc(0) );               // 12 m/sec^2
                serialStream.write( imu_data.acc(1) );               // 13 m/sec^2
                serialStream.write( imu_data.acc(2) );               // 14 m/sec^2
                serialStream.write( imu_data.rpy(0) );               // 15 rad
                serialStream.write( imu_data.rpy(1) );               // 16 rad
                serialStream.write( imu_data.rpy(2) );               // 17 rad

                serialStream.write( voltage(0) );                    // 18 voltage
                serialStream.write( voltage(1) );                    // 19 voltage
                serialStream.write( current_M1 );                    // 20 amps (or at least approximately)
                serialStream.write( current_M2 );                    // 21 amps (or at least approximately)

                serialStream.write( robot_pos(0) );                  // 22 forward speed setpoint in m/sec
                serialStream.write( robot_pos(1) );                  // 23 turn rate setpoint in rad/sec

                serialStream.write( robot_vel(0) );                  // 24 forward speed in m/sec
                serialStream.write( robot_vel(1) );                  // 25 turn rate in rad/sec

                serialStream.write( robot_vel_input(0) );            // 26 forward speed setpoint in m/sec
                serialStream.write( robot_vel_input(1) );            // 27 turn rate setpoint in rad/sec

                serialStream.write( robot_vel_setpoint(0) );         // 28 forward speed setpoint in m/sec
                serialStream.write( robot_vel_setpoint(1) );         // 29 turn rate setpoint in rad/sec
                serialStream.send();

            } else {
                if (enable_motor_driver) {
                    enable_motor_driver = 0;
                    serialStream.reset();

                    motor_M1.setVoltage(0.0f);
                    motor_M2.setVoltage(0.0f);
                    
                    encoder_M1.reset();
                    encoder_M2.reset();
                    robotSetPointIntegrator[0].reset(0.0f);
                    robotSetPointIntegrator[1].reset(0.0f);
                    robot_state = RobotState::CAR;
                }
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
}