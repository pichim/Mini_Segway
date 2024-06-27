#pragma once

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

#define MINI_SEGWAY_ORANGE_VERSION true // if false its the green segway

// task period, MPU6500 runs at 1kHz, so we want to run the control loop at 1kHz
#define MINI_SEGWAY_PERIOD_US 1000
#define MINI_SEGWAY_TS (static_cast<float>(MINI_SEGWAY_PERIOD_US) * 1.0e-6f) // sampling time
#define MINI_SEGWAY_ABS_ANGLE_START_BALANCE_RAD (20.0f * M_PIf / 180.0f)
#define MINI_SEGWAY_ABS_ANGLE_STOP_BALANCE_RAD (60.0f * M_PIf / 180.0f)
#define MINI_SEGWAY_MIXER_GAIN 0.7f // e.g. 0.7f means 70% of the control signal is used forward speed and 30% for turning
#define MINI_SEGWAY_SCALE_MAX_VEL_GAIN 0.5f


// pid controller gains
#define MINI_SEGWAY_CPI_VEL_KP (1.7f)
#define MINI_SEGWAY_CPD_VEL_F_ZERO (0.0064f)
#define MINI_SEGWAY_CPI_VEL_KI (MINI_SEGWAY_CPI_VEL_KP * (2.0f * M_PIf * MINI_SEGWAY_CPD_VEL_F_ZERO))

#define MINI_SEGWAY_CPD_ANG_KP (2.0f)
#define MINI_SEGWAY_CPD_ANG_F_ZERO (5.0f)
#define MINI_SEGWAY_CPD_ANG_KD (MINI_SEGWAY_CPD_ANG_KP / (2.0f * M_PIf * MINI_SEGWAY_CPD_ANG_F_ZERO))
#define MINI_SEGWAY_CPD_ANG_FCUT_D 200.0f

#define MINI_SEGWAY_MOTOR_VELOCITY_FILTER_DAMPING (sqrtf(3.0f) / 2.0f)
#define MINI_SEGWAY_MOTOR_VELOCITY_FILTER_FREQUENCY_HZ 10.0f

#define MINI_SEGWAY_PWM_PERIOD_US 200
// #define MINI_SEGWAY_PWM_TS (static_cast<float>(MINI_SEGWAY_PWM_PERIOD_US) * 1.0e-6f)
// #define MINI_SEGWAY_PWM_UPSAMPLING_FILTER_FREQUENCY_HZ (1.0f / (2.0f * MINI_SEGWAY_TS))
#define MINI_SEGWAY_PWM_MIN_VALUE 0.00f
#define MINI_SEGWAY_PWM_MAX_VALUE 0.99f

#define MINI_SEGWAY_IMU_GYRO_FILTER_FREQUENCY_HZ 40.0f
#define MINI_SEGWAY_IMU_ACC_FILTER_FREQUENCY_HZ 10.0f

#define MINI_SEGWAY_IMU_KP_XY (1.0f * 0.1592f * 2.0f * M_PIf)
// #define MINI_SEGWAY_IMU_KP_XY (0.1592f * 2.0f * M_PIf / (sqrtf(3.0f) / 3.0f))
#define MINI_SEGWAY_IMU_KP_Z  (1.0f * 0.1592f * 2.0f * M_PIf)
#define MINI_SEGWAY_IMU_KI_XY 0.0f
// #define MINI_SEGWAY_IMU_KI_XY (MINI_SEGWAY_IMU_KP_XY * MINI_SEGWAY_IMU_KP_XY / 3.0f)
#define MINI_SEGWAY_IMU_KI_Z  0.0f



// streaming device, openlager or laptop / pc
#define DO_USE_OPENLAGER_FOR_DATA_STREAM true
// serial data stream, tested up to 20 floats at 2 kHz
#if DO_USE_OPENLAGER_FOR_DATA_STREAM
    // openlager
    #define MINI_SEGWAY_TX PC_6
    #define MINI_SEGWAY_RX NC
#else
    // // serial via usb to matlab
    // #define MINI_SEGWAY_TX USBTX
    // #define MINI_SEGWAY_RX USBRX
    // usb 2.0-cable TTL serial 6 pin to computer
    #define MINI_SEGWAY_TX PC_10
    #define MINI_SEGWAY_RX PC_11
#endif
// openlager runs at 2000000 baudrate
#define MINI_SEGWAY_BAUDRATE 2000000
#define MINI_SEGWAY_NUM_OF_FLOATS 30 // tested up to 20 floats at 2 kHz, so 30 floats at 1 kHz should work

// remote control receiver, radiomaster elrs rx, running at 111 Hz := ~9000 mus
#define MINI_SEGWAY_RC_TX NC // not connected
#define MINI_SEGWAY_RC_RX PA_10
#define MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG (10 * (9000 / MINI_SEGWAY_PERIOD_US + 1))
#define MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG
#define MINI_SEGWAY_RC_ARMING_CHANNEL 7
#define MINI_SEGWAY_RC_USE_UPSAMPLING_FILTERS true
#define MINI_SEGWAY_RC_UPSAMPLING_FILTER_DAMPING 1.0f
#define MINI_SEGWAY_RC_UPSAMPLING_FILTER_FREQUENCY_HZ 30.0f
#define MINI_SEGWAY_RC_APPLY_EXPO false
#define MINI_SEGWAY_RC_EXPO_ALPHA 2.3f

// button
#define MINI_SEGWAY_BLUE_BUTTON BUTTON1 // blue button
#define MINI_SEGWAY_ADD_BLUE_BUTTON PC_5// additional blue button
#define MINI_SEGWAY_RESET_BUTTON PB_1   // additional reset button

// additional leds
#define MINI_SEGWAY_LED_PERIOD_US 250000
#define MINI_SEGWAY_LED1 PB_5
#define MINI_SEGWAY_LED2 PA_7

// encoders
#define MINI_SEGWAY_ENCA_M1 PA_6
#define MINI_SEGWAY_ENCB_M1 PC_7
#define MINI_SEGWAY_ENCA_M2 PB_6
#define MINI_SEGWAY_ENCB_M2 PB_7

// motors
#define MINI_SEGWAY_MOTOR_GEAR_RATIO 46.85f
#define MINI_SEGWAY_MOTOR_COUNTS_PER_TURN (48.0f * MINI_SEGWAY_MOTOR_GEAR_RATIO)
#define MINI_SEGWAY_MOTOR_KN (170.0f / 12.0f)
#define MINI_SEGWAY_MOTOR_VOLTAGE_MAX 12.0f
// #define MINI_SEGWAY_MOTOR_VELOCITY_FILTER_DAMPING (sqrtf(3.0f) / 2.0f)
// #define MINI_SEGWAY_MOTOR_VELOCITY_FILTER_FREQUENCY_HZ 10.0f

// motor driver (h-bridge)
#define MINI_SEGWAY_ENABLE_MOTOR_DRIVER PB_15

// pwm
#define MINI_SEGWAY_PWM_M1 PB_13
#define MINI_SEGWAY_PWM_DIR_M1 PB_9
#define MINI_SEGWAY_PWM_M2 PA_9
#define MINI_SEGWAY_PWM_DIR_M2 PA_8
// #define MINI_SEGWAY_PWM_PERIOD_US MINI_SEGWAY_PERIOD_US
// #define MINI_SEGWAY_PWM_MIN_VALUE 0.00f
// #define MINI_SEGWAY_PWM_MAX_VALUE 0.99f

// imu
#define MINI_SEGWAY_IMU_MOSI PC_3
#define MINI_SEGWAY_IMU_MISO PC_2
#define MINI_SEGWAY_IMU_CLK PB_10
#define MINI_SEGWAY_IMU_CS PB_4
#define MINI_SEGWAY_IMU_USE_ADDITIONAL_FILTERS true
// #define MINI_SEGWAY_IMU_GYRO_FILTER_FREQUENCY_HZ 40.0f
// #define MINI_SEGWAY_IMU_ACC_FILTER_FREQUENCY_HZ 10.0f
#define MINI_SEGWAY_IMU_NUM_FOR_AVERAGE 2000 // dont make this shorter than 1000 micro seconds
#define MINI_SEGWAY_IMU_DO_USE_STATIC_ACC_CALIBRATION true // if this is true then averages acc gets overwritten by MINI_SEGWAY_IMU_B_ACC
#if MINI_SEGWAY_ORANGE_VERSION
    #define MINI_SEGWAY_IMU_B_ACC {-0.2997f, -0.1381f, 0.0028f}
#else // has to be green version
    #define MINI_SEGWAY_IMU_B_ACC {-0.2997f, -0.1381f, 0.0028f}
#endif
// % bessel (D = sqrt(3)/2)
// w0 = 3;
// kp = w0 / ( sqrt(3)/3 )
// ki = kp^2 / 3
// % real pole, no integrator, use this if you dont use the mag
// w0 = 3;
// kp = w0;
// ki = 0;
// #define MINI_SEGWAY_IMU_KP_XY (1.0f * 0.1592f * 2.0f * M_PIf)
// // #define MINI_SEGWAY_IMU_KP_XY (0.1592f * 2.0f * M_PIf / (sqrtf(3.0f) / 3.0f))
// #define MINI_SEGWAY_IMU_KP_Z  (1.0f * 0.1592f * 2.0f * M_PIf)
// #define MINI_SEGWAY_IMU_KI_XY 0.0f
// // #define MINI_SEGWAY_IMU_KI_XY (MINI_SEGWAY_IMU_KP_XY * MINI_SEGWAY_IMU_KP_XY / 3.0f)
// #define MINI_SEGWAY_IMU_KI_Z  0.0f

// robot kinematics
#define R_WHEEL 0.039f // wheel radius in meters
#define B_WHEEL 0.125f // wheelbase, distance from wheel to wheel in meters

// chirp signal
#define MINI_SEGWAY_CHIRP_USE_CHIRP false
#if MINI_SEGWAY_CHIRP_USE_CHIRP
    #define MINI_SEGWAY_CHIRP_T1 60.0f
    #define MINI_SEGWAY_CHIRP_F0 (1.0f / MINI_SEGWAY_CHIRP_T1)
    #define MINI_SEGWAY_CHIRP_F1 (0.95f / (2.0f * MINI_SEGWAY_TS))
    #define MINI_SEGWAY_CHIRP_AMPLITUDE 0.1f
    #define MINI_SEGWAY_CHIRP_OFFSET 0.0f
#endif

// current sensors from h-bridge
#define MINI_SEGWAY_CURRENT_AIN_M1 PC_1
#define MINI_SEGWAY_CURRENT_AIN_M2 PB_0
#define MINI_SEGWAY_CURRENT_FILTER_FREQUENCY_HZ 10.0f
#define MINI_SEGWAY_CURRENT_FILTER_DAMPING (sqrtf(3.0f) / 2.0f)

// analog current sensor
#define MINI_SEGWAY_AIN_USE_ADDITIONAL_CURRENT_SENSOR false
#if MINI_SEGWAY_AIN_USE_ADDITIONAL_CURRENT_SENSOR
    #define MINI_SEGWAY_AIN_ADDITIONAL_M1 PA_0
    #define MINI_SEGWAY_AIN_ADDITIONAL_M2 PA_1
#endif