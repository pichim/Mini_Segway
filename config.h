#ifndef CONFIG_H_
#define CONFIG_H_

// MPU6500 runs at 1kHz, so we want to run the control loop at 1kHz

// task period
// #define MINI_SEGWAY_PERIOD_US 10000  //   100 Hz
// #define MINI_SEGWAY_PERIOD_US 2500   //   400 Hz
// #define MINI_SEGWAY_PERIOD_US 2000   //   500 Hz
#define MINI_SEGWAY_PERIOD_US 1000   //  1000 Hz
// #define MINI_SEGWAY_PERIOD_US 500    //  2000 Hz
// #define MINI_SEGWAY_PERIOD_US 333    //  3000 Hz
// #define MINI_SEGWAY_PERIOD_US 200    //  5000 Hz
// #define MINI_SEGWAY_PERIOD_US 150    //  6666 Hz
// #define MINI_SEGWAY_PERIOD_US 133    //  7500 Hz
// #define MINI_SEGWAY_PERIOD_US 100    // 10000 Hz
// #define MINI_SEGWAY_PERIOD_US  50    // 20000 Hz

// sampling time
#define MINI_SEGWAY_TS (static_cast<float>(MINI_SEGWAY_PERIOD_US) * 1.0e-6f)

// streaming device, openlager or laptop / pc
#define DO_USE_OPENLAGER_FOR_DATA_STREAM false
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
    #define MINI_SEGWAY_TX PA_0
    #define MINI_SEGWAY_RX PA_1
#endif
// openlager runs at 2000000 baudrate
#define MINI_SEGWAY_BAUDRATE 2000000
#define MINI_SEGWAY_NUM_OF_FLOATS 30 // tested up to 20 floats at 2 kHz, so 30 floats at 1 kHz should work

// remote control receiver
// radiomaster elrs rx, running at 111 Hz := ~9000 mus
#define MINI_SEGWAY_RC_TX NC // not connected
#define MINI_SEGWAY_RC_RX PA_10
#define MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG (10 * (9000 / MINI_SEGWAY_PERIOD_US + 1))
#define MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG
#define MINI_SEGWAY_RC_ARMING_CHANNEL 7
#define MINI_SEGWAY_RC_USE_UPSAMPLING_FILTERS true
#define MINI_SEGWAY_RC_UPSAMPLING_DAMPING (sqrtf(3.0f) / 2.0f)
#define MINI_SEGWAY_RC_UPSAMPLING_FREQUENCY_HZ 20.0f
#define MINI_SEGWAY_RC_APPLY_EXPO true
#define MINI_SEGWAY_RC_EXPO_ALPHA 2.3f

// button
#define MINI_SEGWAY_BUTTON BUTTON1 // blue button
// #define MINI_SEGWAY_BUTTON PC_5 // additonal button

// additional led
#define MINI_SEGWAY_LED PA_7

// encoders
#define MINI_SEGWAY_ENCA_M1 PA_6
#define MINI_SEGWAY_ENCB_M1 PC_7
#define MINI_SEGWAY_ENCA_M2 PB_6
#define MINI_SEGWAY_ENCB_M2 PB_7

// motors
#define MINI_SEGWAY_GEAR_RATIO 46.85f
#define MINI_SEGWAY_COUNTS_PER_TURN (48.0f * MINI_SEGWAY_GEAR_RATIO)
#define MINI_SEGWAY_KN (170.0f / 12.0f) // 31:1 Metal Gearmotor 20Dx41L mm 12V CB with Extended Motor Shaft
#define MINI_SEGWAY_VOLTAGE_MAX 12.0f
#define MINI_SEGWAY_VELOCITY_FILTER_DAMPING (sqrtf(3.0f) / 2.0f)
#define MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY 20.0f
#define MINI_SEGWAY_VEL_CNTRL_KP 0.2f * 1.2f * 4.2f   // 0.2f so that it is stable (but slow)
#define MINI_SEGWAY_VEL_CNTRL_KI 0.2f * 1.1f * 140.0f // 0.2f so that it is stable (but slow)
#define MINI_SEGWAY_VEL_CNTRL_KD 1.1f * 0.0192f;

// pwm
// if you want to switch the names you also need to adjust the following code in MiniSegway.cpp:
// // invert polarity of pwms
// TIM2->CCER |= TIM_CCER_CC2P; // invert polarity of pwm on PB_9, PWM2/2 : TIM2_CH2
// TIM1->CCER |= TIM_CCER_CC2P; // invert polarity of pwm on PA_9, PWM1/2 : TIM1_CH2
#define MINI_SEGWAY_PWM_PERIOD_US 200 // 5 kHz
#define MINI_SEGWAY_PWM_M1_POS PA_15 // PWM2/1 : TIM2_CH1
#define MINI_SEGWAY_PWM_M1_NEG PB_9  // PWM2/2 : TIM2_CH2
#define MINI_SEGWAY_PWM_M2_POS PA_8  // PWM1/1 : TIM1_CH1
#define MINI_SEGWAY_PWM_M2_NEG PA_9  // PWM1/2 : TIM1_CH2
#define MINI_SEGWAY_PWM_MIN_VALUE 0.001f
#define MINI_SEGWAY_PWM_MAX_VALUE 0.999f

// imu
#define MINI_SEGWAY_IMU_MOSI PC_3
#define MINI_SEGWAY_IMU_MISO PC_2
#define MINI_SEGWAY_IMU_CLK PB_10
#define MINI_SEGWAY_IMU_CS PB_4
#define MINI_SEGWAY_IMU_USE_ADDITIONAL_FILTERS true
#define MINI_SEGWAY_IMU_GYRO_FREQUENCY_HZ 60.0f
#define MINI_SEGWAY_IMU_ACC_FREQUENCY_HZ 15.0f
#define MINI_SEGWAY_IMU_DO_USE_STATIC_ACC_CALIBRATION false // if this is false then acc gets averaged at the beginning and printed to the console
#define MINI_SEGWAY_IMU_ROTATE_SIGNALS_SEGWAY_STANDING false
// imu acc bias and mahony gains
#define MINI_SEGWAY_IMU_B_ACC {0.0f, 0.0f, 0.0f}
// % bessel (D = sqrt(3)/2)
// w0 = 3;
// kp = w0 / ( sqrt(3)/3 )
// ki = kp^2 / 3
// % real pole, no integrator, use this if you dont use the mag
// w0 = 3;
// kp = w0;
// ki = 0;
#define MINI_SEGWAY_IMU_KP_XY (0.1592f * 2.0f * M_PIf)
// #define MINI_SEGWAY_IMU_KP_XY (0.1592f * 2.0f * M_PIf / (sqrtf(3.0f) / 3.0f))
#define MINI_SEGWAY_IMU_KP_Z  (0.1592f * 2.0f * M_PIf)
#define MINI_SEGWAY_IMU_KI_XY 0.0f
// #define MINI_SEGWAY_IMU_KI_XY (MINI_SEGWAY_IMU_KP_XY * MINI_SEGWAY_IMU_KP_XY / 3.0f)
#define MINI_SEGWAY_IMU_KI_Z  0.0f

// robot kinematics
#define R_WHEEL 0.039f // wheel radius in meters
#define B_WHEEL 0.125f // wheelbase, distance from wheel to wheel in meters
// math is design in the way that left turn is with the possitive sign
// rc controller is sending negative sign when turning left with stick
// thats why for the maths we need to multipline rc controller output by -1
// #define TURN_RATIO -5.0f

// chirp signal
#define MINI_SEGWAY_CHIRP_USE_CHIRP false
#if MINI_SEGWAY_CHIRP_USE_CHIRP
    #define MINI_SEGWAY_CHIRP_T1 20.0f
    #define MINI_SEGWAY_CHIRP_F0 (1.0f / MINI_SEGWAY_CHIRP_T1)
    #define MINI_SEGWAY_CHIRP_F1 (1.0f / (2.0f * MINI_SEGWAY_TS))
    #define MINI_SEGWAY_CHIRP_AMPLITUDE 2.0f
    #define MINI_SEGWAY_CHIRP_OFFSET 3.5f
#endif

// analog current sensor
#define MINI_SEGWAY_AIN_USE_CURRENT_SENSOR false
#if MINI_SEGWAY_AIN_USE_CURRENT_SENSOR
    #define MINI_SEGWAY_AIN_1 PB_0
    #define MINI_SEGWAY_AIN_2 PC_1
#endif

#endif /* CONFIG_H_ */