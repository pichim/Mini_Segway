#pragma once

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

#include "pes_board_pinmap.h"

// currently used hardware
// - 1 x openlager (UART)                                              (ok)
// - 1 x serial via programming port to laptop / pc (UART)             (ok)
// - 1 x serial via usb 2.0-cable TTL serial 6 pin to computer (UART)  (ok)
// - 1 x remote control receiver, radiomaster elrs (UART)              (ok)
// - 2 x additional buttons (enable, reset) (GPIOs)                    (ok)
// - 2 x additional leds (DOUT)                                        (ok)
// - 2 x encoder                                                       (ok)
// - 1 X enable motor driver (GPIO)                                    (ok)
// - 2 x motor pwm (PWM)                                               (ok)
// - 2 x motor direction (DOUT)                                        (ok)
// - 1 x imu (SPI)                                                     (ok)
// - 2 x motor current sensor (AIN)                                    (ok)
// - 1 x gimbal servo (DOUT)                                           (ok)

// task period, MPU6500 runs at 1kHz, so we want to run the control loop at 1kHz
#define MINI_SEGWAY_PERIOD_US 1000
#define MINI_SEGWAY_TS (static_cast<float>(MINI_SEGWAY_PERIOD_US) * 1.0e-6f) // sampling time
#define MINI_SEGWAY_ABS_ANGLE_START_BALANCE_RAD (20.0f * M_PIf / 180.0f)
#define MINI_SEGWAY_ABS_ANGLE_STOP_BALANCE_RAD (60.0f * M_PIf / 180.0f)
#define MINI_SEGWAY_MIXER_GAIN 0.8f // e.g. 0.7f means 70% of the control signal is used forward speed and 30% for turning
#define MINI_SEGWAY_SCALE_SPEED_MAX_FAST 1.0f
#define MINI_SEGWAY_SCALE_SPEED_MAX_SLOW 0.6f
#define MINI_SEGWAY_R_WHEEL 0.039f // wheel radius in meters
#define MINI_SEGWAY_B_WHEEL 0.125f // wheelbase, distance from wheel to wheel in meters

// state space controller gains
#define MINI_SEGWAY_FORWARD_CPD_ANG_KP 2.64f
#define MINI_SEGWAY_FORWARD_CPD_ANG_KD 0.05f
#define MINI_SEGWAY_FORWARD_CPD_ANG_D_FILTER_FREQUENCY_HZ 3.0f
#define MINI_SEGWAY_FORWARD_CPD_VEL_KP 2.2f
#define MINI_SEGWAY_FORWARD_CPD_VEL_KD 0.05f
#define MINI_SEGWAY_FORWARD_CPD_VEL_D_FILTER_FREQUENCY_HZ 1.0f
#define MINI_SEGWAY_FORWARD_CP_POS_KP 2.86f
#define MINI_SEGWAY_TURN_CP_POS_KP 10.0f

// streaming device, openlager or laptop / pc
#define DO_USE_OPENLAGER_FOR_DATA_STREAM true
// serial data stream, tested up to 20 floats at 2 kHz
#if DO_USE_OPENLAGER_FOR_DATA_STREAM
    // openlager
    #define MINI_SEGWAY_UART_TX PB_UART3_TX // UART6_TX
    #define MINI_SEGWAY_UART_RX NC
#else
    // // serial via usb to matlab
    // #define MINI_SEGWAY_UART_TX USBTX // PA_2, UART2_TX
    // #define MINI_SEGWAY_UART_RX USBRX // PA_3, UART2_RX
    // usb 2.0-cable TTL serial 6 pin to computer
    #define MINI_SEGWAY_UART_TX PB_UART3_TX // UART3_TX (PC_10)
    #define MINI_SEGWAY_UART_RX PB_UART3_RX // UART3_RX (PC_11)
#endif
// openlager runs at 2000000 baudrate
#define MINI_SEGWAY_BAUDRATE 2000000
#define MINI_SEGWAY_NUM_OF_FLOATS 30 // tested up to 20 floats at 2 kHz, so 30 floats at 1 kHz should work

// remote control receiver, radiomaster elrs rx, running at 111 Hz := ~9000 mus
#define MINI_SEGWAY_RC_UART_TX NC
#define MINI_SEGWAY_RC_UART_RX PB_UART5_RX // UART2_RX
#define MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG (10 * (9000 / MINI_SEGWAY_PERIOD_US + 1))
#define MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG
#define MINI_SEGWAY_RC_ARMING_CHANNEL 7        // top right switch
#define MINI_SEGWAY_RC_MODE_CHANNEL 4          // top left switch
#define MINI_SEGWAY_RC_TURN_RATE_CHANNEL 0     // right stick left to right
#define MINI_SEGWAY_RC_FORWARD_SPEED_CHANNEL 2 // left stick down to up
#define MINI_SEGWAY_RC_USE_UPSAMPLING_FILTERS true
#define MINI_SEGWAY_RC_UPSAMPLING_FILTER_DAMPING 1.0f
#define MINI_SEGWAY_RC_FORWARD_SPEED_UPSAMPLING_FILTER_FREQUENCY_HZ 30.0f
#define MINI_SEGWAY_RC_TURN_RATE_UPSAMPLING_FILTER_FREQUENCY_HZ 30.0f
#define MINI_SEGWAY_RC_APPLY_EXPO true
#define MINI_SEGWAY_RC_EXPO_ALPHA 2.3f

// button
#define MINI_SEGWAY_BLUE_BUTTON_GPIO     PB_DIO_3 // PC_13, blue button
#define MINI_SEGWAY_ADD_BLUE_BUTTON_GPIO PB_DIO_2 // additional blue button

// additional leds
#define MINI_SEGWAY_LED_PERIOD_US 250000
#define MINI_SEGWAY_LED1_DOUT PB_DIO_0 // blue
#define MINI_SEGWAY_LED2_DOUT PB_DIO_1 // green

// encoders
#define MINI_SEGWAY_MOTOR1_ENCA PB_DC_MOT1_ENC_A
#define MINI_SEGWAY_MOTOR1_ENCB PB_DC_MOT1_ENC_B
#define MINI_SEGWAY_MOTOR2_ENCA PB_DC_MOT2_ENC_A
#define MINI_SEGWAY_MOTOR2_ENCB PB_DC_MOT2_ENC_B

// motors
#define MINI_SEGWAY_MOTOR_GEAR_RATIO (4554.0f / 130.0f)
#define MINI_SEGWAY_MOTOR_COUNTS_PER_TURN (4.0f * 1024.0f * MINI_SEGWAY_MOTOR_GEAR_RATIO)
#define MINI_SEGWAY_MOTOR_KN (224.0f / 12.0f) // measured by pmic 20.05.2025
#define MINI_SEGWAY_MOTOR_VOLTAGE_MAX 12.0f
#define MINI_SEGWAY_MOTOR_VELOCITY_FILTER_DAMPING 1.0f
#define MINI_SEGWAY_MOTOR_VELOCITY_FILTER_FREQUENCY_HZ 3.0f

// motor driver (h-bridge)
#define MINI_SEGWAY_ENABLE_MOTOR_DRIVER_GPIO PB_DC_MOT_ENABLE

// pwm
#define MINI_SEGWAY_MOTOR1_PWM          PB_DC_MOT1_PWM
#define MINI_SEGWAY_MOTOR1_PWM_DIR_DOUT PB_DC_MOT1_DIR
#define MINI_SEGWAY_MOTOR2_PWM          PB_DC_MOT2_PWM
#define MINI_SEGWAY_MOTOR2_PWM_DIR_DOUT PB_DC_MOT2_DIR
#define MINI_SEGWAY_MOTOR_PWM_PERIOD_US 50
#define MINI_SEGWAY_MOTOR_PWM_MIN_VALUE 0.01f
#define MINI_SEGWAY_MOTOR_PWM_MAX_VALUE 0.99f

// imu
#define MINI_SEGWAY_IMU_SDA PB_I2C_SDA
#define MINI_SEGWAY_IMU_SCL PB_I2C_SCL
#define MINI_SEGWAY_IMU_USE_ADDITIONAL_FILTERS true
#define MINI_SEGWAY_IMU_GYRO_FILTER_FREQUENCY_HZ 60.0f
#define MINI_SEGWAY_IMU_ACC_FILTER_FREQUENCY_HZ 60.0f
#define MINI_SEGWAY_IMU_NUM_RUNS_SKIP 1000 // dont make this shorter than 1000 micro seconds, the openlager needs 1 second to start up
#define MINI_SEGWAY_IMU_NUM_RUNS_FOR_AVERAGE 1000
#define MINI_SEGWAY_IMU_DO_USE_STATIC_ACC_CALIBRATION true // if this is true then averages acc gets overwritten by MINI_SEGWAY_IMU_B_ACC
#define MINI_SEGWAY_IMU_B_ACC {0.0f, 0.0f, 0.0f}
#define MINI_SEGWAY_IMU_KP_XY (0.1592f * 2.0f * M_PIf)
#define MINI_SEGWAY_IMU_KP_Z  (0.1592f * 2.0f * M_PIf)
#define MINI_SEGWAY_IMU_KI_XY 0.0f
#define MINI_SEGWAY_IMU_KI_Z  0.0f

// chirp signal
#define MINI_SEGWAY_CHIRP_USE_CHIRP false
#if MINI_SEGWAY_CHIRP_USE_CHIRP
    #define MINI_SEGWAY_CHIRP_T1 60.0f
    #define MINI_SEGWAY_CHIRP_F0 0.5f //(1.0f / MINI_SEGWAY_CHIRP_T1)
    #define MINI_SEGWAY_CHIRP_F1 100.0f //(0.95f / (2.0f * MINI_SEGWAY_TS))
    #define MINI_SEGWAY_CHIRP_AMPLITUDE 0.07f
    #define MINI_SEGWAY_CHIRP_OFFSET 0.0f
#endif

// current sensors from h-bridge
#define MINI_SEGWAY_CURRENT_MOTOR1_AIN PB_DC_MOT1_FB
#define MINI_SEGWAY_CURRENT_MOTOR2_AIN PB_DC_MOT2_FB
#define MINI_SEGWAY_CURRENT_FILTER_FREQUENCY_HZ 10.0f
#define MINI_SEGWAY_CURRENT_FILTER_DAMPING (sqrtf(3.0f) / 2.0f)

// analog current sensor (this is additional to the current sensors from the h-bridge and not used in the MiniSegway)
#define MINI_SEGWAY_AIN_USE_ADDITIONAL_CURRENT_SENSOR false
#if MINI_SEGWAY_AIN_USE_ADDITIONAL_CURRENT_SENSOR
    #define MINI_SEGWAY_AIN_ADDITIONAL_M1 PA_0
    #define MINI_SEGWAY_AIN_ADDITIONAL_M2 PA_1
#endif

// gimbal servo (can only be used if no analog current sensor is used, see above)
#define MINI_SEGWAY_SERVO_DOUT PB_AIN_3
#define MINI_SEGWAY_SERVO_PERIOD_US 20000
#define MINI_SEGWAY_SERVO_VALUE_MIN 0.0397f // maps approx. to -97 deg
#define MINI_SEGWAY_SERVO_VALUE_MAX 0.0586f // maps approx. to  97 deg
#define MINI_SEGWAY_SERVO_VALUE_RAD_MAX (97.0f * M_PIf / 180.0f)
#define MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX (60.0f * M_PIf / 180.0f)
#define MINI_SEGWAY_SERVO_ANGLE_OFFSET_RAD (0.0f * M_PIf / 180.0f)
#define MINI_SEGWAY_SERVO_ANGLE_FILTER_FREQUENCY_HZ 0.7f
