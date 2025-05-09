#ifndef PES_BOARD_PINMAP_H_
#define PES_BOARD_PINMAP_H_

#define PB_DC_MOT1_DIR      PB_14
#define PB_DC_MOT1_PWM      PB_15
#define PB_DC_MOT1_ENC_A    PA_6
#define PB_DC_MOT1_ENC_B    PC_7
#define PB_DC_MOT1_FB       PA_5

#define PB_DC_MOT2_DIR      PB_10
#define PB_DC_MOT2_PWM      PA_9
#define PB_DC_MOT2_ENC_A    PB_6
#define PB_DC_MOT2_ENC_B    PB_7
#define PB_DC_MOT2_FB       PA_7

#define PB_DC_MOT3_DIR      PC_4
#define PB_DC_MOT3_PWM      PB_13
#define PB_DC_MOT3_ENC_A    PA_0
#define PB_DC_MOT3_ENC_B    PA_1
#define PB_DC_MOT3_FB       PB_1

#define PB_DC_MOT_ENABLE    PB_9
#define PB_DC_MOT_STATE     PH_1

#define PB_STEPPER1_STEP    PB_12
#define PB_STEPPER1_DIR     PB_2

#define PB_STEPPER2_STEP    PC_6
#define PB_STEPPER2_DIR     PC_5

#define PB_STEPPER_ENABLE   PC_8
#define PB_STEPPER_RESET    PB_8

#define PB_AIN_0            PC_3
#define PB_AIN_1            PC_0
#define PB_AIN_2            PC_2
#define PB_AIN_3            PC_1

#define PB_DIO_0            PA_4
#define PB_DIO_1            PB_0
#define PB_DIO_2            PA_15
#define PB_DIO_3            PC_13

#define PB_SERVO_0          PB_DIO_0
#define PB_SERVO_1          PB_DIO_1
#define PB_SERVO_2          PB_DIO_2
#define PB_SERVO_3          PB_DIO_3

#define PB_I2C_SDA          PC_9
#define PB_I2C_SCL          PA_8

#define PB_CAN_TD           PA_12
#define PB_CAN_RD           PA_11

#define PB_SDCARD_MOSI      PB_5
#define PB_SDCARD_MISO      PB_4
#define PB_SDCARD_SCLK      PB_3
#define PB_SDCARD_CS        PA_10

#define PB_UART3_TX         PC_10
#define PB_UART3_RX         PC_11

#define PB_UART5_TX         PC_12
#define PB_UART5_RX         PD_2

#define PB_USER_BUTTON      PH_0

#endif /* #ifndef PES_BOARD_PINMAP_H_ */