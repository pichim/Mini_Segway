#ifndef CONFIG_H_
#define CONFIG_H_

#define USE_NUCLEO_L432KC false

#if USE_NUCLEO_L432KC
    // task period
    // #define MINI_SEGWAY_PERIOD_US 2500   //  400 Hz
    // #define MINI_SEGWAY_PERIOD_US 2000   //  500 Hz
    #define MINI_SEGWAY_PERIOD_US 1000   // 1000 Hz
    // #define MINI_SEGWAY_PERIOD_US 500    // 2000Hz

    // streaming device, openlager or laptop / pc
    #define DO_USE_OPENLAGER_FOR_DATA_STREAM false
    // serial data stream, tested up to 20 floats at 2 kHz
    #if DO_USE_OPENLAGER_FOR_DATA_STREAM
        // openlager UART2
        // - this only works if you desolder the 0 Ohm resistor SB2 and SB3
        #define MINI_SEGWAY_TX PA_2
        #define MINI_SEGWAY_RX NC
    #else
        // serial via usb to matlab UART2
        #define MINI_SEGWAY_TX USBTX // PA_2
        // - receiving only works if you did not desolder the 0 Ohm resistor SB2 and SB3
        //   (default NUCLEO_L432KC configuration)
        #define MINI_SEGWAY_RX USBRX // PA_15
    #endif
    // openlager runs at 2000000 baudrate
    #define MINI_SEGWAY_BAUDRATE 2000000
    #define MINI_SEGWAY_NUM_OF_FLOATS 30

    // receiver to remote controll connection
    #define DO_USE_PPM_IN false
    #if DO_USE_PPM_IN
        // tbs crossfire nano rx, running at 50 Hz := 20000 mus
        #define MINI_SEGWAY_RC_DI PB_0
        #define MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG (10 * (20000 / MINI_SEGWAY_PERIOD_US + 1))
        #define MINI_SEGWAY_RC_ARMING_CHANNEL 4
    #else
        // radiomaster elrs rx, running at 111 Hz := ~9000 mus
        #define MINI_SEGWAY_RC_TX NC // not even connected
        #define MINI_SEGWAY_RC_RX PA_10
        #define MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG (10 * (9000 / MINI_SEGWAY_PERIOD_US + 1))
        #define MINI_SEGWAY_RC_ARMING_CHANNEL 7
    #endif
    #define MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG
    #define MINI_SEGWAY_RC_UPSAMPLING_FREQUENCY_HZ 30.0f

    // additonal button
    #define MINI_SEGWAY_BUTTON PB_7

    // additional led
    #define MINI_SEGWAY_LED PA_12

    // encoders
    #define MINI_SEGWAY_ENCA_M1 PA_8 // working
    #define MINI_SEGWAY_ENCB_M1 PA_9
    // #define MINI_SEGWAY_ENCA_M2 PA_5 // not working
    // #define MINI_SEGWAY_ENCB_M2 PA_1
    // https://www.pololu.com/product/3053/specs
    #define MINI_SEGWAY_COUNTS_PER_TURN (150.58f * 12.0f)
    // #define MINI_SEGWAY_KN (220.0f / 12.0f) // 150:1 Micro Metal Gearmotor HPCB 12V with Extended Motor Shaft
    #define MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY 15.0f
    #define MINI_SEGWAY_VOLTAGE_MAX 9.0f

    // pwm
    #define MINI_SEGWAY_ENABLE_MOTOR_DRIVER PB_5
    #define MINI_SEGWAY_PWM_M1 PA_6
    #define MINI_SEGWAY_PWM_M2 PA_3
    #define MINI_SEGWAY_PWM_MIN_VALUE 0.01f
    #define MINI_SEGWAY_PWM_MAX_VALUE 0.99f

    // TODO assign proper pins

    // // IMU
    // #define DO_USE_EXTERNAL_IMU true

    // #if DO_USE_EXTERNAL_IMU
    //     // imu
    //     #define MINI_SEGWAY_IMU_MOSI
    //     #define MINI_SEGWAY_IMU_MISO
    //     #define MINI_SEGWAY_IMU_CLK
    // #else
    //     // imu
    //     #define MINI_SEGWAY_IMU_SDA PB_9
    //     #define MINI_SEGWAY_IMU_SCL PA_7
    // #endif

#else
       // task period
    // #define MINI_SEGWAY_PERIOD_US 2500   //  400 Hz
    // #define MINI_SEGWAY_PERIOD_US 2000   //  500 Hz
    #define MINI_SEGWAY_PERIOD_US 1000   // 1000 Hz
    // #define MINI_SEGWAY_PERIOD_US 500    // 2000Hz

    // streaming device, openlager or laptop / pc
    #define DO_USE_OPENLAGER_FOR_DATA_STREAM false
    // serial data stream, tested up to 20 floats at 2 kHz
    #if DO_USE_OPENLAGER_FOR_DATA_STREAM
        // openlager UART2
        #define MINI_SEGWAY_TX PA_0
        #define MINI_SEGWAY_RX NC
    #else
        // serial via usb to matlab UART2
        #define MINI_SEGWAY_TX USBTX // PA_2
        // - receiving only works if you did not desolder the 0 Ohm resistor SB2 and SB3
        //   (default NUCLEO_F446RE configuration)
        #define MINI_SEGWAY_RX USBRX // PA_3
    #endif
    // openlager runs at 2000000 baudrate
    #define MINI_SEGWAY_BAUDRATE 2000000
    #define MINI_SEGWAY_NUM_OF_FLOATS 30

    // receiver to remote controll connection
    // radiomaster elrs rx, running at 111 Hz := ~9000 mus
    #define MINI_SEGWAY_RC_TX NC // not even connected
    #define MINI_SEGWAY_RC_RX PA_10
    #define MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG (10 * (9000 / MINI_SEGWAY_PERIOD_US + 1))
    #define MINI_SEGWAY_RC_ARMING_CHANNEL 7

    #define MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG
    #define MINI_SEGWAY_RC_UPSAMPLING_FREQUENCY_HZ 30.0f

    // additonal button
    #define MINI_SEGWAY_BUTTON PC_5

    // additional led
    #define MINI_SEGWAY_LED PB_9

    // encoders
    #define MINI_SEGWAY_ENCA_M1 PA_6 // working
    #define MINI_SEGWAY_ENCB_M1 PC_7
    #define MINI_SEGWAY_ENCA_M2 PB_6 // working
    #define MINI_SEGWAY_ENCB_M2 PB_7
    // https://www.pololu.com/product/3487/specs
    #define MINI_SEGWAY_COUNTS_PER_TURN (31.25f * 20.0f)
    #define MINI_SEGWAY_KN (450.0f / 12.0f) // 31:1 Metal Gearmotor 20Dx41L mm 12V CB with Extended Motor Shaft
    #define MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY 20.0f
    #define MINI_SEGWAY_VOLTAGE_MAX 6.0f

    // pwm
    #define MINI_SEGWAY_ENABLE_MOTOR_DRIVER PB_15
    #define MINI_SEGWAY_PWM_M1 PB_13
    #define MINI_SEGWAY_PWM_M2 PA_9
    #define MINI_SEGWAY_PWM_MIN_VALUE 0.01f
    #define MINI_SEGWAY_PWM_MAX_VALUE 0.99f

    // imu
    #define MINI_SEGWAY_IMU_MOSI PC_3
    #define MINI_SEGWAY_IMU_MISO PC_2
    #define MINI_SEGWAY_IMU_CLK PB_10
    #define MINI_SEGWAY_IMU_CS PB_4

    
#endif

#endif /* CONFIG_H_ */