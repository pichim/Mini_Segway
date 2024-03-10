#ifndef CONFIG_H_
#define CONFIG_H_

// task period
// #define MINI_SEGWAY_PERIOD_US 2000   //  500 Hz
// #define MINI_SEGWAY_PERIOD_US 1000   // 1000 Hz
#define MINI_SEGWAY_PERIOD_US 500    // 2000Hz

// streaming device, openlager or laptop / pc
#define DO_USE_OPENLAGER_FOR_DATA_STREAM true
// serial data stream, tested up to 20 floats at 2 kHz
#if DO_USE_OPENLAGER_FOR_DATA_STREAM
    // openlager
    #define MINI_SEGWAY_TX PB_6
    #define MINI_SEGWAY_RX NC
#else
    // serial via usb to matlab
    #define MINI_SEGWAY_TX USBTX // PA_2
    #define MINI_SEGWAY_RX USBRX // PA_15
#endif
// openlager runs at 2000000 baudrate
#define MINI_SEGWAY_BAUDRATE 2000000
#define MINI_SEGWAY_NUM_OF_FLOATS 20

// receiver to remote controll connection
#define DO_USE_PPM_IN true
#if DO_USE_PPM_IN
    // tbs crossfire nano rx, running at 50 Hz := 20000 mus
    #define MINI_SEGWAY_RC_DI PB_0
    #define MINI_SEGWAY_NUM_OF_ALLOWED_INVALID_RC_DATA_PKG 5 * (20000 / MINI_SEGWAY_PERIOD_US + 1)
    #define MINI_SEGWAY_ARMING_CHANNEL 4
#else
    // radiomaster elrs rx, running at 111 Hz := ~9000 mus
    #define MINI_SEGWAY_RC_TX NC // not even connected
    #define MINI_SEGWAY_RC_RX PA_10
    #define MINI_SEGWAY_NUM_OF_ALLOWED_INVALID_RC_DATA_PKG 5 * (9000 / MINI_SEGWAY_PERIOD_US + 1)
    #define MINI_SEGWAY_ARMING_CHANNEL 7
#endif

// additonal button
#define MINI_SEGWAY_BUTTON PB_7

// additional led
#define MINI_SEGWAY_LED PA_12

#endif /* CONFIG_H_ */