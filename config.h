#ifndef CONFIG_H_
#define CONFIG_H_

// IMPORTANT: 20000 has to be a multiple of PERIOD_MUS
// period in micro seconds
// #define PERIOD_MUS 20000  //   50 Hz
// #define PERIOD_MUS 10000  //  100 Hz
// #define PERIOD_MUS 5000   //  200 Hz
// #define PERIOD_MUS 2000   //  500 Hz
#define PERIOD_MUS 500    // 2000Hz


// streaming device, openlager or laptop / pc
#define DO_USE_OPENLAGER_FOR_DATA_STREAM false
// serial data stream, tested up to 20 floats at 2 kHz
#if DO_USE_OPENLAGER_FOR_DATA_STREAM
    // openlager
    #define RX NC   // PB_7 not even connected to openlager
    #define TX PB_6
#else
    // serial via usb to matlab
    #define TX USBTX // PA_2  on NUCLEO_L432KC
    #define RX USBRX // PA_15
#endif
// openlager runs at 2000000 baudrate
#define BAUDRATE 2000000


// additonal button
#define BUTTON PB_7


// tbs crossfire nano rx, running at 50 Hz := 20000 mus
#define PPM_IN PB_0
#define NUM_OF_ALLOWED_INVALID_RC_DATA_PKG 5 * (20000 / PERIOD_MUS)


// additional led
#define LED PA_12

#endif /* CONFIG_H_ */