#ifndef MINI_SEGWAY_H_
#define MINI_SEGWAY_H_

#include "mbed.h"

#include "ThreadFlag.h"
#include "SerialStream.h"
#include "PpmIn.h"

// // openlager attached to NUCLEO_L432KC -> 20 floats
// #define RX PB_7 // unused, not even connected to openlager
// #define TX PB_6

// serial via usb to matlab NUCLEO_L432KC -> 20 floats
#define TX USBTX
#define RX USBRX
#define NUM_OF_FLOATS_MAX 10
#define BAUDRATE 2000000 // openlager runs at 2000000 baudrate

// additonal button for testing
#define BUTTON PB_0

// TBS CROSSFIRE Nano RX
#define PPM_IN PA_12

// period in micro seconds
// #define PERIOD_MUS 2000 // 500 Hz
#define PERIOD_MUS 500000 // 2 Hz

class MiniSegway
{
public:
    explicit MiniSegway();
    virtual ~MiniSegway();

private:
    // static constexpr int64_t PERIOD_MUS = 500000;

    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    void threadTask();
    void sendThreadFlag(); 
};
#endif /* MINI_SEGWAY_H_ */
