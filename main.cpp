#include "mbed.h"

/**
 * TODO:
 * - move serialStream and remoteCntrl to MiniSegway and remove option of SBus as thread
 * 
 * - check for all threads the destructor
 *  _Timeout.detach();
    _Ticker.detach();
    _Thread.terminate();
*
* - check for all const functions possible move to header
*
* - move all unused destructors to header
*
* - check all defines to have the header name first
*
* - check serial stream and usage of serial pipe and 
*   buffered serial again according to sbus
*/

// MiniSegway includes PpmIn and SBus
#include "MiniSegway.h"

SerialStream serialStream(MINI_SEGWAY_NUM_OF_FLOATS,
                          MINI_SEGWAY_TX,
                          MINI_SEGWAY_RX,
                          MINI_SEGWAY_BAUDRATE);
#if DO_USE_PPM_IN
PpmIn remoteCntrl(MINI_SEGWAY_RC_DI);    
#else
SBus remoteCntrl(MINI_SEGWAY_RC_RX);
#endif
MiniSegway miniSegway(remoteCntrl,
                      serialStream);


// main thread is just blinking the led on the nucleo
int main()
{
    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1;
        thread_sleep_for(1000);
    }
}