#include "mbed.h"

/**
 * TODO:
 * - move serialStream and rc to MiniSegway and remove option of SBus as thread
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
#include "IMU.h"
#include "MiniSegway.h"

#if DO_USE_PPM_IN
PpmIn rc(MINI_SEGWAY_RC_DI);
#else
SBus rc(MINI_SEGWAY_RC_RX);
#endif
IMU imu(MINI_SEGWAY_IMU_SDA,
        MINI_SEGWAY_IMU_SCL);
MiniSegway miniSegway(rc, imu);

// main thread is just blinking the led on the nucleo
int main()
{
    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1;
        thread_sleep_for(1000);
    }
}