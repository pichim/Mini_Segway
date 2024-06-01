#include "mbed.h"

/**
 * TODO new (F446RE):
 * - Run DCMotor not as thread.
 * - imu internal filters need to be checked
 * - after checking the internal filter, mahony needs to be tuned
 * - think about acc calibration when we use the above signal order (might need to be static)
 *   function in minisegway directly
 * - we might need an external mechanical button
 * - we might need an external power switch
 * - reset via button needs to work properly (for all variables, obj, etc.)
*/

/**
 * TODO move tooling:
 * - mahonyRP.m and eval file
*/

/**
 * TODO old:
 * - check for all threads the destructor
 *      _Timeout.detach();
 *      _Ticker.detach();
 *      _Thread.terminate();
 * - check for all const functions possible move to header
 * - move all unused destructors to header
 * - check all defines to have the header name first
*/

#include "IMU.h"
#include "MiniSegway.h"
#include "RC.h"

RC rc(MINI_SEGWAY_RC_RX);
IMU imu(MINI_SEGWAY_IMU_MOSI,
        MINI_SEGWAY_IMU_MISO,
        MINI_SEGWAY_IMU_CLK,
        MINI_SEGWAY_IMU_CS);
MiniSegway miniSegway(rc, imu);

// main thread is just blinking the led on the nucleo
int main()
{
    printf("MiniSegway running...\n");
    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1;
        thread_sleep_for(1000);
    }
}