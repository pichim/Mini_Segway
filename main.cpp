#include "mbed.h"

/**
 * TODO new construction:
 * - check sbus, why is armed not always true when button is pressed? sometimes you have to disarm and arm it again...
 */

/**
 * TODO new construction:
 * - updating Hardware F446RE - Connection
 * - 2 additional buttons (reset and blue)
 * - 1 additional power switch
 * - nothing in the way or at least nut much shen plugging in a cable
 * - find out if current measurement from motor driver can be improved with additional cap, maybe talk to camille
 * - update hardwarelist_F446RE.txt
 * - move wheels as close to the body as possible (sunk srews) while maintaining thick enough walls (trade off)
 * - would be nice if the segway was level lying on one of the two sides when on a level ground
 * - test software filter for motor driver current
 */

/**
 * TODO new (F446RE):
 * - Figure out sign with Cw2r transformation.
 * - Adjust DCMotor class according to Motor class.
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


#include "DebounceIn.h"
#include "MiniSegway.h"
#include "RC.h"

RC rc(MINI_SEGWAY_RC_RX); // rc needs to be declared here and passed to miniSegway
MiniSegway miniSegway(rc);

int main()
{
    // additional reset button
    DebounceIn additionalResetButton(MINI_SEGWAY_RESET_BUTTON, PullUp);
    additionalResetButton.fall(&NVIC_SystemReset);

    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1; // main thread is just blinking the green led on the nucleo
        thread_sleep_for(1000);
    }
}