#include "mbed.h"

#include "DebounceIn.h"
#include "MiniSegway.h"
#include "RC.h"

// TODOs:
// - fix additional blue button
// - remove pes_board_pinmap.h (maybe) for what ever reason
// - check imu filter configuration
// - check imu alignment
// - maybe sampling time needs to be adjusted to 2 ms (500 Hz)
// - update and fix config.h, also NC, check TODO's
// - update README.md

RC rc(MINI_SEGWAY_RC_UART_RX); // rc needs to be declared here and passed to miniSegway
MiniSegway miniSegway(rc);

int main()
{
    // additional reset button
    // DebounceIn additionalResetButton(MINI_SEGWAY_RESET_BUTTON_GPIO, PullUp);
    // additionalResetButton.fall(&NVIC_SystemReset);

    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1; // main thread is just blinking the green led on the nucleo
        thread_sleep_for(1000);
    }
}
