#include "mbed.h"

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