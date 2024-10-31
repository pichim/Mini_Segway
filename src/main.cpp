// /home/pichim/.platformio/packages/framework-mbed/platformio/package_deps/py3/past/builtins/misc.py
// 45: # from imp import reload
// 46: from importlib import reload
// 
// source .platformio/penv/bin/activate
// pip install setuptools
// -  Python 3.8.10
// 
// - framework-mbed @ 6.61700.231105 (6.17.0) 
// - toolchain-gccarmnoneeabi @ 1.90201.191206 (9.2.1)

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