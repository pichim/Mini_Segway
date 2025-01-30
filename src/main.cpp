// Notes:
// - Last updated and tested on 30.01.2025
// - PlatformIO Build is not working, but does not matter. Simply flash by drag and drop.
//

// TODO:
// - Check latest changes to SBus and RC according to ROS1 package: https://github.com/rtlab-ims-pub/elrs_sbus_teleop

// Processing nucleo_f446re (platform: ststm32; board: nucleo_f446re; framework: mbed)
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Verbose mode can be enabled via `-v, --verbose` option
// CONFIGURATION: https://docs.platformio.org/page/boards/ststm32/nucleo_f446re.html
// PLATFORM: ST STM32 (17.6.0) > ST Nucleo F446RE
// HARDWARE: STM32F446RET6 180MHz, 128KB RAM, 512KB Flash
// DEBUG: Current (stlink) On-board (stlink) External (blackmagic, cmsis-dap, jlink)
// PACKAGES: 
//  - framework-mbed @ 6.61700.231105 (6.17.0) 
//  - toolchain-gccarmnoneeabi @ 1.90201.191206 (9.2.1)
// Collecting mbed sources...
// LDF: Library Dependency Finder -> https://bit.ly/configure-pio-ldf
// LDF Modes: Finder ~ chain, Compatibility ~ soft
// Found 19 compatible libraries
// Scanning dependencies...
// Dependency Graph
// |-- Eigen @ 0.0.0+sha.13a5d365ba16
// |-- DebounceIn
// |-- MiniSegway
// |-- RC
// Building in release mode


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