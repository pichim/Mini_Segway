#include "mbed.h"

#include "DebounceIn.h"
#include "MiniSegway.h"
#include "RC.h"

// Notes:
// - Acc signal has a clear offset, this was different via spi with the imu breakout board

// TODOs:
// - check motor constant kn                                     ok, measured
// - check motor cpr at gear end                                 ok
// - check motor pwm                                             ok, increased from 5 kHz to 20 kHz (now at max)
// - remove pes_board_pinmap.h (maybe) for what ever reason      can't, otherwise Camille is mad
// - check imu filter configuration                              ok, according to the code, did not check spectras
// - check imu alignment                                         ok, adjusted based on measurement
// - check imu units                                             ok, adjusted based on measurement
// - maybe sampling time needs to be adjusted to 2 ms (500 Hz)   ok, we leave it at 1 kHz
// - check sd card logging
// - check the sign of all data when logging while driving
// - adjust controller parameters
// - check all TODOs
// - update README.md

RC rc(MINI_SEGWAY_RC_UART_RX); // rc needs to be declared here and passed to miniSegway
MiniSegway miniSegway(rc);

int main()
{
    while (true) {
        thread_sleep_for(1000);
    }
}
