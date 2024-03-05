#include "mbed.h"

#include "MiniSegway.h"

PpmIn ppmIn(PPM_IN);
MiniSegway miniSegway(ppmIn);

int main()
{
    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1;
        thread_sleep_for(1000);
    }
}