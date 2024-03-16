#include "mbed.h"

#include "EncoderCounter.h"

// https://github.com/altb71/mini_cuboid/blob/master/Lib_Misc/EncoderCounter.h
#define ENC_TIM1_CH1 PA_8
#define ENC_TIM1_CH2 PA_9
// hurc
#define ENC_TIM2_CH1 PA_0
#define ENC_TIM2_CH2 PA_1

int main()
{
    EncoderCounter encoderCounter_M1(ENC_TIM1_CH1, ENC_TIM1_CH2);
    EncoderCounter encoderCounter_M2(ENC_TIM2_CH1, ENC_TIM2_CH2);
    encoderCounter_M1.reset();
    encoderCounter_M2.reset();

    DigitalOut led1(LED1);
    printf("starting...\n");
    while (true) {

        printf("M1: %d\n", encoderCounter_M1.read());
        printf("M2: %d\n", encoderCounter_M2.read());

        led1 = !led1;
        thread_sleep_for(100);
    }
}