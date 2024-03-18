#include "mbed.h"

#include "EncoderCounter.h"

/** 
    In order to use two encoders you need to change the following file in mbed-os:
    - mbed-os/targets/TARGET_STM/TARGET_STM32L4/us_ticker_data.h"

#ifndef __US_TICKER_DATA_H
#define __US_TICKER_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx.h"
#include "stm32l4xx_ll_tim.h"
#include "cmsis_nvic.h"

#if defined TIM5_BASE

#define TIM_MST      TIM5
#define TIM_MST_IRQ  TIM5_IRQn
#define TIM_MST_RCC  __HAL_RCC_TIM5_CLK_ENABLE()
#define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM5()

#define TIM_MST_RESET_ON   __HAL_RCC_TIM5_FORCE_RESET()
#define TIM_MST_RESET_OFF  __HAL_RCC_TIM5_RELEASE_RESET()

#define TIM_MST_BIT_WIDTH  32 // 16 or 32

#else

#define TIM_MST      TIM6
#define TIM_MST_IRQ  TIM6_IRQn
#define TIM_MST_RCC  __HAL_RCC_TIM6_CLK_ENABLE()
#define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM6()

#define TIM_MST_RESET_ON   __HAL_RCC_TIM6_FORCE_RESET()
#define TIM_MST_RESET_OFF  __HAL_RCC_TIM6_RELEASE_RESET()

#define TIM_MST_BIT_WIDTH  16 // 16 or 32

#endif

#define TIM_MST_PCLK  1 // Select the peripheral clock number (1 or 2)


#ifdef __cplusplus
}
#endif

#endif // __US_TICKER_DATA_H


*/

#define ENC_TIM1_CH1 PA_8
#define ENC_TIM1_CH2 PA_9
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

        printf("M1: %d, ", encoderCounter_M1.read());
        printf("M2: %d\n", encoderCounter_M2.read());

        led1 = !led1;
        thread_sleep_for(100);
    }
}