#include "mbed.h"

#include "EncoderCounter.h"
#include "MiniSegway.h"

/** 
    In order to use two encoders you need to change the following file in mbed-os:
    - mbed-os/targets/TARGET_STM/TARGET_STM32L4/us_ticker_data.h"
-------------------------------------------------------------------------------------------
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

#else

#define TIM_MST      TIM16
#define TIM_MST_IRQ  TIM1_UP_TIM16_IRQn
#define TIM_MST_RCC  __HAL_RCC_TIM16_CLK_ENABLE()
#define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM16()

#define TIM_MST_RESET_ON   __HAL_RCC_TIM16_FORCE_RESET()
#define TIM_MST_RESET_OFF  __HAL_RCC_TIM16_RELEASE_RESET()

#endif

#define TIM_MST_BIT_WIDTH  16 // 16 or 32

#define TIM_MST_PCLK  2 // Select the peripheral clock number (1 or 2)


#ifdef __cplusplus
}
#endif

#endif // __US_TICKER_DATA_H
-------------------------------------------------------------------------------------------
*/


// EncoderCounter encoderCounter_M1(PA_8, PA_9);
EncoderCounter encoderCounter_M2(PA_5, PA_1);
MiniSegway miniSegway;

int main()
{
    int i{0};

    // DigitalOut led1(LED1);
    printf("main starting...\n");

    while (true) {

        // printf("M1: %d,\n", encoderCounter_M1.read());
        printf("M2: %d\n", encoderCounter_M2.read());
        // printf(" i: %d\n", i++);

        // led1 = !led1;
        thread_sleep_for(100);
    }
}