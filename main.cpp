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

#define TIM_MST_BIT_WIDTH  32 // 16 or 32

#else

// #define TIM_MST      TIM2
// #define TIM_MST_IRQ  TIM2_IRQn
// #define TIM_MST_RCC  __HAL_RCC_TIM2_CLK_ENABLE()
// #define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM2()

// #define TIM_MST_RESET_ON   __HAL_RCC_TIM2_FORCE_RESET()
// #define TIM_MST_RESET_OFF  __HAL_RCC_TIM2_RELEASE_RESET()

// #define TIM_MST_BIT_WIDTH  32 // 16 or 32

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
-------------------------------------------------------------------------------------------
*/


/**
 * when using:

    #define TIM_MST      TIM2
    #define TIM_MST_IRQ  TIM2_IRQn
    #define TIM_MST_RCC  __HAL_RCC_TIM2_CLK_ENABLE()
    #define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM2()

    #define TIM_MST_RESET_ON   __HAL_RCC_TIM2_FORCE_RESET()
    #define TIM_MST_RESET_OFF  __HAL_RCC_TIM2_RELEASE_RESET()

    #define TIM_MST_BIT_WIDTH  32 // 16 or 32

* and defining EncoderCounter encoderCounter_M2(PA_0, PA_1) and using it leads to:

    ++ MbedOS Error Info ++
    Error Status: 0x80020126 Code: 294 Module: 2
    Error Message: CMSIS-RTOS error: ISR Queue overflow
    Location: 0x8002305
    Error Value: 0x2
    Current Thread: rtx_idle Id: 0x200008EC Entry: 0x80022D9 StackSize: 0x300 StackMem: 0x20000930 SP: 0x2000FF3C
    For more info, visit: https://mbed.com/s/error?error=0x80020126&tgt=NUCLEO_L432KC
    -- MbedOS Error Info --
*/


/**
 * when using:

    #define TIM_MST      TIM6
    #define TIM_MST_IRQ  TIM6_IRQn
    #define TIM_MST_RCC  __HAL_RCC_TIM6_CLK_ENABLE()
    #define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM6()

    #define TIM_MST_RESET_ON   __HAL_RCC_TIM6_FORCE_RESET()
    #define TIM_MST_RESET_OFF  __HAL_RCC_TIM6_RELEASE_RESET()

    #define TIM_MST_BIT_WIDTH  16 // 16 or 32

* and defining encoder counter, then the mini segway thread is not running...
*/


EncoderCounter encoderCounter_M1(PA_8, PA_9);
// EncoderCounter encoderCounter_M2(PA_0, PA_1);
MiniSegway miniSegway;

int main()
{
    int i{0};

    // DigitalOut led1(LED1);
    printf("main starting...\n");

    while (true) {

        printf("M1: %d, ", encoderCounter_M1.read());
        // printf("M2: %d\n", encoderCounter_M2.read());
        printf(" i: %d\n", i++);

        // led1 = !led1;
        thread_sleep_for(1000);
    }
}