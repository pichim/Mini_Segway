#include "mbed.h"

/**
 * TODO:
 * - adjust LSM6DS3 internal filter settings
 * - move serialStream and rc to MiniSegway
 * - check for all threads the destructor
 *      _Timeout.detach();
 *      _Ticker.detach();
 *      _Thread.terminate();
 * - check for all const functions possible move to header
 * - move all unused destructors to header
 * - check all defines to have the header name first
*/

/** 
    In order to use two encoders you need to change the following file in mbed-os and comment out TIM2 and uncomment TIM16:
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

#define TIM_MST      TIM2
#define TIM_MST_IRQ  TIM2_IRQn
#define TIM_MST_RCC  __HAL_RCC_TIM2_CLK_ENABLE()
#define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM2()

#define TIM_MST_RESET_ON   __HAL_RCC_TIM2_FORCE_RESET()
#define TIM_MST_RESET_OFF  __HAL_RCC_TIM2_RELEASE_RESET()

// #define TIM_MST      TIM16
// #define TIM_MST_IRQ  TIM1_UP_TIM16_IRQn
// #define TIM_MST_RCC  __HAL_RCC_TIM16_CLK_ENABLE()
// #define TIM_MST_DBGMCU_FREEZE  __HAL_DBGMCU_FREEZE_TIM16()

// #define TIM_MST_RESET_ON   __HAL_RCC_TIM16_FORCE_RESET()
// #define TIM_MST_RESET_OFF  __HAL_RCC_TIM16_RELEASE_RESET()

#endif

#define TIM_MST_BIT_WIDTH  32 // 16 or 32

#define TIM_MST_PCLK  1 // Select the peripheral clock number (1 or 2)

// #define TIM_MST_BIT_WIDTH  16 // 16 or 32

// #define TIM_MST_PCLK  2 // Select the peripheral clock number (1 or 2)


#ifdef __cplusplus
}
#endif

#endif // __US_TICKER_DATA_H
-------------------------------------------------------------------------------------------
*/

#include "IMU.h"
#include "MiniSegway.h"
#include "PpmIn.h"
#include "SBus.h"

#if DO_USE_PPM_IN
PpmIn rc(MINI_SEGWAY_RC_DI);
#else
SBus rc(MINI_SEGWAY_RC_RX);
#endif
IMU imu(MINI_SEGWAY_IMU_SDA, MINI_SEGWAY_IMU_SCL);
MiniSegway miniSegway(rc, imu);

// main thread is just blinking the led on the nucleo
int main()
{
    DigitalOut led1(LED1);
    while (true) {
        led1 = !led1;
        thread_sleep_for(1000);
    }
}