/*
 * EncoderCounter.cpp
 * Copyright (c) 2018, ZHAW
 * All rights reserved.
 *
 *  Created on: 26.01.2018
 *      Author: Marcel Honegger
 */

#include "EncoderCounter.h"

using namespace std;

/**
 * Creates and initializes the driver to read the quadrature
 * encoder counter of the STM32 microcontroller.
 * @param a the input pin for the channel A.
 * @param b the input pin for the channel B.
 */
EncoderCounter::EncoderCounter(PinName a, PinName b) {
    
    // check pins
    
    if ((a == PA_8) && (b == PA_9)) {
        
        // pinmap OK for TIM1 CH1 and CH2
        TIM = TIM1;
        
        // configure general purpose I/O registers
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // manually enable port A
        
        GPIOA->MODER &= ~GPIO_MODER_MODER8;     // reset port A8
        GPIOA->MODER |= GPIO_MODER_MODER8_1;    // set alternate mode of port A8
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR8;     // reset pull-up/pull-down on port A8
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_1;    // set input as pull-down
        GPIOA->AFR[1] &= ~(0xF << 4*0);         // reset alternate function of port A8
        GPIOA->AFR[1] |= 1 << 4*0;              // set alternate funtion 1 of port A8
        
        GPIOA->MODER &= ~GPIO_MODER_MODER9;     // reset port A9
        GPIOA->MODER |= GPIO_MODER_MODER9_1;    // set alternate mode of port A9
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR9;     // reset pull-up/pull-down on port A9
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_1;    // set input as pull-down
        GPIOA->AFR[1] &= ~(0xF << 4*1);         // reset alternate function of port A9
        GPIOA->AFR[1] |= 1 << 4*1;              // set alternate funtion 1 of port A9
        
        // configure reset and clock control registers
        
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;  //reset TIM1 controller
        RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
        
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;     // TIM1 clock enable
        
    } else if ((a == PA_0) && (b == PA_1)) {
        
        // pinmap OK for TIM2 CH1 and CH2
        
        TIM = TIM2;
        
        // configure general purpose I/O registers
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // manually enable port A
        
        GPIOA->MODER &= ~GPIO_MODER_MODER0;     // reset port A0
        GPIOA->MODER |= GPIO_MODER_MODER0_1;    // set alternate mode of port A0
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;     // reset pull-up/pull-down on port A0
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;    // set input as pull-down
        GPIOA->AFR[0] &= ~(0xF << 4*0);         // reset alternate function of port A0
        GPIOA->AFR[0] |= 1 << 4*0;              // set alternate funtion 1 of port A0
        
        GPIOA->MODER &= ~GPIO_MODER_MODER1;     // reset port A1
        GPIOA->MODER |= GPIO_MODER_MODER1_1;    // set alternate mode of port A1
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;     // reset pull-up/pull-down on port A1
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1;    // set input as pull-down
        GPIOA->AFR[0] &= ~(0xF << 4*1);         // reset alternate function of port A1
        GPIOA->AFR[0] |= 1 << 4*1;              // set alternate funtion 1 of port A1
        
        // configure reset and clock control registers
        
        RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST;  //reset TIM2 controller
        RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;
        
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;     // TIM2 clock enable

    
    } else if ((a == PA_5) && (b == PA_1)) {
        
        // pinmap OK for TIM2 CH1 and CH2
        
        TIM = TIM2;
        
        // configure general purpose I/O registers
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // manually enable port A
        
        GPIOA->MODER &= ~GPIO_MODER_MODER5;     // reset port A0
        GPIOA->MODER |= GPIO_MODER_MODER5_1;    // set alternate mode of port A0
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;     // reset pull-up/pull-down on port A0
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_1;    // set input as pull-down
        GPIOA->AFR[0] &= ~(0xF << 4*5);         // reset alternate function of port A0
        GPIOA->AFR[0] |= 1 << 4*5;              // set alternate funtion 1 of port A0
        
        GPIOA->MODER &= ~GPIO_MODER_MODER1;     // reset port A1
        GPIOA->MODER |= GPIO_MODER_MODER1_1;    // set alternate mode of port A1
        GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;     // reset pull-up/pull-down on port A1
        GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_1;    // set input as pull-down
        GPIOA->AFR[0] &= ~(0xF << 4*1);         // reset alternate function of port A1
        GPIOA->AFR[0] |= 1 << 4*1;              // set alternate funtion 1 of port A1
        
        // configure reset and clock control registers
        
        RCC->APB1RSTR1 |= RCC_APB1RSTR1_TIM2RST;  //reset TIM2 controller
        RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM2RST;
        
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;     // TIM2 clock enable

    
    } else {
        
        printf("pinmap not found for peripheral\n");
    }
    
    // configure general purpose timer 1
    
    TIM->CR1 = 0x0000;          // counter disable
    TIM->CR2 = 0x0000;          // reset master mode selection
    TIM->SMCR = TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0; // counting on both TI1 & TI2 edges
    TIM->CCMR1 = TIM_CCMR1_CC2S_0 | TIM_CCMR1_CC1S_0;
    TIM->CCMR2 = 0x0000;        // reset capture mode register 2
    TIM->CCER = TIM_CCER_CC2E | TIM_CCER_CC1E;
    TIM->CNT = 0x0000;          // reset counter value
    TIM->ARR = 0xFFFF;          // auto reload register
    TIM->CR1 = TIM_CR1_CEN;     // counter enable
}

EncoderCounter::~EncoderCounter() {}

/**
 * Resets the counter value to zero.
 */
void EncoderCounter::reset() {
    
    TIM->CNT = 0x0000;
}

/**
 * Resets the counter value to a given offset value.
 * @param offset the offset value to reset the counter to.
 */
void EncoderCounter::reset(int16_t offset) {
    
    TIM->CNT = -offset;
}

/**
 * Reads the quadrature encoder counter value.
 * @return the quadrature encoder counter as a signed 16-bit integer value.
 */
int16_t EncoderCounter::read() {
    
    return static_cast<int16_t>(-TIM->CNT);
}

/**
 * The empty operator is a shorthand notation of the <code>read()</code> method.
 */
EncoderCounter::operator int16_t() {
    
    return read();
}
