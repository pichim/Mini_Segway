#include "mbed.h"

#include "src/DebounceIn.h"
#include "src/FastPWM/FastPWM.h"

#define MINI_SEGWAY_PWM_M1_POS PC_8 // PWM3/3 : TIM3_CH3
#define MINI_SEGWAY_PWM_M1_NEG PC_9 // PWM3/4 : TIM3_CH4
FastPWM pwm_M1_pos(MINI_SEGWAY_PWM_M1_POS);
FastPWM pwm_M1_neg(MINI_SEGWAY_PWM_M1_NEG);

#define MINI_SEGWAY_PWM_M2_POS PA_8 // PWM1/1 : TIM1_CH1
#define MINI_SEGWAY_PWM_M2_NEG PA_9 // PWM1/2 : TIM1_CH2
FastPWM pwm_M2_pos(MINI_SEGWAY_PWM_M2_POS);
FastPWM pwm_M2_neg(MINI_SEGWAY_PWM_M2_NEG);

float val_M1 = 0.5f;
float val_M2 = 0.5f;

bool execute_main_task = false;
DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn();

int main()
{
    user_button.fall(&toggle_do_execute_main_fcn);

    // invert polarity of pwms
    TIM3->CCER |= TIM_CCER_CC4P; // invert polarity of pwm on PC_9, PWM3/4 : TIM3_CH4
    TIM1->CCER |= TIM_CCER_CC2P; // invert polarity of pwm on PA_9, PWM1/2 : TIM1_CH2

    pwm_M1_pos.write(val_M1);
    pwm_M1_neg.write(val_M1);

    pwm_M2_pos.write(val_M2);
    pwm_M2_neg.write(val_M2);


    printf("Test Project running...\n");
    DigitalOut led1(LED1);
    while (true) {
        printf("pwm val_M1: %5.2f, pwm val_M2: %5.2f\n", val_M1, val_M2);
        led1 = !led1;
        thread_sleep_for(1000);
    }
}

void toggle_do_execute_main_fcn()
{
    execute_main_task = !execute_main_task;

    if (execute_main_task) {
        val_M1 = 0.8f;
        val_M2 = 0.9f;
    } else {
        val_M1 = 0.3f;
        val_M2 = 0.4f;
    }

    pwm_M1_pos.write(val_M1);
    pwm_M1_neg.write(val_M1);

    pwm_M2_pos.write(val_M2);
    pwm_M2_neg.write(val_M2);
}