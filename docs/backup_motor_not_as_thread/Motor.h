#pragma once

#include "config.h"

#include "FastPWM/FastPWM.h"

#define MOTOR_DUTY_CYCLE_MIN_VALUE MINI_SEGWAY_PWM_MIN_VALUE
#define MOTOR_DUTY_CYCLE_MAX_VALUE MINI_SEGWAY_PWM_MAX_VALUE

class Motor
{
public:
    explicit Motor(PinName pwm,
                   PinName dout,
                   float voltage_max = 12.0f);
    virtual ~Motor() = default;

    float setVoltage(float voltage);

private:
    FastPWM _pwm;
    DigitalOut _dir;
    float _voltage_max;
};