#ifndef MOTOR_H_
#define MOTOR_H_

#include "config.h"

#include "FastPWM/FastPWM.h"

#define MOTOR_DUTY_CYCLE_MIN_VALUE MINI_SEGWAY_PWM_MIN_VALUE // used to be 0.01f
#define MOTOR_DUTY_CYCLE_MAX_VALUE MINI_SEGWAY_PWM_MAX_VALUE // used to be 0.99f

class Motor
{
public:
    explicit Motor(PinName pin_pwm_pos,
                   PinName pin_pwm_neg,
                   float voltage_max = 12.0f);
    virtual ~Motor() {};

    void reset(float voltage = 0.0f);
    void setVoltage(float voltage);

private:
    FastPWM _pwm_pos, _pwm_neg;
    float _voltage_max;
};

#endif /* MOTOR_H_ */