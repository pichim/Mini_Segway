#include "Motor.h"

Motor::Motor(PinName pin_pwm_pos,
             PinName pin_pwm_neg,
             float voltage_max) : _pwm_pos(pin_pwm_pos)
                                , _pwm_neg(pin_pwm_neg)
                                , _voltage_max(voltage_max)
{
    _pwm_pos.period_mus(MINI_SEGWAY_PWM_PERIOD_US);
    _pwm_neg.period_mus(MINI_SEGWAY_PWM_PERIOD_US);
    reset();
}

void Motor::reset(float voltage)
{
    setVoltage(voltage);
}

void Motor::setVoltage(float voltage)
{
    float duty_cycle = 0.5f + 0.5f * voltage / _voltage_max;
    duty_cycle = (duty_cycle < MOTOR_DUTY_CYCLE_MIN_VALUE) ? MOTOR_DUTY_CYCLE_MIN_VALUE :
                 (duty_cycle > MOTOR_DUTY_CYCLE_MAX_VALUE) ? MOTOR_DUTY_CYCLE_MAX_VALUE :
                  duty_cycle;
    _pwm_pos.write(duty_cycle);
    _pwm_neg.write(duty_cycle);
}