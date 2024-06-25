#include "Motor.h"

Motor::Motor(PinName pwm,
             PinName dout,
             float voltage_max) : _pwm(pwm)
                                , _dir(dout)
                                , _voltage_max(voltage_max)
{
    _pwm.period_mus(MINI_SEGWAY_PWM_PERIOD_US);
    setVoltage(0.0f);
}

float Motor::setVoltage(float voltage)
{
    const float sign = copysignf(1.0f, voltage);
    voltage = fabsf(voltage);
    float duty_cycle = voltage / _voltage_max;
    duty_cycle = (duty_cycle < MOTOR_DUTY_CYCLE_MIN_VALUE) ? MOTOR_DUTY_CYCLE_MIN_VALUE :
                 (duty_cycle > MOTOR_DUTY_CYCLE_MAX_VALUE) ? MOTOR_DUTY_CYCLE_MAX_VALUE :
                  duty_cycle;
    _pwm.write(duty_cycle);
    _dir = (sign > 0.0f) ? 1 : 0;
    return sign * duty_cycle * _voltage_max;
}