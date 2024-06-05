#include "Motor.h"

Motor::Motor(PinName pwm,
             PinName dout,
             float voltage_max) : _pwm(pwm)
                                , _dir(dout)
                                , _voltage_max(voltage_max)
{
    _pwm.period_mus(MINI_SEGWAY_PWM_PERIOD_US);
    _dir = 1;;
    reset();
}

void Motor::reset(float voltage)
{
    setVoltage(voltage);
}

void Motor::setVoltage(float voltage)
{
    const float sign = copysignf(1.0f, voltage);
    _dir = (sign > 0.0f) ? 1 : 0;
    voltage = fabsf(voltage);

    float duty_cycle = voltage / _voltage_max;
    duty_cycle = (duty_cycle < MOTOR_DUTY_CYCLE_MIN_VALUE) ? MOTOR_DUTY_CYCLE_MIN_VALUE :
                 (duty_cycle > MOTOR_DUTY_CYCLE_MAX_VALUE) ? MOTOR_DUTY_CYCLE_MAX_VALUE :
                  duty_cycle;
    _pwm.write(duty_cycle);
}