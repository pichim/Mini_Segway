#include "Motor.h"

Motor::Motor(PinName c,
             float voltage_max) : _pwm(c)
                                , _voltage_max(voltage_max)
{
    reset();
}

void Motor::reset()
{
    setVoltage(0.0f);
}

void Motor::setVoltage(float voltage)
{
    const float duty_cycle = 0.5f + 0.5f * voltage / _voltage_max;
    _pwm.write((duty_cycle < MOTOR_DUTY_CYCLE_MIN_VALUE) ? MOTOR_DUTY_CYCLE_MIN_VALUE :
               (duty_cycle > MOTOR_DUTY_CYCLE_MAX_VALUE) ? MOTOR_DUTY_CYCLE_MAX_VALUE :
                duty_cycle);
}