#include "Motor.h"

Motor::Motor(PinName a,
             PinName b,
             float voltage_max) : _pwm_pos(a)
                                , _pwm_neg(b)
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
    float duty_cycle = 0.5f + 0.5f * voltage / _voltage_max;
    duty_cycle = (duty_cycle < MOTOR_DUTY_CYCLE_MIN_VALUE) ? MOTOR_DUTY_CYCLE_MIN_VALUE :
                 (duty_cycle > MOTOR_DUTY_CYCLE_MAX_VALUE) ? MOTOR_DUTY_CYCLE_MAX_VALUE :
                  duty_cycle;
    _pwm_pos.write(duty_cycle);
    _pwm_neg.write(duty_cycle);
}