#ifndef MOTOR_H_
#define MOTOR_H_

#include "FastPWM/FastPWM.h"

#define MOTOR_DUTY_CYCLE_MIN_VALUE 0.01f
#define MOTOR_DUTY_CYCLE_MAX_VALUE 0.99f

class Motor
{
public:
    explicit Motor(PinName c,
                   float voltage_max);
    virtual ~Motor() {};

    void reset();
    void setVoltage(float voltage);

private:
    FastPWM _pwm;
    float _voltage_max;
};

#endif /* MOTOR_H_ */