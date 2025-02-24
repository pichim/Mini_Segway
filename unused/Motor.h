#pragma once

#include "config.h"

#include "FastPWM.h"
#include "IIRFilter.h"
#include "ThreadFlag.h"

#define MOTOR_DUTY_CYCLE_MIN_VALUE MINI_SEGWAY_PWM_MIN_VALUE
#define MOTOR_DUTY_CYCLE_MAX_VALUE MINI_SEGWAY_PWM_MAX_VALUE

using namespace std::chrono;

class Motor
{
public:
    explicit Motor(PinName pwm,
                   PinName dout,
                   float voltage_max = 12.0f);
    virtual ~Motor();

    float setVoltage(float voltage = 0.0f);

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    FastPWM _pwm;
    DigitalOut _dir;
    IIRFilter _upsamplingLowPass1;
    float _voltage;
    float _voltage_max;

    void threadTask();
    void sendThreadFlag();
};