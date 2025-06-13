#pragma once

#include "config.h"

#include "ThreadFlag.h"

using namespace std::chrono;

class Servo
{
public:
    explicit Servo(PinName pin);
    ~Servo();

    void write(float val);

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    DigitalOut _DigitalOut;
    Timeout _Timeout;

    float _val;

    const float _angle_gain = (MINI_SEGWAY_SERVO_VALUE_MAX - MINI_SEGWAY_SERVO_VALUE_MIN) / (MINI_SEGWAY_SERVO_VALUE_RAD_MAX - MINI_SEGWAY_SERVO_VALUE_RAD_MIN);
    const float _normalised_offset = MINI_SEGWAY_SERVO_VALUE_MIN;

    void writeAngleAsSoftPWM(float val);

    void enableDigitalOutput();
    void disableDigitalOutput();

    void threadTask();
    void sendThreadFlag();
};
