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

    // // map from (-MINI_SEGWAY_SERVO_VALUE_RAD_MAX, MINI_SEGWAY_SERVO_VALUE_RAD_MAX) -> (0.0f, 1.0f)
    // const float _angle_gain = 1.0f / (2.0f * MINI_SEGWAY_SERVO_VALUE_RAD_MAX);
    // const float _angle_offset = _angle_gain * MINI_SEGWAY_SERVO_VALUE_RAD_MAX;
    // // map from (0.0f, 1.0f) -> (MINI_SEGWAY_SERVO_VALUE_MIN, MINI_SEGWAY_SERVO_VALUE_MAX)
    // const float _normalised_gain = MINI_SEGWAY_SERVO_VALUE_MAX - MINI_SEGWAY_SERVO_VALUE_MIN;
    // const float _normalised_offset = MINI_SEGWAY_SERVO_VALUE_MIN;

    // Simplified mapping
    const float _angle_gain = (MINI_SEGWAY_SERVO_VALUE_MAX - MINI_SEGWAY_SERVO_VALUE_MIN) / (MINI_SEGWAY_SERVO_VALUE_RAD_MAX - MINI_SEGWAY_SERVO_VALUE_RAD_MIN);
    const float _normalised_offset = MINI_SEGWAY_SERVO_VALUE_MIN;

    void writeAngleAsSoftPWM(float val);

    void enableDigitalOutput();
    void disableDigitalOutput();

    void threadTask();
    void sendThreadFlag();
};
