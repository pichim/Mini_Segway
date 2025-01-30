#pragma once

#include "mbed.h"

using namespace std::chrono;

class PwmIn
{
public:
    explicit PwmIn(PinName pin);
    virtual ~PwmIn() {};

    uint32_t getPeriod() const { return _period; }
    uint32_t getPulseWidth() const { return _pulsewidth; }
    float getDutyCycle() const;
    void invertPolarity();

private:
    InterruptIn _InteruptIn;
    Timer _Timer;

    uint32_t _period{0};
    uint32_t _pulsewidth{0};
    microseconds _time_previous{0};

    void rise();
    void fall();
};