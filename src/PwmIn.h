#ifndef PWM_IN_H_
#define PWM_IN_H_

#include "mbed.h"

using namespace std::chrono;

class PwmIn
{
public:
    explicit PwmIn(PinName pin);
    virtual ~PwmIn() {};

    uint32_t getPeriod() const { _period; }
    uint32_t getPulseWidth() const { _pulsewidth; }
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

#endif /* PWM_IN_H_ */