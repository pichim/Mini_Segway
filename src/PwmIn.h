#ifndef PWM_IN_H_
#define PWM_IN_H_

#include "mbed.h"

using namespace std::chrono;

class PwmIn
{
public:
    explicit PwmIn(PinName pin);
    virtual ~PwmIn();

    uint32_t period();
    uint32_t pulseWidth();
    float dutyCycle();
    void invertPolarity();

private:
    InterruptIn _InteruptIn;
    Timer _Timer;

    uint32_t _pulsewidth_us{0};
    uint32_t _period_us{0};
    microseconds _time_previous_us{0};

    void rise();
    void fall();
};

#endif /* PWM_IN_H_ */