#include "PwmIn.h"

PwmIn::PwmIn(PinName pin) : _InteruptIn(pin)
{
    _InteruptIn.enable_irq();
    _InteruptIn.rise(callback(this, &PwmIn::rise));
    _InteruptIn.fall(callback(this, &PwmIn::fall));
    _Timer.start();
}

float PwmIn::getDutyCycle() const
{
    const float period_us = static_cast<float>(_period);
    if (period_us != 0.0f)
        return static_cast<float>(_pulsewidth) / period_us;
    else
        return 0.0f;
}

void PwmIn::rise()
{
    const microseconds time = _Timer.elapsed_time();
    _period = duration_cast<microseconds>(time - _time_previous).count();
    _time_previous = time;
}

void PwmIn::fall()
{
    _pulsewidth = duration_cast<microseconds>(_Timer.elapsed_time() - _time_previous).count();
}

void PwmIn::invertPolarity()
{
    _InteruptIn.disable_irq();
    _pulsewidth = 0;
    _period = 0;
    _InteruptIn.rise(callback(this, &PwmIn::fall));
    _InteruptIn.fall(callback(this, &PwmIn::rise));
    _InteruptIn.enable_irq();
}