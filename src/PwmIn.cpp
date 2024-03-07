#include "PwmIn.h"

PwmIn::PwmIn(PinName pin) : _InteruptIn(pin)
{
    _InteruptIn.enable_irq();
    _InteruptIn.rise(callback(this, &PwmIn::rise));
    _InteruptIn.fall(callback(this, &PwmIn::fall));
    _Timer.start();
}

PwmIn::~PwmIn()
{
}

uint32_t PwmIn::period()
{
    return _period_us;
}

uint32_t PwmIn::pulseWidth()
{
    return _pulsewidth_us;
}

float PwmIn::dutyCycle()
{
    const float period_mus = static_cast<float>(_period_us);
    if (period_mus != 0.0f)
        return static_cast<float>(_pulsewidth_us) / period_mus;
    else
        return 0.0f;
}

void PwmIn::rise()
{
    const microseconds time_us = _Timer.elapsed_time();
    _period_us = duration_cast<microseconds>(time_us - _time_previous_us).count();
    _time_previous_us = time_us;
}

void PwmIn::fall()
{
    _pulsewidth_us = duration_cast<microseconds>(_Timer.elapsed_time() - _time_previous_us).count();
}

void PwmIn::invertPolarity()
{
    _InteruptIn.disable_irq();
    _pulsewidth_us = 0;
    _period_us = 0;
    _InteruptIn.rise(callback(this, &PwmIn::fall));
    _InteruptIn.fall(callback(this, &PwmIn::rise));
    _InteruptIn.enable_irq();
}