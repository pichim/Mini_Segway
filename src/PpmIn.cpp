#include "PpmIn.h"

PpmIn::PpmIn(PinName pin) : _InteruptIn(pin)
{
    _InteruptIn.enable_irq();
    _InteruptIn.rise(callback(this, &PpmIn::rise));
    _InteruptIn.fall(callback(this, &PpmIn::fall));
    _Timer.start();
}

PpmIn::~PpmIn(void)
{
    _Timeout.detach();
}

uint16_t PpmIn::getChannelMus(uint8_t idx) const
{
    if (_is_data_valid && (idx < NUM_OF_CHANNELS))
        return _channel_mus[idx];
    else
        return 0;
}

uint16_t PpmIn::period() const
{
    if (_is_data_valid)
        return _period_mus;
    else
        return 0;
}

bool PpmIn::isLow(uint8_t idx) const
{
    const uint16_t channel_us = getChannelMus(idx);
    if (channel_us == 0)
        return false;
    else
        return (channel_us < CHANNEL_LOW_HIGHEST_VALUE_MUS) ? true : false;
}

bool PpmIn::isCenter(uint8_t idx) const
{
    const uint16_t channel_us = getChannelMus(idx);
    if (channel_us == 0)
        return false;
    else
        return ((CHANNEL_LOW_HIGHEST_VALUE_MUS < channel_us) &&
                (channel_us < CHANNEL_HIGH_LOWEST_VALUE_MUS)) ? true : false;
}

bool PpmIn::isHigh(uint8_t idx) const
{
    const uint16_t channel_us = getChannelMus(idx);
    if (channel_us == 0)
        return false;
    else
        return (CHANNEL_HIGH_LOWEST_VALUE_MUS < channel_us) ? true : false;
}

float PpmIn::getChannelMinusToPlusOne(uint8_t idx) const
{
    const static float gain = 2.0f / (CHANNEL_MAX_VALUE_MUS - CHANNEL_MIN_VALUE_MUS);
    const uint16_t channel_us = getChannelMus(idx);
    if (channel_us == 0)
        return 0.0f;
    else
        return static_cast<float>(channel_us - CHANNEL_MIN_VALUE_MUS) * gain - 1.0f;
}

float PpmIn::getChannelZeroToPlusOne(uint8_t idx) const
{
    const static float gain = 1.0f / (CHANNEL_MAX_VALUE_MUS - CHANNEL_MIN_VALUE_MUS);
    const uint16_t channel_us = getChannelMus(idx);
    if (channel_us == 0)
        return 0.0f;
    else
        return static_cast<float>(channel_us - CHANNEL_MIN_VALUE_MUS) * gain;
}

void PpmIn::rise(void)
{
    _time_previous_us = _Timer.elapsed_time();
    _Timeout.attach(callback(this, &PpmIn::fall), microseconds{MAX_Timeout_MUS});
}

void PpmIn::fall(void)
{
    static uint8_t channel_cntr = 0;
    static microseconds time_previous_us{0};

    const uint16_t channel_us = duration_cast<microseconds>(_Timer.elapsed_time() - _time_previous_us).count();

    if (channel_us > MAX_Timeout_MUS) {
        _is_data_valid = false;
    } else {
        // detect the start of a new data frame
        if ((channel_us > TIME_BETWEEN_DATA_MUS) || (channel_cntr == NUM_OF_CHANNELS)) {
            channel_cntr = 0;
            // meassure signal period
            const microseconds time_mus = _Timer.elapsed_time();
            _period_mus = duration_cast<microseconds>(time_mus - time_previous_us).count();
            time_previous_us = time_mus;
        } else {
            // update channel
            _channel_mus[channel_cntr++] = (channel_us < CHANNEL_MIN_VALUE_MUS) ? CHANNEL_MIN_VALUE_MUS :
                                           (channel_us > CHANNEL_MAX_VALUE_MUS) ? CHANNEL_MAX_VALUE_MUS :
                                            channel_us;
            // check if all channels were written at least once
            if (!_is_data_valid && (channel_cntr == NUM_OF_CHANNELS))
                _is_data_valid = true;
        }
    }
    _Timeout.attach(callback(this, &PpmIn::rise), microseconds{MAX_Timeout_MUS});
}