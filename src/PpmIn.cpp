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
    if (idx < NUM_OF_CHANNELS)
        return _channel_us[idx];
}

uint16_t PpmIn::period() const
{
    return _period_us;
}

bool PpmIn::isLow(uint8_t idx) const
{
    const uint16_t channel_us = getChannelMus(idx);
    return (channel_us < CHANNEL_LOW_HIGHEST_VALUE_US) ? true : false;
}

bool PpmIn::isCenter(uint8_t idx) const
{
    const uint16_t channel_us = getChannelMus(idx);
    return ((CHANNEL_LOW_HIGHEST_VALUE_US < channel_us) &&
            (channel_us < CHANNEL_HIGH_LOWEST_VALUE_US)) ? true : false;
}

bool PpmIn::isHigh(uint8_t idx) const
{
    const uint16_t channel_us = getChannelMus(idx);
    return (CHANNEL_HIGH_LOWEST_VALUE_US < channel_us) ? true : false;
}

float PpmIn::getChannelMinusToPlusOne(uint8_t idx) const
{
    const static float gain = 2.0f / (CHANNEL_MAX_VALUE_US - CHANNEL_MIN_VALUE_US);
    const uint16_t channel_us = getChannelMus(idx);
    return static_cast<float>(channel_us - CHANNEL_MIN_VALUE_US) * gain - 1.0f;
}

float PpmIn::getChannelZeroToPlusOne(uint8_t idx) const
{
    const static float gain = 1.0f / (CHANNEL_MAX_VALUE_US - CHANNEL_MIN_VALUE_US);
    const uint16_t channel_us = getChannelMus(idx);
    return static_cast<float>(channel_us - CHANNEL_MIN_VALUE_US) * gain;
}

void PpmIn::rise(void)
{
    _time_previous_us = _Timer.elapsed_time();
    _Timeout.attach(callback(this, &PpmIn::fall), microseconds{MAX_TIMEOUT_US});
}

void PpmIn::fall(void)
{
    static uint8_t channel_cntr = 0;
    static microseconds time_previous_us{0};

    const uint16_t channel_us = duration_cast<microseconds>(_Timer.elapsed_time() - _time_previous_us).count();

    if (channel_us > MAX_TIMEOUT_US) {
        _is_pkg_valid = false;
    } else {
        // detect the start of a new data frame
        if ((channel_us > TIME_BETWEEN_DATA_US) || (channel_cntr == NUM_OF_CHANNELS)) {
            channel_cntr = 0;
            // meassure signal period
            const microseconds time_us = _Timer.elapsed_time();
            _period_us = duration_cast<microseconds>(time_us - time_previous_us).count();
            time_previous_us = time_us;
        } else {
            // update channel
            _buffer_us[channel_cntr++] = channel_us;

            // check if all channels were written at least once
            if (channel_cntr == NUM_OF_CHANNELS) {
                // check if all channels are within a healthy range
                _is_pkg_valid = true;
                for (uint8_t i = 0; i < NUM_OF_CHANNELS; i++) {
                    if ((_buffer_us[i] < CHANNEL_HEALTHY_MIN_VALUE_US) || (_buffer_us[i] > CHANNEL_HEALTHY_MAX_VALUE_US)) {
                        _is_pkg_valid = false;
                        // exit the for loop
                        break;
                    } else {
                        _channel_us[i] = (_buffer_us[i] < CHANNEL_MIN_VALUE_US) ? CHANNEL_MIN_VALUE_US :
                                         (_buffer_us[i] > CHANNEL_MAX_VALUE_US) ? CHANNEL_MAX_VALUE_US :
                                          _buffer_us[i];
                    }
                }
            }
        }
    }
    _Timeout.attach(callback(this, &PpmIn::rise), microseconds{MAX_TIMEOUT_US});
}