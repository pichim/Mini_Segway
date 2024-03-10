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

uint16_t PpmIn::getChannel(uint8_t idx) const
{
    if (idx < PPM_IN_NUM_OF_CHANNELS)
        return (_channels[idx] < PPM_IN_MIN_VALUE) ? PPM_IN_MIN_VALUE :
               (_channels[idx] > PPM_IN_MAX_VALUE) ? PPM_IN_MAX_VALUE :
                _channels[idx];
    else
        return 0;
}

bool PpmIn::isLow(uint8_t idx) const
{
    return (getChannel(idx) < PPM_IN_LOW_HIGHEST_VALUE) ? true : false;
}

bool PpmIn::isCenter(uint8_t idx) const
{
    const uint16_t channel = getChannel(idx);
    return ((PPM_IN_LOW_HIGHEST_VALUE < channel) &&
            (channel < PPM_IN_HIGH_LOWEST_VALUE)) ? true : false;
}

bool PpmIn::isHigh(uint8_t idx) const
{
    return (PPM_IN_HIGH_LOWEST_VALUE < getChannel(idx)) ? true : false;
}

float PpmIn::getChannelMinusToPlusOne(uint8_t idx) const
{
    const static float gain = 2.0f / (PPM_IN_MAX_VALUE - PPM_IN_MIN_VALUE);
    return static_cast<float>(getChannel(idx) - PPM_IN_MIN_VALUE) * gain - 1.0f;
}

float PpmIn::getChannelZeroToPlusOne(uint8_t idx) const
{
    const static float gain = 1.0f / (PPM_IN_MAX_VALUE - PPM_IN_MIN_VALUE);
    return static_cast<float>(getChannel(idx) - PPM_IN_MIN_VALUE) * gain;
}

void PpmIn::rise(void)
{
    _time_previous = _Timer.elapsed_time();
    _Timeout.attach(callback(this, &PpmIn::fall), microseconds{PPM_IN_MAX_TIMEOUT});
}

void PpmIn::fall(void)
{
    static uint16_t channels[PPM_IN_NUM_OF_CHANNELS] = {0};
    static uint8_t idx = 0;
    static microseconds time_previous{0};

    // measure delta time
    const uint16_t channel = duration_cast<microseconds>(_Timer.elapsed_time() - _time_previous).count();

    if (channel < PPM_IN_MAX_TIMEOUT) {
        // detect the start of a new data frame
        if ((channel > PPM_IN_TIME_BETWEEN_DATA) || (idx == PPM_IN_NUM_OF_CHANNELS))
            idx = 0;
        else {
            // update channel
            channels[idx++] = channel;
            // check if all channels were written at least once
            if (idx == PPM_IN_NUM_OF_CHANNELS) {
                // check if all channels are within a healthy range
                for (uint8_t i = 0; i < PPM_IN_NUM_OF_CHANNELS; i++) {
                    if ((channels[i] < PPM_IN_HEALTHY_MIN_VALUE) || (channels[i] > PPM_IN_HEALTHY_MAX_VALUE)) {
                        // exit the for loop
                        break;
                    }
                    // update channels and set pkg as valid
                    memcpy(&_channels, &channels, sizeof(_channels));
                    _is_pkg_valid = true;
                    // meassure signal period
                    const microseconds time = _Timer.elapsed_time();
                    _period = duration_cast<microseconds>(time - time_previous).count();
                    time_previous = time;
                }                    
            }
        }
    }
    _Timeout.attach(callback(this, &PpmIn::rise), microseconds{PPM_IN_MAX_TIMEOUT});
}