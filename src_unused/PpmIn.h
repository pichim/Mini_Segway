#pragma once

/**
 * This class is designed to read PPM signals from a crsf xf nano rx
 * - on the tested receiver only 8 channels work
 * - measured min. time between to data frames are approx. 2.7 ms
 * - receiver is sending data at 50 hz
 * 
 * Channel Mapping when remote is a Tango2 and was set up for Betaflight, measurement from 29.02.2024:
 * CH1: Roll     - Right Stick                  , left to right               -> (591, 1615) mus
 * CH2: Pitch    - Right Stick                  , down to up                  -> (591, 1611) mus
 * CH3: Throttle - Left Stick                   , down to up                  -> (587, 1606) mus
 * CH4: Yaw      - Left Stick                   , left to right               -> (587, 1611) mus
 * CH5: Arm      - Top right big Switch         , disabled, enabled           -> (587, 1611) mus
 * CH6:          - Behind right momentary switch, disabled, enabled           -> (587, 1612) mus
 * CH7:          - Top right 3-way switch       , (forward, center, backward) -> (587, 1100, 1611) mus
 * CH8:          - Top left  3-was switch       , (forward, center, backward) -> (587, 1100, 1611) mus
*/

#include "mbed.h"

// tbs crossfire nano rx, running at 50 Hz := 20000 mus
#define PPM_IN_NUM_OF_CHANNELS 8
#define PPM_IN_TIME_BETWEEN_DATA 2500
#define PPM_IN_MAX_TIMEOUT 20000
#define PPM_IN_MIN_VALUE 600
#define PPM_IN_MAX_VALUE 1600
#define PPM_IN_HEALTHY_MIN_VALUE 400
#define PPM_IN_HEALTHY_MAX_VALUE 1800
#define PPM_IN_LOW_HIGHEST_VALUE 850
#define PPM_IN_HIGH_LOWEST_VALUE 1350

using namespace std::chrono;

class PpmIn
{
public:
    explicit PpmIn(PinName pin);
    virtual ~PpmIn();

    uint16_t getChannel(uint8_t idx) const;
    uint32_t getPeriod() const { return _period; }
    uint8_t getNumOfChannels() const { return PPM_IN_NUM_OF_CHANNELS; }
    bool isLow(uint8_t idx) const;
    bool isCenter(uint8_t idx) const;
    bool isHigh(uint8_t idx) const;
    float getChannelMinusToPlusOne(uint8_t idx) const;
    float getChannelZeroToPlusOne(uint8_t idx) const;
    bool isPkgValid() const { return _is_pkg_valid; }
    void setPkgValidFalse() { _is_pkg_valid = false; }

private:
    InterruptIn _InteruptIn;
    Timer _Timer;
    Timeout _Timeout;

    microseconds _time_previous{0};
    uint16_t _channels[PPM_IN_NUM_OF_CHANNELS] = {0};
    uint32_t _period{0};
    bool _is_pkg_valid{false};

    void rise();
    void fall();
};