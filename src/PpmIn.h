#ifndef PPM_IN_H_
#define PPM_IN_H_

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

#define NUM_OF_CHANNELS 8
#define TIME_BETWEEN_DATA_MUS 2500
#define MAX_Timeout_MUS 20000
#define CHANNEL_MIN_VALUE_MUS 600
#define CHANNEL_MAX_VALUE_MUS 1600
#define CHANNEL_LOW_HIGHEST_VALUE_MUS 850
#define CHANNEL_HIGH_LOWEST_VALUE_MUS 1350

using namespace std::chrono;

class PpmIn
{
public:
    explicit PpmIn(PinName pin);
    virtual ~PpmIn();

    uint16_t getChannelMus(uint8_t idx) const;
    uint8_t getNumOfChannels() const { return NUM_OF_CHANNELS; }
    bool isDataValid() const { return _is_data_valid; }
    uint16_t period() const;
    bool isLow(uint8_t idx) const;
    bool isCenter(uint8_t idx) const;
    bool isHigh(uint8_t idx) const;
    float getChannelMinusToPlusOne(uint8_t idx) const;
    float getChannelZeroToPlusOne(uint8_t idx) const;

private:
    InterruptIn _InteruptIn;
    Timer _Timer;
    Timeout _Timeout;

    microseconds _time_previous_us{0};
    uint16_t _channel_mus[NUM_OF_CHANNELS] = {0};
    bool _is_data_valid{false};
    uint16_t _period_mus{0};

    void rise();
    void fall();
};

#endif /* PPM_IN_H_ */