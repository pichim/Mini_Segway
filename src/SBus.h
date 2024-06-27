#pragma once

/**
 * This class is designed to read SBUS signals from a elrs receiver
 * - on the tested receiver only 8 channels work
 * - receiver is sending data at 111 Hz
 * 
 * Channel Mapping when remote is a Tango2 and was set up for Betaflight, measurement from 29.02.2024:
 * CH1: Roll     - Right Stick          , left to right
 * CH2: Pitch    - Right Stick          , down to up
 * CH3: Throttle - Left Stick           , down to up
 * CH4: Yaw      - Left Stick           , left to right
 * CH5: untested
 * CH6: untested
 * CH7: untested
 * CH8: Arm      - Top right big Switch , disabled, enabled
*/

#include "mbed.h"

#include "ThreadFlag.h"
#include "SerialPipe/serial_pipe.h"

#define SBUS_NUM_OF_CHANNELS 8 // 18 reduced to 8, if you want all 18 channels, you need to change the
                               // number to 18 and uncomment the commented code in the decode function
#define SBUS_NUM_OF_BYTES 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
#define SBUS_END_BYTE2 0x04
#define SBUS_MASK 0x07FF
#define SBUS_MIN_VALUE 192
#define SBUS_MAX_VALUE 1792
#define SBUS_LOW_HIGHEST_VALUE 602
#define SBUS_HIGH_LOWEST_VALUE 1402

using namespace std::chrono;

class SBus
{
public:
    explicit SBus(PinName rx);
    virtual ~SBus() {};

    uint16_t getChannel(uint8_t idx) const;
    uint32_t getPeriod() const { return _period; }
    uint8_t getNumOfChannels() const { return SBUS_NUM_OF_CHANNELS; }
    bool isLow(uint8_t idx) const;
    bool isCenter(uint8_t idx) const;
    bool isHigh(uint8_t idx) const;
    float getChannelMinusToPlusOne(uint8_t idx) const;
    float getChannelZeroToPlusOne(uint8_t idx) const;
    bool isPkgValid() const { return _is_pkg_valid; }
    void setPkgValidFalse() { _is_pkg_valid = false; }
    void processReceivedData();

    uint32_t getNumOfDecoderErrorFrames() const { return _num_of_decoder_error_frames; }
    bool isFailSafe() const { return _is_failsafe; }
    uint32_t getNumOfLostFrames() const { return _num_of_lost_frames; }
    uint32_t getNumOfSkippedStartFrames() const { return _num_of_skipped_start_frames; }

private:
    SerialPipe _SerialPipe;
    Timer _Timer;

    uint16_t _channels[SBUS_NUM_OF_CHANNELS] = {0};
    uint32_t _num_of_skipped_start_frames{0};
    uint32_t _num_of_decoder_error_frames{0};
    bool _is_failsafe{false};
    uint32_t _num_of_lost_frames{0};
    uint32_t _period{0};
    bool _is_pkg_valid{false};
};