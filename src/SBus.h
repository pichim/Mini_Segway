#ifndef SBUS_H_
#define SBUS_H_

#include "mbed.h"

#include "ThreadFlag.h"
#include "SerialPipe/serial_pipe.h"

#define  SBUS_DO_RUN_AS_THREAD false
#if SBUS_DO_RUN_AS_THREAD
    // radiomaster elrs rx, running at 111 Hz := ~9000 mus
    #define SBUS_PERIOD_US 500
#endif

#define SBUS_NUM_OF_CHANNELS 8 // 18 reduced to 8, since I don't need more channels
#define SBUS_NUM_OF_BYTES 25
#define SBUS_START_BYTE 0x0f
#define SBUS_END_BYTE 0x00
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
#if !SBUS_DO_RUN_AS_THREAD
    void processReceivedData();
#endif

private:
#if SBUS_DO_RUN_AS_THREAD
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;
#endif

    SerialPipe _SerialPipe;
    Timer _Timer;

    uint16_t _channels[SBUS_NUM_OF_CHANNELS] = {0};
    uint32_t _num_of_decoder_error_frames{0};
    bool _is_failsafe{false};
    uint32_t _num_of_lost_frames{0};
    uint32_t _period{0};
    bool _is_pkg_valid{false};

#if SBUS_DO_RUN_AS_THREAD
    void processReceivedData();
    void sendThreadFlag();
#endif
};

#endif /* SBUS_H_ */
