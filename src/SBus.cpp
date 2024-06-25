#include "SBus.h"

SBus::SBus(PinName rx) : _SerialPipe(NC, rx, /* SBus baudrate*/ 100000)
{
    _SerialPipe.format(/* bits */ 8,
                       /* parity */ SerialBase::Even,
                       /* stop bits */ 2);
    _SerialPipe.enable_output(false);

    _Timer.start();
}

uint16_t SBus::getChannel(uint8_t idx) const
{
    if (idx < SBUS_NUM_OF_CHANNELS)
        return (_channels[idx] < SBUS_MIN_VALUE) ? SBUS_MIN_VALUE :
               (_channels[idx] > SBUS_MAX_VALUE) ? SBUS_MAX_VALUE :
                _channels[idx];
    else
        return 0;
}

bool SBus::isLow(uint8_t idx) const
{
    return (getChannel(idx) < SBUS_LOW_HIGHEST_VALUE) ? true : false;
}

bool SBus::isCenter(uint8_t idx) const
{
    const uint16_t channel = getChannel(idx);
    return ((SBUS_LOW_HIGHEST_VALUE < channel) &&
            (channel < SBUS_HIGH_LOWEST_VALUE)) ? true : false;
}

bool SBus::isHigh(uint8_t idx) const
{
    return (SBUS_HIGH_LOWEST_VALUE < getChannel(idx)) ? true : false;
}

float SBus::getChannelMinusToPlusOne(uint8_t idx) const
{
    const static float gain = 2.0f / (SBUS_MAX_VALUE - SBUS_MIN_VALUE);
    return static_cast<float>(getChannel(idx) - SBUS_MIN_VALUE) * gain - 1.0f;
}

float SBus::getChannelZeroToPlusOne(uint8_t idx) const
{
    const static float gain = 1.0f / (SBUS_MAX_VALUE - SBUS_MIN_VALUE);
    return static_cast<float>(getChannel(idx) - SBUS_MIN_VALUE) * gain;
}

void SBus::processReceivedData()
{
    static uint16_t channels[SBUS_NUM_OF_CHANNELS] = {0};
    static uint8_t buffer[128] = {0}; // default size of SerialPipe rx is 128
    static uint8_t pkg[SBUS_NUM_OF_BYTES] = {0};
    static uint8_t idx{0};
    static microseconds time_previous{0};

    // read all available bytes
    int bytes_read = _SerialPipe.get(&buffer, sizeof(buffer), false);

    for (uint8_t i = 0; i < bytes_read; i++) {
        // find start byte
        if (idx == 0 && buffer[i] != SBUS_START_BYTE) {
            // incorrect start byte, out of sync
            _num_of_skipped_start_frames++;
            continue;
        }
        // update sbus pkg
        pkg[idx++] = buffer[i];
        // decode sbus pkg
        if (idx == SBUS_NUM_OF_BYTES) {
            // reset pkg index
            idx = 0;
            // check if end byte is correct
            if (pkg[SBUS_NUM_OF_BYTES - 1] != SBUS_END_BYTE) {
                // incorrect end byte, out of sync
                _num_of_decoder_error_frames++;
                // TODO: understand and fix the following
                // somehow the system is hanging sometimes if continue is used here, 
                // but get sync back if armed and disarmed again... never dug any deeper
                return; // continue; 
            }

            // decode pkg
            channels[0]  = ((pkg[1]    |pkg[2] <<8)             & SBUS_MASK);
            channels[1]  = ((pkg[2]>>3 |pkg[3] <<5)             & SBUS_MASK);
            channels[2]  = ((pkg[3]>>6 |pkg[4] <<2|pkg[5]<<10)  & SBUS_MASK);
            channels[3]  = ((pkg[5]>>1 |pkg[6] <<7)             & SBUS_MASK);
            channels[4]  = ((pkg[6]>>4 |pkg[7] <<4)             & SBUS_MASK);
            channels[5]  = ((pkg[7]>>7 |pkg[8] <<1|pkg[9]<<9)   & SBUS_MASK);
            channels[6]  = ((pkg[9]>>2 |pkg[10]<<6)             & SBUS_MASK);
            channels[7]  = ((pkg[10]>>5|pkg[11]<<3)             & SBUS_MASK);
            // channels[8]  = ((pkg[12]   |pkg[13]<<8)             & SBUS_MASK);
            // channels[9]  = ((pkg[13]>>3|pkg[14]<<5)             & SBUS_MASK);
            // channels[10] = ((pkg[14]>>6|pkg[15]<<2|pkg[16]<<10) & SBUS_MASK);
            // channels[11] = ((pkg[16]>>1|pkg[17]<<7)             & SBUS_MASK);
            // channels[12] = ((pkg[17]>>4|pkg[18]<<4)             & SBUS_MASK);
            // channels[13] = ((pkg[18]>>7|pkg[19]<<1|pkg[20]<<9)  & SBUS_MASK);
            // channels[14] = ((pkg[20]>>2|pkg[21]<<6)             & SBUS_MASK);
            // channels[15] = ((pkg[21]>>5|pkg[22]<<3)             & SBUS_MASK);

            ((pkg[23])      & 0x0001) ? channels[16] = 2047: channels[16] = 0;
            ((pkg[23] >> 1) & 0x0001) ? channels[17] = 2047: channels[17] = 0;

            if ((pkg[23] >> 3) & 0x0001) {
                _is_failsafe = true;
            } else
                _is_failsafe = false;

            if ((pkg[23] >> 2) & 0x0001)
                _num_of_lost_frames++;
                
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