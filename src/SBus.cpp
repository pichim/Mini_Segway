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
    static uint8_t byte_previous{SBUS_END_BYTE};
    static uint8_t pkg[SBUS_NUM_OF_BYTES] = {0};
    static uint8_t idx{0};
    static microseconds time_previous{0};

    // read all available bytes
    int bytes_read = _SerialPipe.get(&buffer, sizeof(buffer), false);

    // process all bytes
    for (uint8_t i = 0; i < bytes_read; i++) {

        // read actual byte
        uint8_t byte_actual = buffer[i];

        // find start byte
        if (idx == 0) {
            if ((byte_actual == SBUS_START_BYTE) &&((byte_previous == SBUS_END_BYTE) || ((byte_previous & 0x0F) == SBUS_END_BYTE2))) {
                pkg[idx++] = byte_actual;
            } else {
                 // incorrect start byte, out of sync
                _num_of_skipped_start_frames++;
            }
        // read all bytes following start byte into pkg[1] ... pkg[23]
        } else if (idx < SBUS_NUM_OF_BYTES - 1) {
            pkg[idx++] = byte_actual;
        // pkg complete
        } else if (idx == SBUS_NUM_OF_BYTES - 1) {
            // reset pkg index
            idx = 0;
            // check if end byte is correct and if so decode pkg[1] ... pkg[23]
            if ((byte_actual == SBUS_END_BYTE) || ((byte_actual & 0x0F) == SBUS_END_BYTE2)) {
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
                // ((pkg[23])      & 0x0001) ? channels[16] = 2047: channels[16] = 0;
                // ((pkg[23] >> 1) & 0x0001) ? channels[17] = 2047: channels[17] = 0;
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
            } else {
                // incorrect end byte, out of sync
                _num_of_decoder_error_frames++;
            }
        }

        // store current byte as previous byte for next run
        byte_previous = byte_actual;

        // // find start byte
        // if (idx == 0 && buffer[i] != SBUS_START_BYTE) {
        //     // incorrect start byte, out of sync
        //     _num_of_skipped_start_frames++;
        //     continue;
        // }
        // // update sbus pkg
        // pkg[idx++] = buffer[i];
        // // decode sbus pkg
        // if (idx == SBUS_NUM_OF_BYTES) {
        //     // reset pkg index
        //     idx = 0;
        //     // check if end byte is correct
        //     if (pkg[SBUS_NUM_OF_BYTES - 1] != SBUS_END_BYTE) {
        //         // incorrect end byte, out of sync
        //         _num_of_decoder_error_frames++;
        //         // TODO: understand and fix the following
        //         // somehow the system is hanging sometimes if continue is used here, 
        //         // but get sync back if armed and disarmed again... never dug any deeper
        //         return; // continue; 
        //     }

        //     // decode pkg
        //     channels[0]  = ((pkg[1]    |pkg[2] <<8)             & SBUS_MASK);
        //     channels[1]  = ((pkg[2]>>3 |pkg[3] <<5)             & SBUS_MASK);
        //     channels[2]  = ((pkg[3]>>6 |pkg[4] <<2|pkg[5]<<10)  & SBUS_MASK);
        //     channels[3]  = ((pkg[5]>>1 |pkg[6] <<7)             & SBUS_MASK);
        //     channels[4]  = ((pkg[6]>>4 |pkg[7] <<4)             & SBUS_MASK);
        //     channels[5]  = ((pkg[7]>>7 |pkg[8] <<1|pkg[9]<<9)   & SBUS_MASK);
        //     channels[6]  = ((pkg[9]>>2 |pkg[10]<<6)             & SBUS_MASK);
        //     channels[7]  = ((pkg[10]>>5|pkg[11]<<3)             & SBUS_MASK);
        //     // channels[8]  = ((pkg[12]   |pkg[13]<<8)             & SBUS_MASK);
        //     // channels[9]  = ((pkg[13]>>3|pkg[14]<<5)             & SBUS_MASK);
        //     // channels[10] = ((pkg[14]>>6|pkg[15]<<2|pkg[16]<<10) & SBUS_MASK);
        //     // channels[11] = ((pkg[16]>>1|pkg[17]<<7)             & SBUS_MASK);
        //     // channels[12] = ((pkg[17]>>4|pkg[18]<<4)             & SBUS_MASK);
        //     // channels[13] = ((pkg[18]>>7|pkg[19]<<1|pkg[20]<<9)  & SBUS_MASK);
        //     // channels[14] = ((pkg[20]>>2|pkg[21]<<6)             & SBUS_MASK);
        //     // channels[15] = ((pkg[21]>>5|pkg[22]<<3)             & SBUS_MASK);

        //     ((pkg[23])      & 0x0001) ? channels[16] = 2047: channels[16] = 0;
        //     ((pkg[23] >> 1) & 0x0001) ? channels[17] = 2047: channels[17] = 0;

        //     if ((pkg[23] >> 3) & 0x0001) {
        //         _is_failsafe = true;
        //     } else
        //         _is_failsafe = false;

        //     if ((pkg[23] >> 2) & 0x0001)
        //         _num_of_lost_frames++;
                
        //     // update channels and set pkg as valid
        //     memcpy(&_channels, &channels, sizeof(_channels));
        //     _is_pkg_valid = true;
        //     // meassure signal period
        //     const microseconds time = _Timer.elapsed_time();
        //     _period = duration_cast<microseconds>(time - time_previous).count();
        //     time_previous = time;
        // }

    }
}

// bool SbusRx::Parse()
// {
//     /* Parse messages */
//     while (uart_->available())
//     {
//         cur_byte_ = uart_->read();
//         if (state_ == 0)
//         {
//             if ((cur_byte_ == HEADER_) && ((prev_byte_ == FOOTER_) ||
//                                            ((prev_byte_ & 0x0F) == FOOTER2_)))
//             {
//                 buf_[state_++] = cur_byte_;
//             }
//             else
//             {
//                 state_ = 0;
//             }
//         }
//         else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_)
//         {
//             buf_[state_++] = cur_byte_;
//         }
//         else if (state_ < PAYLOAD_LEN_ + HEADER_LEN_ + FOOTER_LEN_)
//         {
//             state_ = 0;
//             prev_byte_ = cur_byte_;
//             if ((cur_byte_ == FOOTER_) || ((cur_byte_ & 0x0F) == FOOTER2_))
//             {
//                 /* Grab the channel data */
//                 data_.ch[0] = static_cast<int16_t>(buf_[1] |
//                                                    ((buf_[2] << 8) & 0x07FF));
//                 data_.ch[1] = static_cast<int16_t>((buf_[2] >> 3) |
//                                                    ((buf_[3] << 5) & 0x07FF));
//                 data_.ch[2] = static_cast<int16_t>((buf_[3] >> 6) |
//                                                    (buf_[4] << 2) |
//                                                    ((buf_[5] << 10) & 0x07FF));
//                 data_.ch[3] = static_cast<int16_t>((buf_[5] >> 1) |
//                                                    ((buf_[6] << 7) & 0x07FF));
//                 data_.ch[4] = static_cast<int16_t>((buf_[6] >> 4) |
//                                                    ((buf_[7] << 4) & 0x07FF));
//                 data_.ch[5] = static_cast<int16_t>((buf_[7] >> 7) |
//                                                    (buf_[8] << 1) |
//                                                    ((buf_[9] << 9) & 0x07FF));
//                 data_.ch[6] = static_cast<int16_t>((buf_[9] >> 2) |
//                                                    ((buf_[10] << 6) & 0x07FF));
//                 data_.ch[7] = static_cast<int16_t>((buf_[10] >> 5) |
//                                                    ((buf_[11] << 3) & 0x07FF));
//                 data_.ch[8] = static_cast<int16_t>(buf_[12] |
//                                                    ((buf_[13] << 8) & 0x07FF));
//                 data_.ch[9] = static_cast<int16_t>((buf_[13] >> 3) |
//                                                    ((buf_[14] << 5) & 0x07FF));
//                 data_.ch[10] = static_cast<int16_t>((buf_[14] >> 6) |
//                                                     (buf_[15] << 2) |
//                                                     ((buf_[16] << 10) & 0x07FF));
//                 data_.ch[11] = static_cast<int16_t>((buf_[16] >> 1) |
//                                                     ((buf_[17] << 7) & 0x07FF));
//                 data_.ch[12] = static_cast<int16_t>((buf_[17] >> 4) |
//                                                     ((buf_[18] << 4) & 0x07FF));
//                 data_.ch[13] = static_cast<int16_t>((buf_[18] >> 7) |
//                                                     (buf_[19] << 1) |
//                                                     ((buf_[20] << 9) & 0x07FF));
//                 data_.ch[14] = static_cast<int16_t>((buf_[20] >> 2) |
//                                                     ((buf_[21] << 6) & 0x07FF));
//                 data_.ch[15] = static_cast<int16_t>((buf_[21] >> 5) |
//                                                     ((buf_[22] << 3) & 0x07FF));
//                 /* CH 17 */
//                 data_.ch17 = buf_[23] & CH17_MASK_;
//                 /* CH 18 */
//                 data_.ch18 = buf_[23] & CH18_MASK_;
//                 /* Grab the lost frame */
//                 data_.lost_frame = buf_[23] & LOST_FRAME_MASK_;
//                 /* Grab the failsafe */
//                 data_.failsafe = buf_[23] & FAILSAFE_MASK_;
//                 return true;
//             }
//             else
//             {
//                 return false;
//             }
//         }
//         else
//         {
//             state_ = 0;
//         }
//         prev_byte_ = cur_byte_;
//     }
//     return false;
// }