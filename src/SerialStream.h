#ifndef SERIAL_STREAM_H_
#define SERIAL_STREAM_H_

#include "SerialPipe/serial_pipe.h"

#define NUM_OF_FLOATS_MAX 20
#define CLAMP(x) (x <= NUM_OF_FLOATS_MAX ? x : NUM_OF_FLOATS_MAX)
#define START_BYTE 255

class SerialStream {
public:
    explicit SerialStream(u_int8_t num_of_floats,
                          PinName tx,
                          PinName rx,
                          int baudrate = 2000000);
    virtual ~SerialStream();

    void write(const float val);
    void send();
    bool isStartByteReceived();

private:
    SerialPipe _SerialPipe;
    char _buffer[4 * NUM_OF_FLOATS_MAX];
    u_int8_t _buffer_size;
    u_int8_t _bytes_cntr{0};

    void sendNumOfFloatsOnce();
};

#endif /* SERIAL_STREAM_H_ */
