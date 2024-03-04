#include "SerialStream.h"

SerialStream::SerialStream(u_int8_t num_of_floats,
                           PinName tx,
                           PinName rx,
                           int baudrate) : _SerialPipe(tx,
                                                       rx,
                                                       baudrate,
                                                       1 + 1, // serial pipe extects 1 byte more
                                                       sizeof(float) * CLAMP(num_of_floats) + 1),
                                                       _buffer_size(sizeof(float) * CLAMP(num_of_floats))
{
}

SerialStream::~SerialStream()
{
}

void SerialStream::write(const float val)
{
    memcpy(&_buffer[_bytes_cntr], &val, sizeof(float));
    _bytes_cntr += sizeof(float);

    // send the data if the buffer is full immediately
    if (_bytes_cntr == _buffer_size)
        send();
}

void SerialStream::send()
{
    // return if there is no data to send
    if (_bytes_cntr == 0)
        return;

    // blocking so that we guarantee that the number of floats is sent once
    // and it will not occupy the buffer for actual data to be sent
    sendNumOfFloatsOnce();

    // TODO: handle case where bytes_writeable < _bytes_cntr
    const int bytes_writeable = _SerialPipe.writeable();
    if (bytes_writeable > 0)
        _SerialPipe.put(_buffer,
                        (_bytes_cntr < bytes_writeable ? _bytes_cntr : bytes_writeable),
                        false);
    _bytes_cntr = 0;
}

bool SerialStream::isStartByteReceived()
{   
    // logic so that we can read non-blocking
    static uint8_t byte = 0;
    if (byte == START_BYTE)
        return true;

    static bool is_byte_received = false;
    const int bytes_readable = _SerialPipe.readable();
    if (!is_byte_received && (bytes_readable > 0)) {
        is_byte_received = true;
        _SerialPipe.get(&byte, 1, false);
    }
    return false;
}

void SerialStream::sendNumOfFloatsOnce()
{
    static bool is_send_once = false;
    if (is_send_once)
        return;
    else {
        is_send_once = true;
        const u_int8_t num_of_floats = _bytes_cntr / sizeof(float);
        _SerialPipe.put(&num_of_floats, 1, true);
    }
}