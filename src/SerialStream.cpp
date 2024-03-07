#include "SerialStream.h"

SerialStream::SerialStream(u_int8_t num_of_floats,
                           PinName tx,
                           PinName rx,
                           int baudrate) : _buffer_size(sizeof(float) * CLAMP(num_of_floats))
#if DO_USE_SERIAL_PIPE
                                         , _SerialPipe(tx, rx, baudrate, 1 + 1, // serial pipe extects 1 byte more
                                                       sizeof(float) * CLAMP(num_of_floats) + 1)
{
#else
                                         , _BufferedSerial(tx, rx, baudrate)
{
    _BufferedSerial.set_blocking(false);
#endif
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

#if DO_USE_SERIAL_PIPE
    // TODO: handle case where bytes_writeable < _bytes_cntr
    const int bytes_writeable = _SerialPipe.writeable();
    if (bytes_writeable > 0)
        _SerialPipe.put(_buffer, _bytes_cntr, false);
#else
    if (_BufferedSerial.writable())
        // TODO: handle case where bytes_written < _bytes_cntr
        ssize_t bytes_written = _BufferedSerial.write(_buffer, _bytes_cntr);
#endif
    _bytes_cntr = 0;
}

bool SerialStream::startByteReceived()
{   
    return checkByteReceived(_start, START_BYTE);
}

void SerialStream::reset()
{
    memset(_buffer, 0, sizeof(_buffer));
    _bytes_cntr = 0;
    resetByteMsg(_start);
    _send_num_of_floats_once = false;
}

bool SerialStream::checkByteReceived(byte_msg_t& byte_msg, const uint8_t byte_expected)
{
    // logic so that we can read non-blocking
    if (byte_msg.byte == byte_expected)
        return true;

#if DO_USE_SERIAL_PIPE
    const int bytes_readable = _SerialPipe.readable();
    if (!byte_msg.received && (bytes_readable > 0)) {
        byte_msg.received = true;
        _SerialPipe.get(&byte_msg.byte, 1, false);
    }
#else
    if (!byte_msg.received && _BufferedSerial.readable()) {
        byte_msg.received = true;
        _BufferedSerial.read(&byte_msg.byte, 1);
    }
#endif
    return false;
}

void SerialStream::resetByteMsg(byte_msg_t& byte_msg)
{
    byte_msg.byte = 0;
    byte_msg.received = false;
}

void SerialStream::sendNumOfFloatsOnce()
{
    if (_send_num_of_floats_once)
        return;
    else {
        _send_num_of_floats_once = true;
        const u_int8_t num_of_floats = _bytes_cntr / sizeof(float);
#if DO_USE_SERIAL_PIPE
        _SerialPipe.put(&num_of_floats, 1, true);
#else
        _BufferedSerial.write(&num_of_floats, 1);
#endif
    }
}