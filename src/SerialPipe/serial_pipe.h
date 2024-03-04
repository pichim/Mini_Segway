/* Copyright (c) 2017 Michael Ammann
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SERIAL_PIPE_H
#define SERIAL_PIPE_H

#include "mbed.h"
#include "pipe.h"

#define _SerialPipeBase SerialBase //!< base class used by this class

/** Buffered serial interface (rtos capable/interrupt driven)
*/
class SerialPipe : public _SerialPipeBase
{
public:
    /** Constructor
        \param tx the trasmitting pin
        \param rx the receiving pin
        \param baudate the serial baud rate
        \param rxSize the size of the receiving buffer
        \param txSize the size of the transmitting buffer
    */
    SerialPipe(PinName tx, PinName rx, int baudrate, int rxSize = 128, int txSize = 128);
    
    /** Destructor
    */
    virtual ~SerialPipe(void);
    
    // tx channel
    //----------------------------------------------------
    
    /** check if writable 
        return the number of free bytes
    */
    int writeable(void);
    
    /** send a character (blocking)
        \param c the character to send
        \return c
    */
    int putc(int c);
    
    /** send a buffer
        \param buffer the buffer to send
        \param length the size of the buffer to send
        \param blocking, if true this function will block 
               until all bytes placed in the buffer. 
        \return the number of bytes written 
    */
    int put(const void* buffer, int length, bool blocking);
    
    // rx channel
    //----------------------------------------------------
    
    /** check if readable
     *
        \return the size available in the buffer.
    */
    int readable(void);
    
    /** receive one character from the serial port (blocking)
        \param the character received 
    */
    int getc(void);
    
    /** read a buffer from the serial port
        \param pointer to the buffer to read.
        \param length number of bytes to read 
        \param blocking true if all bytes shall be read. false if only the available bytes.
        \return the number of bytes read.
    */
    int get(void* buffer, int length, bool blocking);
    
protected:
    //! receive interrupt routine
    void rxIrqBuf(void);
    //! transmit interrupt woutine 
    void txIrqBuf(void);
    //! start transmission helper
    void txStart(void);
    //! move bytes to hardware
    void txCopy(void);
    Pipe<char> _pipeRx; //!< receive pipe
    Pipe<char> _pipeTx; //!< transmit pipe
};

#endif

// End Of File
