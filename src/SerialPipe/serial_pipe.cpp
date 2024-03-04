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

#include "serial_pipe.h"

SerialPipe::SerialPipe(PinName tx, PinName rx, int baudrate, int rxSize, int txSize) :
            _SerialPipeBase(tx, rx, baudrate),
            _pipeRx( (rx!=NC) ? rxSize : 0), 
            _pipeTx( (tx!=NC) ? txSize : 0)
{
    if (rx!=NC) {
        attach(callback(this, &SerialPipe::rxIrqBuf), RxIrq);
    }
}

SerialPipe::~SerialPipe(void)
{
    attach(NULL, RxIrq);
    attach(NULL, TxIrq);
}

// tx channel
int SerialPipe::writeable(void)    
{
    return _pipeTx.free();
}

int SerialPipe::putc(int c)    
{
    c = _pipeTx.putc(c);
    txStart();
    return c;
}

int SerialPipe::put(const void* buffer, int length, bool blocking)    
{ 
    int count = length;
    const char* ptr = (const char*)buffer;
    if (count) {
        do {
            int written = _pipeTx.put(ptr, count, false);
            if (written) {
                ptr += written;
                count -= written;
                txStart();
            }
            else if (!blocking) {
                /* nothing / just wait */;
                break;
            }
        }
        while (count);
    }
    
    return (length - count);
}

void SerialPipe::txCopy(void)
{
    while (_SerialPipeBase::writeable() && _pipeTx.readable()) {
        char c = _pipeTx.getc();
        _SerialPipeBase::_base_putc(c);
    }
}

void SerialPipe::txIrqBuf(void)
{
    txCopy();
    // detach tx isr if we are done 
    if (!_pipeTx.readable()) {
        attach(NULL, TxIrq);
    }
}

void SerialPipe::txStart(void)
{
    // disable the tx isr to avoid interruption
    attach(NULL, TxIrq);
    txCopy();
    // attach the tx isr to handle the remaining data
    if (_pipeTx.readable()) {
        attach(callback(this, &SerialPipe::txIrqBuf), TxIrq);
    }
}

// rx channel
int SerialPipe::readable(void)                      
{ 
    return _pipeRx.readable(); 
} 

int SerialPipe::getc(void)                          
{ 
    if (!_pipeRx.readable()) {
        return EOF;
    }

    return _pipeRx.getc(); 
} 

int SerialPipe::get(void* buffer, int length, bool blocking) 
{ 
    return _pipeRx.get((char*)buffer,length,blocking); 
}

void SerialPipe::rxIrqBuf(void)
{
    while (_SerialPipeBase::readable())
    {
        char c = _SerialPipeBase::_base_getc();
        if (_pipeRx.writeable()) {
            _pipeRx.putc(c);
        } else {
            /* overflow */
        }
    }
}
