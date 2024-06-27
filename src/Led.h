#pragma once

#include "config.h"

#include "mbed.h"

#include "ThreadFlag.h"

class Led
{
public:
    explicit Led(PinName pin);
    virtual ~Led();

    void on();
    void off();
    void blink();

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    DigitalOut _DigitalOut;

    bool _led_on;
    bool _led_blink;

    void threadTask();
    void sendThreadFlag();

};