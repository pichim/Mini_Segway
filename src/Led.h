#pragma once

#include "mbed.h"
#include "ThreadFlag.h"

#ifndef LED_H_
#define LED_H_

class Led
{
public:
    explicit Led(PinName led_pin);
    virtual ~Led() {};

    void onLed();
    void offLed();
    void blinkLed();

private:
    static constexpr int64_t PERIOD_MUS = 200000;
    bool ledOn = false;
    bool ledBlink = false;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;
    DigitalOut m_DigitalOut;

    void threadTask();
    void sendThreadFlag();

};




#endif /* MOTION_H_ */