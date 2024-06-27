#include "Led.h"

Led::Led(PinName led_pin) : m_DigitalOut(led_pin), 
                            m_Thread(osPriorityLow, 4096)

                             
{
    // start thread
    m_Thread.start(callback(this, &Led::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &Led::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

void Led::onLed()
{
    ledOn = true;
    ledBlink = false;
}

void Led::offLed()
{
    ledOn = false;
    ledBlink = false;
}

void Led::blinkLed()
{
    ledOn = false;
    ledBlink = true;
}

void Led::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);
        
        if (ledOn == true){
            m_DigitalOut = 1;
        } else if (ledBlink == true) {
            m_DigitalOut = !m_DigitalOut;
        } else {
            m_DigitalOut = 0;
        }
    }
}


void Led::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}