#include "Led.h"

Led::Led(PinName pin) : _Thread(osPriorityLow, 4096)
                      , _DigitalOut(pin)
                      , _led_on(false)
                      , _led_blink(false)
{
    _Thread.start(callback(this, &Led::threadTask));
    _Ticker.attach(callback(this, &Led::sendThreadFlag), std::chrono::microseconds{MINI_SEGWAY_LED_PERIOD_US});
}

Led::~Led()
{
    _Ticker.detach();
    _Thread.terminate();
}

void Led::on()
{
    _led_on = true;
    _led_blink = false;
}

void Led::off()
{
    _led_on = false;
    _led_blink = false;
}

void Led::blink()
{
    _led_on = false;
    _led_blink = true;
}

void Led::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        // led should either blink
        if (_led_blink) {
            _DigitalOut = !_DigitalOut;
        // if not blinking then it should
        } else {
            // be on
            if (_led_on && !_DigitalOut) {
                _DigitalOut = 1;
            // or off
            } else if (!_led_on && _DigitalOut) {
                _DigitalOut = 0;
            }
        }
    }
}

void Led::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}