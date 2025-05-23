#include "Servo.h"

Servo::Servo(PinName pin) : _Thread(osPriorityNormal, 4096)
                          , _DigitalOut(pin)
                          , _val(0.0f)
{
    _Thread.start(callback(this, &Servo::threadTask));
    _Ticker.attach(callback(this, &Servo::sendThreadFlag), microseconds{MINI_SEGWAY_SERVO_PERIOD_US});
}

Servo::~Servo()
{
    _Ticker.detach();
    _Timeout.detach();
    _Thread.terminate();
}

void Servo::write(float val)
{
    _val = val;
}

void Servo::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        writeAngleAsSoftPWM(_val);
    }
}

void Servo::writeAngleAsSoftPWM(float val)
{
    // if calibrated, input argument val is in radians

    // // clamp angle to (-MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX, MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX)
    // val = (val < -MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX) ? -MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX :
    //       (val >  MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX) ?  MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX :
    //        val;

    // // map from (-MINI_SEGWAY_SERVO_VALUE_RAD_MAX, MINI_SEGWAY_SERVO_VALUE_RAD_MAX) -> (0.0f, 1.0f)
    // val = _angle_gain * val + _angle_offset;

    // // map from (0.0f, 1.0f) -> (MINI_SEGWAY_SERVO_VALUE_MIN, MINI_SEGWAY_SERVO_VALUE_MAX)
    // val = _normalised_gain * val + _normalised_offset;


    // simplified mapping
    // clamp val to (MINI_SEGWAY_SERVO_VALUE_RAD_MIN, MINI_SEGWAY_SERVO_VALUE_RAD_MAX)
    val = (val < MINI_SEGWAY_SERVO_VALUE_RAD_MIN) ? MINI_SEGWAY_SERVO_VALUE_RAD_MIN :
          (val > MINI_SEGWAY_SERVO_VALUE_RAD_MAX) ? MINI_SEGWAY_SERVO_VALUE_RAD_MAX :
          val;

    val = (val * _angle_gain) + _normalised_offset;
    
    // convert to pulse width
    const uint16_t pulse_mus = static_cast<uint16_t>(val * static_cast<float>(MINI_SEGWAY_SERVO_PERIOD_US));

    // enable digital output and attach disableDigitalOutput() to timeout for soft PWM
    enableDigitalOutput();
    _Timeout.attach(callback(this, &Servo::disableDigitalOutput), microseconds{pulse_mus});
}

void Servo::enableDigitalOutput()
{
    _DigitalOut = 1;
}

void Servo::disableDigitalOutput()
{
    _DigitalOut = 0;
}

void Servo::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}
