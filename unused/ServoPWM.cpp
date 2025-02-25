#include "Servo.h"

Servo::Servo(PinName pin, int us) : _Thread(osPriorityNormal, 4096)
                                  , _FastPWM(pin, us)
                                  , _val(0.0f)
                            
{
    writeAngleAsPWM(0.0f);
    _Thread.start(callback(this, &Servo::threadTask));
    _Ticker.attach(callback(this, &Servo::sendThreadFlag), microseconds{MINI_SEGWAY_SERVO_PERIOD_US});
}

Servo::~Servo()
{
    _Ticker.detach();
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

        writeAngleAsPWM(_val);
    }
}

void Servo::writeAngleAsPWM(float val)
{
    // if calibrated, input argument val is in radians

    // clamp angle to (-MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX, MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX)
    val = (val < -MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX) ? -MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX :
          (val >  MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX) ?  MINI_SEGWAY_SERVO_VALUE_CLAMP_RAD_MAX :
           val;

    // map from (-MINI_SEGWAY_SERVO_VALUE_RAD_MAX, MINI_SEGWAY_SERVO_VALUE_RAD_MAX) -> (0.0f, 1.0f)
    val = _angle_gain * val + _angle_offset;

    // map from (0.0f, 1.0f) -> (MINI_SEGWAY_SERVO_VALUE_MIN, MINI_SEGWAY_SERVO_VALUE_MAX)
    val = _normalised_gain * val + _normalised_offset;

    // write pwm to servo
    _FastPWM.write(val);
}

void Servo::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}
