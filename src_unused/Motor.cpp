#include "Motor.h"

Motor::Motor(PinName pwm,
             PinName dout,
             float voltage_max) : _Thread(osPriorityHigh1, 4096)
                                , _pwm(pwm)
                                , _dir(dout)
                                , _voltage(0.0f)
                                , _voltage_max(voltage_max)
{
    _pwm.period_mus(MINI_SEGWAY_PWM_PERIOD_US);
    _upsamplingLowPass1.lowPass1Init(MINI_SEGWAY_PWM_UPSAMPLING_FILTER_FREQUENCY_HZ,
                                     MINI_SEGWAY_PWM_TS);

    _Thread.start(callback(this, &Motor::threadTask));
    _Ticker.attach(callback(this, &Motor::sendThreadFlag), microseconds{MINI_SEGWAY_PWM_PERIOD_US});
}

Motor::~Motor()
{
    _Ticker.detach();
    _Thread.terminate();
}

float Motor::setVoltage(float voltage)
{
    _voltage = voltage;
}

void Motor::threadTask()
{
    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);
        
        float voltage_filtered = _upsamplingLowPass1.applyConstrained(_voltage, -MINI_SEGWAY_MOTOR_VOLTAGE_MAX, MINI_SEGWAY_MOTOR_VOLTAGE_MAX);
        
        const float sign = copysignf(1.0f, voltage_filtered);

        _dir = (sign > 0.0f) ? 1 : 0;
        voltage_filtered = fabsf(voltage_filtered);
        float duty_cycle = voltage_filtered / _voltage_max;
        duty_cycle = (duty_cycle < MOTOR_DUTY_CYCLE_MIN_VALUE) ? MOTOR_DUTY_CYCLE_MIN_VALUE :
                    (duty_cycle > MOTOR_DUTY_CYCLE_MAX_VALUE) ? MOTOR_DUTY_CYCLE_MAX_VALUE :
                    duty_cycle;
        _pwm.write(duty_cycle);
    }
}

void Motor::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}