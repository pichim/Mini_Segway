#include "MiniSegway.h"

MiniSegway::MiniSegway(PpmIn& PpmIn) : _Button(BUTTON, PullUp)
                                     , _PpmIn(PpmIn)
                                     , _Thread(osPriorityHigh, 4096)
{
    _Button.fall(callback(this, &MiniSegway::toggleDoExecute));

    // start thread
    _Thread.start(callback(this, &MiniSegway::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    _Ticker.attach(callback(this, &MiniSegway::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

MiniSegway::~MiniSegway()
{
}

void MiniSegway::threadTask()
{
    DigitalOut led(LED);
    led = 0;

    Timer timer;
    microseconds time_previous_us{0};

    SerialStream serialStream(NUM_OF_FLOATS_MAX, TX, RX, BAUDRATE);

    rc_pkg_t rc_pkg;

    // give the logger 1000 msec time to start
    thread_sleep_for(1000);

    timer.start();

    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        // arm is only true if receiver data is valid and arm button is pressed
        updateRcPkg(rc_pkg);

        // TODO: Use rc_pkg.armed

        const microseconds time_mus = timer.elapsed_time();
        const float dtime_mus_f = duration_cast<microseconds>(time_mus - time_previous_us).count();
        time_previous_us = time_mus;

        if (_do_execute) {
            if (serialStream.isStartByteReceived()) {
                serialStream.write(dtime_mus_f);
                serialStream.write(rc_pkg.roll);
                serialStream.write(rc_pkg.pitch);
                serialStream.write(rc_pkg.throttle);
                serialStream.write(rc_pkg.yaw);
                serialStream.write((rc_pkg.arm ? 1.0f : 0.0f));
                serialStream.send();
            }
            led = 1;
        } else {
            if (_do_reset) {
                led = 0;
                _do_reset = false;
            }
        }
    }
}

void MiniSegway::updateRcPkg(rc_pkg_t& rc_pkg)
{
    static uint16_t invalid_rc_pkg_pkg_cntr = 0;
    if (_PpmIn.isPkgValid()) {
        invalid_rc_pkg_pkg_cntr = 0;
        // update rc_pkg
        rc_pkg.roll = _PpmIn.getChannelMinusToPlusOne(0); 
        rc_pkg.pitch = _PpmIn.getChannelMinusToPlusOne(1);
        rc_pkg.throttle = _PpmIn.getChannelZeroToPlusOne(2);
        rc_pkg.yaw = _PpmIn.getChannelMinusToPlusOne(3);
        rc_pkg.arm = _PpmIn.isHigh(4);
    } else {
        invalid_rc_pkg_pkg_cntr++;
        if (invalid_rc_pkg_pkg_cntr > NUM_OF_ALLOWED_INVALID_RC_DATA_PKG) {
            invalid_rc_pkg_pkg_cntr = NUM_OF_ALLOWED_INVALID_RC_DATA_PKG;
            // reset rc_pkg
            rc_pkg.roll = 0.0f; 
            rc_pkg.pitch = 0.0f;
            rc_pkg.throttle = 0.0f;
            rc_pkg.yaw = 0.0f;
            rc_pkg.arm = false;
        }
    }
}

void MiniSegway::toggleDoExecute()
{
    _do_execute = !_do_execute;
    if (_do_execute)
        _do_reset = true;
}

void MiniSegway::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    _Thread.flags_set(_ThreadFlag);
}

