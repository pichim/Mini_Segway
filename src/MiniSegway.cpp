#include "MiniSegway.h"

#if DO_USE_PPM_IN
MiniSegway::MiniSegway(PpmIn& ppmIn, SerialStream& serialStream) : _remoteCntrl(ppmIn)
#else
MiniSegway::MiniSegway(SBus& sBus, SerialStream& serialStream) : _remoteCntrl(sBus)
#endif
                                   , _Button(MINI_SEGWAY_BUTTON, PullUp)
                                   , _SerialStream(serialStream)
                                   , _Thread(osPriorityHigh, 4096)
{
    _Button.fall(callback(this, &MiniSegway::toggleDoExecute));

    _Thread.start(callback(this, &MiniSegway::threadTask));
    _Ticker.attach(callback(this, &MiniSegway::sendThreadFlag),
                   std::chrono::microseconds{MINI_SEGWAY_PERIOD_US});
}


MiniSegway::~MiniSegway()
{
    _Ticker.detach();
    _Thread.terminate();
}

void MiniSegway::updateRcPkg(rc_pkg_t& rc_pkg)
{
    static uint16_t invalid_rc_pkg_pkg_cntr = 0;
    if (_remoteCntrl.isPkgValid()) {
        invalid_rc_pkg_pkg_cntr = 0;
        // update rc_pkg
        rc_pkg.roll = _remoteCntrl.getChannelMinusToPlusOne(0); 
        rc_pkg.pitch = _remoteCntrl.getChannelMinusToPlusOne(1);
        rc_pkg.throttle = _remoteCntrl.getChannelZeroToPlusOne(2);
        rc_pkg.yaw = _remoteCntrl.getChannelMinusToPlusOne(3);
        rc_pkg.arm = _remoteCntrl.isHigh(MINI_SEGWAY_ARMING_CHANNEL);
        _remoteCntrl.setPkgValidFalse();
    } else {
        invalid_rc_pkg_pkg_cntr++;
        if (invalid_rc_pkg_pkg_cntr > MINI_SEGWAY_NUM_OF_ALLOWED_INVALID_RC_DATA_PKG) {
            invalid_rc_pkg_pkg_cntr = MINI_SEGWAY_NUM_OF_ALLOWED_INVALID_RC_DATA_PKG;
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

void MiniSegway::threadTask()
{
    // additional LED
    DigitalOut led(MINI_SEGWAY_LED);
    led = 0;

    // timer to measure delta time
    Timer timer;
    timer.start();
    microseconds time_previous_us{0};

    // rc package received either from ppm in or sbus
    rc_pkg_t rc_pkg;

    // give the logger 1000 msec time to start
    thread_sleep_for(1000);

    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        // logic so that _do_execute can also be triggered by the start byte via matlab
        static bool is_start_byte_received = false;
        if (!is_start_byte_received && _SerialStream.startByteReceived()) {
            is_start_byte_received = true;
            toggleDoExecute();
        }

// only process the sbus pkg if sbus is used and is running as an own thread
#if !DO_USE_PPM_IN && !SBUS_DO_RUN_AS_THREAD
        // sbus pkg needs to be processed to update the rc_pkg
        // whereas when its ppm in, the rc_pkg is updated via ISR
        _remoteCntrl.processReceivedData();

#endif
        // arm is only true if receiver data is valid and arm button is pressed
        updateRcPkg(rc_pkg);

        // TODO: Use rc_pkg.armed
        // TODO: Use _SerialStream.startByteReceibed()

        // measure delta time
        const microseconds time_us = timer.elapsed_time();
        const float dtime_us_f = duration_cast<microseconds>(time_us - time_previous_us).count();
        time_previous_us = time_us;

        // here lifes the main logic of the mini segway
        if (_do_execute) {
            _SerialStream.write(dtime_us_f);
            _SerialStream.write(rc_pkg.roll);
            _SerialStream.write(rc_pkg.pitch);
            _SerialStream.write(rc_pkg.throttle);
            _SerialStream.write(rc_pkg.yaw);
            _SerialStream.write((rc_pkg.arm ? 1.0f : 0.0f));
#if DO_USE_PPM_IN
            _SerialStream.write(static_cast<float>(_remoteCntrl.getPeriod()) * 1.0e-4f);
#else
            _SerialStream.write(static_cast<float>(_remoteCntrl.getPeriod()) * 2.2222e-04f);
#endif
            _SerialStream.send();

            led = 1;
        } else {
            if (_do_reset) {
                led = 0;
                _SerialStream.reset();
                _do_reset = false;
            }
        }
    }
}

void MiniSegway::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    _Thread.flags_set(_ThreadFlag);
}