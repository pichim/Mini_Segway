#include "MiniSegway.h"

#if DO_USE_PPM_IN
MiniSegway::MiniSegway(PpmIn& rc, IMU& imu)
#else
MiniSegway::MiniSegway(SBus& rc, IMU& imu)
#endif
                                 : _rc(rc)
                                 , _imu(imu)
                                 , _Button(MINI_SEGWAY_BUTTON, PullUp)
                                 , _Thread(osPriorityHigh, 4096)
{
    _Button.fall(callback(this, &MiniSegway::toggleDoExecute));

    _Thread.start(callback(this, &MiniSegway::threadTask));
    _Ticker.attach(callback(this, &MiniSegway::sendThreadFlag), microseconds{MINI_SEGWAY_PERIOD_US});
}

MiniSegway::~MiniSegway()
{
    _Ticker.detach();
    _Thread.terminate();
}

void MiniSegway::updateRcPkg(rc_pkg_t& rc_pkg)
{
    static uint16_t invalid_rc_pkg_pkg_cntr = 0;
    if (_rc.isPkgValid()) {
        invalid_rc_pkg_pkg_cntr = 0;
        // update rc_pkg
        rc_pkg.turn_rate     = _rc.getChannelMinusToPlusOne(0);        // right stick left to right (roll)
        rc_pkg.forward_speed = _rc.getChannelMinusToPlusOne(1);        // right stick down to up (pitch)
        rc_pkg.armed         = _rc.isHigh(MINI_SEGWAY_ARMING_CHANNEL); // arm button
        _rc.setPkgValidFalse();
    } else {
        invalid_rc_pkg_pkg_cntr++;
        if (invalid_rc_pkg_pkg_cntr > MINI_SEGWAY_NUM_OF_ALLOWED_INVALID_RC_DATA_PKG) {
            invalid_rc_pkg_pkg_cntr = MINI_SEGWAY_NUM_OF_ALLOWED_INVALID_RC_DATA_PKG;
            // reset rc_pkg
            rc_pkg.turn_rate     = 0.0f; 
            rc_pkg.forward_speed = 0.0f;
            rc_pkg.armed         = false;
        }
    }
}

void MiniSegway::toggleDoExecute()
{
    _do_execute = !_do_execute;
    if (_do_execute)
        _do_reset = true;
}

float MiniSegway::evaluateEncoder(EncoderCounter& encoder, long& counts)
{  
    // avoid overflow
    static short counts_previous{0};
    const short counts_actual = encoder.read();
    const short counts_delta = counts_actual - counts_previous;
    counts_previous = counts_actual;
    counts += counts_delta;
    return static_cast<float>(counts_delta) / MINI_SEGWAY_COUNTS_PER_TURN;
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

    // serial stream either to matlab or to the openlager
    DigitalOut enable_motor_driver(MINI_SEGWAY_ENABLE_MOTOR_DRIVER);
    SerialStream serialStream(MINI_SEGWAY_NUM_OF_FLOATS,
                              MINI_SEGWAY_TX,
                              MINI_SEGWAY_RX,
                              MINI_SEGWAY_BAUDRATE);
    
    // encoders
    Encoder encoder_M1(MINI_SEGWAY_ENCA_M1,
                       MINI_SEGWAY_ENCB_M1,
                       MINI_SEGWAY_COUNTS_PER_TURN,
                       MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY,
                       static_cast<float>(MINI_SEGWAY_PERIOD_US) * 1.0e-6f);
    // Encoder encoder_M2(MINI_SEGWAY_ENCA_M2,
    //                    MINI_SEGWAY_ENCB_M2,
    //                    MINI_SEGWAY_COUNTS_PER_TURN,
    //                    MINI_SEGWAY_VELOCITY_FILTER_FREQUENCY,
    //                    static_cast<float>(MINI_SEGWAY_PERIOD_US) * 1.0e-6f);
    Encoder::encoder_signals_t encoder_signals_M1 = encoder_M1.read();
    // Encoder::encoder_signals_t encoder_signals_M2 = encoder_M2.read();
    // TODO: remove rotations_previous_M1 and rotations_previous_M2
    float rotations_previous_M1{encoder_signals_M1.rotations};
    // float rotations_previous_M2{encoder_signals_M2.rotations};

    // motors
    Motor motor_M1(MINI_SEGWAY_PWM_M1, MINI_SEGWAY_VOLTAGE_MAX);
    Motor motor_M2(MINI_SEGWAY_PWM_M2, MINI_SEGWAY_VOLTAGE_MAX);

    // imu
    ImuData imu_data;

    // give the logger 1000 msec time to start
    thread_sleep_for(1000);

    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

        // logic so that _do_execute can also be triggered by the start byte via matlab
        static bool is_start_byte_received = false;
        if (!is_start_byte_received && serialStream.startByteReceived()) {
            is_start_byte_received = true;
            toggleDoExecute();
        }

// only process the sbus pkg if sbus is used and is running as an own thread
#if !DO_USE_PPM_IN && !SBUS_DO_RUN_AS_THREAD
        // sbus pkg needs to be processed to update the rc_pkg
        // whereas when its ppm in, the rc_pkg is updated via ISR
        _rc.processReceivedData();
#endif
        // arm is only true if receiver data is valid and arm button is pressed
        updateRcPkg(rc_pkg);

        // read motor signals
        encoder_signals_M1 = encoder_M1.read();
        // encoder_signals_M2 = encoder_M2.read();

        // read imu data
        imu_data = _imu.getImuData();

        // enable motor drivers and write output to the motors only if armed
        if (rc_pkg.armed) {
            if (enable_motor_driver == 0)
                enable_motor_driver = 1;
                motor_M1.setVoltage(rc_pkg.forward_speed * MINI_SEGWAY_VOLTAGE_MAX);
                motor_M2.setVoltage(rc_pkg.forward_speed * MINI_SEGWAY_VOLTAGE_MAX);
        } else {
            if (enable_motor_driver == 1) {
                enable_motor_driver = 0;
                motor_M1.reset();
                motor_M2.reset();
            }
        }

        // measure delta time
        const microseconds time_us = timer.elapsed_time();
        const float dtime_us_f = duration_cast<microseconds>(time_us - time_previous_us).count();
        time_previous_us = time_us;

        // TODO: Use rc_pkg.armed
        // TODO: Use serialStream.startByteReceibed()
        // here lifes the main logic of the mini segway
        if (_do_execute) {
            serialStream.write(dtime_us_f);
            serialStream.write(rc_pkg.forward_speed);
            serialStream.write(rc_pkg.turn_rate);
            serialStream.write((rc_pkg.armed ? 1.0f : 0.0f));
#if DO_USE_PPM_IN
            serialStream.write(static_cast<float>(_rc.getPeriod()) * 1.0e-4f);
#else
            serialStream.write(static_cast<float>(_rc.getPeriod()) * 2.2222e-04f);
#endif
            serialStream.write(encoder_signals_M1.rotations - rotations_previous_M1);
            rotations_previous_M1 = encoder_signals_M1.rotations;
            serialStream.write(encoder_signals_M1.velocity);
            serialStream.write(encoder_signals_M1.rotations);
            // serialStream.write(encoder_signals_M2.rotations - rotations_previous_M2);
            // rotations_previous_M2 = encoder_signals_M2.rotations;
            // serialStream.write(encoder_signals_M2.velocity);
            // serialStream.write(encoder_signals_M2.rotations);
            serialStream.write(imu_data.gyro(0));
            serialStream.write(imu_data.gyro(1));
            serialStream.write(imu_data.gyro(2));
            serialStream.write(imu_data.acc(0));
            serialStream.write(imu_data.acc(1));
            serialStream.write(imu_data.acc(2));
            serialStream.write(imu_data.rpy(0));
            serialStream.write(imu_data.rpy(1));
            serialStream.write(imu_data.rpy(2));
            serialStream.send();

            led = 1;
        } else {
            if (_do_reset) {
                led = 0;
                serialStream.reset();
                encoder_M1.reset();
                rotations_previous_M1 = 0;
                // encoder_M2.reset();
                // rotations_previous_M2 = 0;
                motor_M1.reset();
                motor_M2.reset();
                _do_reset = false;
            }
        }
    }
}

void MiniSegway::sendThreadFlag()
{
    _Thread.flags_set(_ThreadFlag);
}