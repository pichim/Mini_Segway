#include "RC.h"

RC::RC(PinName pin) : _rc(pin)
{
    _upsamplingLowPass2[0].lowPass2Init(MINI_SEGWAY_RC_TURN_RATE_UPSAMPLING_FILTER_FREQUENCY_HZ,
                                        MINI_SEGWAY_RC_UPSAMPLING_FILTER_DAMPING,
                                        MINI_SEGWAY_TS);
    _upsamplingLowPass2[1].lowPass2Init(MINI_SEGWAY_RC_FORWARD_SPEED_UPSAMPLING_FILTER_FREQUENCY_HZ,
                                        MINI_SEGWAY_RC_UPSAMPLING_FILTER_DAMPING,
                                        MINI_SEGWAY_TS);
}

void RC::processReceivedData()
{
    _rc.processReceivedData();
}

float RC::getPeriod()
{
    return static_cast<float>(_rc.getPeriod());
}

RC::rc_pkg_t RC::update()
{
    static uint16_t valid_rc_pkg_cntr = 0;
    static uint16_t invalid_rc_pkg_cntr = 0;

    static float turn_rate = 0.0f;
    static float forward_speed = 0.0f;
    static bool armed = false;
    static bool mode = false;

    static bool reset_filters = true;

    if (_rc.isPkgValid()) {

#if RC_PRINT_DEBUG
        static Timer timer;
        timer.start();
        static microseconds time_previous;
        static uint8_t cntr = 0;
        if (++cntr % 111 * 100 == 0) {
            cntr = 0;
            const microseconds time = timer.elapsed_time();
            uint32_t period = duration_cast<microseconds>(time - time_previous).count();
            time_previous = time;

            const uint8_t failsafe = _rc.isFailSafe() ? 1.0 : 0.0;
            printf("period : %d, arming channel: %d, failsafe: %d, num of dec err.: %d, num of lost frames: %d, num of skipped start frames: %d\n",
                   period / 100,
                   _rc.getChannel(MINI_SEGWAY_RC_ARMING_CHANNEL),
                   failsafe,
                   _rc.getNumOfDecoderErrorFrames(),
                   _rc.getNumOfLostFrames(),
                   _rc.getNumOfSkippedStartFrames());
            printf("ch0: %d, ch1: %d, ch2: %d, ch3: %d, ch4: %d, ch5: %d, ch6: %d, ch7: %d\n",
                   _rc.getChannel(0),
                   _rc.getChannel(1),
                   _rc.getChannel(2),
                   _rc.getChannel(3),
                   _rc.getChannel(4),
                   _rc.getChannel(5),
                   _rc.getChannel(6),
                   _rc.getChannel(7));
        }
#endif

        // update counters
        valid_rc_pkg_cntr++;
        if (valid_rc_pkg_cntr > MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG)
            valid_rc_pkg_cntr = MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG;
        invalid_rc_pkg_cntr = 0;
        // update _rc_pkg
#if MINI_SEGWAY_RC_APPLY_EXPO
        turn_rate     = applyExpoMinusToPlusOne(_rc.getChannelMinusToPlusOne(MINI_SEGWAY_RC_TURN_RATE_CHANNEL));
        forward_speed = applyExpoMinusToPlusOne(_rc.getChannelMinusToPlusOne(MINI_SEGWAY_RC_FORWARD_SPEED_CHANNEL));
#else
        turn_rate     = _rc.getChannelMinusToPlusOne(MINI_SEGWAY_RC_TURN_RATE_CHANNEL);
        forward_speed = _rc.getChannelMinusToPlusOne(MINI_SEGWAY_RC_FORWARD_SPEED_CHANNEL);
#endif
        armed = _rc.isHigh(MINI_SEGWAY_RC_ARMING_CHANNEL);
        mode = _rc.isHigh(MINI_SEGWAY_RC_MODE_CHANNEL);
        _rc.setPkgValidFalse();
    } else {
        invalid_rc_pkg_cntr++;
        if (invalid_rc_pkg_cntr > MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG) {
            invalid_rc_pkg_cntr = MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG;
            valid_rc_pkg_cntr = 0;
            reset_filters = true;
        }
    }
    
    // upsampling rc_pkg data
    if (valid_rc_pkg_cntr < MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG ||
        invalid_rc_pkg_cntr == MINI_SEGWAY_RC_NUM_OF_ALLOWED_INVALID_DATA_PKG) {
        _rc_pkg.turn_rate     = 0.0f;
        _rc_pkg.forward_speed = 0.0f;
        _rc_pkg.armed = false;
        _rc_pkg.mode = false;
    } else {
#if MINI_SEGWAY_RC_USE_UPSAMPLING_FILTERS
        if (reset_filters) {
            reset_filters = false;
            _upsamplingLowPass2[0].reset(turn_rate);
            _upsamplingLowPass2[1].reset(forward_speed);
        }
        _rc_pkg.turn_rate     = _upsamplingLowPass2[0].apply(turn_rate);
        _rc_pkg.forward_speed = _upsamplingLowPass2[1].apply(forward_speed);
#else
        _rc_pkg.turn_rate     = turn_rate;
        _rc_pkg.forward_speed = forward_speed;
#endif
        _rc_pkg.armed = armed;
        _rc_pkg.mode = mode;
    }

    return _rc_pkg;
}

float RC::applyExpoMinusToPlusOne(float x)
{
    static const float scale = 1.0f / ( powf( expf(1.0f), MINI_SEGWAY_RC_EXPO_ALPHA ) - 1.0f );
    return copysignf(scale, x) * ( powf( expf(fabsf(x)), MINI_SEGWAY_RC_EXPO_ALPHA ) - 1.0f );
}