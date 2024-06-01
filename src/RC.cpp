#include "RC.h"

RC::RC(PinName pin) : _rc(pin)
                    , _upsampling_filters{IIR_Filter(MINI_SEGWAY_RC_UPSAMPLING_FREQUENCY_RAD_SEC,
                                                     MINI_SEGWAY_RC_UPSAMPLING_DAMPING, MINI_SEGWAY_TS,
                                                     1.0f),
                                          IIR_Filter(MINI_SEGWAY_RC_UPSAMPLING_FREQUENCY_RAD_SEC,
                                                     MINI_SEGWAY_RC_UPSAMPLING_DAMPING, MINI_SEGWAY_TS,
                                                     1.0f)}
{}

void RC::processReceivedData()
{
#if !DO_USE_PPM_IN
    _rc.processReceivedData();
#endif
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

    static bool reset_filters = true;

    if (_rc.isPkgValid()) {
        // update counters
        valid_rc_pkg_cntr++;
        if (valid_rc_pkg_cntr > MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG)
            valid_rc_pkg_cntr = MINI_SEGWAY_RC_NUM_OF_NECESSARY_VALID_DATA_PKG;
        invalid_rc_pkg_cntr = 0;
        // update _rc_pkg
#if MINI_SEGWAY_RC_APPLY_EXPO
        turn_rate     = applyExpoMinusToPlusOne(_rc.getChannelMinusToPlusOne(0)); // right stick left to right
        forward_speed = applyExpoMinusToPlusOne(_rc.getChannelMinusToPlusOne(2)); // left  stick down to up
#else
        turn_rate     = _rc.getChannelMinusToPlusOne(0); // right stick left to right
        forward_speed = _rc.getChannelMinusToPlusOne(2); // left  stick down to up
#endif
        armed  = _rc.isHigh(MINI_SEGWAY_RC_ARMING_CHANNEL); // arm button
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
    } else {
#if MINI_SEGWAY_RC_USE_UPSAMPLING_FILTERS
        if (reset_filters) {
            reset_filters = false;
            _upsampling_filters[0].reset(turn_rate);
            _upsampling_filters[1].reset(forward_speed);
        }
        _rc_pkg.turn_rate     = _upsampling_filters[0].filter(turn_rate);
        _rc_pkg.forward_speed = _upsampling_filters[1].filter(forward_speed);
#else
        _rc_pkg.turn_rate     = turn_rate;
        _rc_pkg.forward_speed = forward_speed;
#endif
        _rc_pkg.armed = armed;
    }

    return _rc_pkg;
}

float RC::applyExpoMinusToPlusOne(float x)
{
    static const float scale = 1.0f / ( powf( expf(1.0f), MINI_SEGWAY_RC_EXPO_ALPHA ) - 1.0f );
    return copysignf(scale, x) * ( powf( expf(fabsf(x)), MINI_SEGWAY_RC_EXPO_ALPHA ) - 1.0f );
}
