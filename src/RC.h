#pragma once

#include "config.h"

#include "IIRFilter.h"
#include "SBus.h"

#define RC_PRINT_DEBUG false

class RC
{
public:
    explicit RC(PinName pin);
    virtual ~RC() {};

    typedef struct rc_pkg_s {
        float turn_rate{0.0f};
        float forward_speed{0.0f};
        bool armed{false};
    } rc_pkg_t;

    void processReceivedData();
    float getPeriod();
    rc_pkg_t update();

private:
    SBus _rc;
    IIRFilter _upsamplingLowPass2[2];

    rc_pkg_t _rc_pkg;

    float applyExpoMinusToPlusOne(float val);
};