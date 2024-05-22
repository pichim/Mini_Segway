#ifndef RC_
#define RC_

#include "config.h"

#include "IIR_Filter.h"
#if DO_USE_PPM_IN
    #include "PpmIn.h"
#else
    #include "SBus.h"
#endif

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

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
#if DO_USE_PPM_IN
    PpmIn _rc;
#else
    SBus _rc;
#endif
    IIR_Filter _upsampling_filters[2];

    rc_pkg_t _rc_pkg;

    float applyExpoMinusToPlusOne(float val);
};

#endif /* RC_ */
