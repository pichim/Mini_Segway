#ifndef MINI_SEGWAY_H_
#define MINI_SEGWAY_H_

#include "config.h"

#include "DebounceIn.h"
// #include "eigen/Dense.h"
#include "Encoder.h"
#include "IMU.h"
#include "Motor.h"
#include "PpmIn.h"
#include "SBus.h"
#include "SerialStream.h"
#include "ThreadFlag.h"

using namespace std::chrono;

class MiniSegway
{
public:
#if DO_USE_PPM_IN
    explicit MiniSegway(PpmIn& rc, IMU& imu);
#else
    explicit MiniSegway(SBus& rc, IMU& imu);
#endif
    virtual ~MiniSegway();

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

#if DO_USE_PPM_IN
    PpmIn &_rc;
#else
    SBus &_rc;
#endif

    IMU &_imu;

    typedef struct rc_pkg_s {
        float turn_rate{0.0f};
        float forward_speed{0.0f};
        bool armed{false};
    } rc_pkg_t;

    DebounceIn _Button;    
    bool _do_execute{false};
    bool _do_reset{false};

    void updateRcPkg(rc_pkg_t& rc_pkg);
    void toggleDoExecute();
    float evaluateEncoder(EncoderCounter& encoder, long& counts);
    void threadTask();
    void sendThreadFlag();
};
#endif /* MINI_SEGWAY_H_ */
