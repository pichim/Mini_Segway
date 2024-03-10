#ifndef MINI_SEGWAY_H_
#define MINI_SEGWAY_H_

#include "config.h"

#include "DebounceIn.h"
#include "PpmIn.h"
#include "SBus.h"
#include "SerialStream.h"
#include "ThreadFlag.h"

using namespace std::chrono;

class MiniSegway
{
public:
    explicit MiniSegway(PpmIn& rc, SerialStream& serialStream);
    explicit MiniSegway(SBus& sBus, SerialStream& serialStream);
    virtual ~MiniSegway();

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

#if DO_USE_PPM_IN
    PpmIn& _remoteCntrl;
#else
    SBus& _remoteCntrl;
#endif

    typedef struct rc_pkg_s {
        float roll{0.0f};
        float pitch{0.0f};
        float throttle{0.0f};
        float yaw{0.0f};
        bool arm{false};
    } rc_pkg_t;

    // serial stream either to matlab or to the openlager
    SerialStream& _SerialStream;

    DebounceIn _Button;    
    bool _do_execute{false};
    bool _do_reset{false};

    void updateRcPkg(rc_pkg_t& rc_pkg);
    void toggleDoExecute();
    void threadTask();
    void sendThreadFlag();
};
#endif /* MINI_SEGWAY_H_ */
