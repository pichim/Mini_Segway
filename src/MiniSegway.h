#ifndef MINI_SEGWAY_H_
#define MINI_SEGWAY_H_

#include "config.h"

#include "DebounceIn.h"
#include "PpmIn.h"
#include "SerialStream.h"
#include "ThreadFlag.h"

using namespace std::chrono;

class MiniSegway
{
public:
    explicit MiniSegway(PpmIn& _PpmIn);
    virtual ~MiniSegway();

private:
    // static constexpr int64_t PERIOD_MUS = 500000;

    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    DebounceIn _Button;
    PpmIn& _PpmIn;
    
    bool _do_execute{false};
    bool _do_reset{false};

    typedef struct rc_pkg_s {
        float roll{0.0f};
        float pitch{0.0f};
        float throttle{0.0f};
        float yaw{0.0f};
        bool arm{false};
    } rc_pkg_t;

    void updateRcPkg(rc_pkg_t& rc_pkg);

    void toggleDoExecute();

    void threadTask();
    void sendThreadFlag(); 
};
#endif /* MINI_SEGWAY_H_ */
