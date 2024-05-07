#ifndef MINI_SEGWAY_H_
#define MINI_SEGWAY_H_

#include "config.h"

#include "eigen/Dense.h"

#include "Chirp.h"
#include "DebounceIn.h"
#include "Encoder.h"
#include "IMU.h"
#include "Motor.h"
#include "RC.h"
#include "SerialStream.h"
#include "ThreadFlag.h"

using namespace std::chrono;

class MiniSegway
{
public:
    explicit MiniSegway(RC& rc, IMU& imu);
    virtual ~MiniSegway();

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    RC &_rc;
    IMU &_imu;

    DebounceIn _button;    
    bool _do_execute{false};
    bool _do_reset{false};

    void toggleDoExecute();
    void threadTask();
    void sendThreadFlag();
    float vel_cntrl_v2_fcn(const float& set_wheel_speed,
                           const float& b,
                           const float& robot_omega,
                           const Eigen::Matrix2f& Cwheel2robot);
};
#endif /* MINI_SEGWAY_H_ */
