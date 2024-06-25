#pragma once

#include "config.h"

#include "eigen/Dense.h"

#if MINI_SEGWAY_CHIRP_USE_CHIRP
    #include "Chirp.h"
#endif
#include "Encoder.h" // TODO: Remove
// #include "DCMotor.h"
#include "DebounceIn.h"
#include "IIRFilter.h"
#include "IMU.h"
#include "Motor.h" // TODO: Remove
#include "PIDController.h"
#include "RC.h"
#include "SerialStream.h"
#include "ThreadFlag.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

using namespace std::chrono;

class MiniSegway
{
public:
    explicit MiniSegway(RC& rc);
    virtual ~MiniSegway();

private:
    Thread _Thread;
    Ticker _Ticker;
    ThreadFlag _ThreadFlag;

    RC &_rc;
    IMU _imu;

    DebounceIn _button;    
    bool _do_execute{false};
    bool _do_reset{false};

    void threadTask();
    void sendThreadFlag();
    void toggleDoExecute();
};