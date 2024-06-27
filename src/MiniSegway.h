#pragma once

#include "config.h"

#include "eigen/Dense.h"

#if MINI_SEGWAY_CHIRP_USE_CHIRP
    #include "Chirp.h"
#endif
#include "Encoder.h"
#include "DebounceIn.h"
#include "IIRFilter.h"
#include "IMU.h"
#include "Led.h"
#include "Motor.h"
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
    DebounceIn _additional_button;
    bool _do_execute{false};

    void threadTask();
    void sendThreadFlag();
    void toggleDoExecute();
};