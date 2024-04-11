#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"
#include "eigen/Dense.h"

#include "SPI.h"
#include "MPU6500/mpu6500_spi.h"
#include "LinearCharacteristics3.h"
#include "Mahony.h"
#include "ThreadFlag.h"

#define IMU_DO_PRINTF false
#define IMU_DO_USE_STATIC_ACC_CALIBRATION true  // if this is false then acc gets averaged at the beginning and printed to the console

namespace Parameters
{
    // % real pole, no integrator, use this if you dont use the mag
    // w0 = 3;
    // kp = w0;
    // ki = 0;
    static const float kp = 5.0f * 2.0f * M_PI; // PES Board: kp = 3.0f;
    static const float ki = 0.0f;

    static const Eigen::Vector3f b_acc = (Eigen::Vector3f() << 0.0000000f, 0.0000000f, 0.0000000f).finished();
}
using namespace std::chrono;
class IMU
{
public:
    explicit IMU(PinName pin_mosi, PinName pin_miso, PinName pin_clk, PinName pin_cl);
    virtual ~IMU();

    class ImuData
    {
    public:
        ImuData() {
            init();
        };
        virtual ~ImuData(){};

        Eigen::Vector3f gyro, acc;
        Eigen::Quaternionf quat;
        Eigen::Vector3f rpy;
        float tilt = 0.0f;

        void init() {
            gyro.setZero();
            acc.setZero();
            quat.setIdentity();
            rpy.setZero();
        };
    };

    ImuData getImuData() const;
    uint32_t getPeriod() const { return m_period; }

private:
    static constexpr int64_t PERIOD_MUS = 2000; // PES Board: PERIOD_MUS = 20000
    static constexpr float TS = 1.0e-6f * static_cast<float>(PERIOD_MUS);

    ImuData m_ImuData;
    mpu6500_spi m_ImuMPU6500;
    SPI m_spi;
    LinearCharacteristics3 m_magCalib;
    Mahony m_Mahony;

    Thread m_Thread;
    Ticker m_Ticker;
    ThreadFlag m_ThreadFlag;

    Timer m_Timer;
    uint32_t m_period{0};

    void threadTask();
    void sendThreadFlag();
};

#endif /* IMU_H_ */
