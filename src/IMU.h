#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"
#include "eigen/Dense.h"

#include "config.h"

#include "Mahony.h"
#include "MPU6500/mpu6500_spi.h"

// if this is false then acc gets averaged at the beginning and printed to the console
#define IMU_DO_USE_STATIC_ACC_CALIBRATION true

namespace Parameters
{
    // % real pole, no integrator, use this if you dont use the mag
    // w0 = 3;
    // kp = w0;
    // ki = 0;
    static const float kp = 5.0f * 2.0f * M_PI;
    static const float ki = 0.0f;

    static const Eigen::Vector3f b_acc = (Eigen::Vector3f() << 0.0000000f, 0.0000000f, 0.0000000f).finished();
}

class IMU
{
public:
    explicit IMU(PinName pin_mosi, PinName pin_miso, PinName pin_clk, PinName pin_cl);
    virtual ~IMU() {};

    class ImuData
    {
    public:
        ImuData() { init(); };
        virtual ~ImuData() {};

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
    void update();

private:
    ImuData m_ImuData;
    mpu6500_spi m_ImuMPU6500;
    SPI m_spi;
    Mahony m_Mahony;
};

#endif /* IMU_H_ */
