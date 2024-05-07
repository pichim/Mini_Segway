#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"
#include "eigen/Dense.h"

#include "config.h"

#include "IIR_Filter.h"
#include "Mahony.h"
#include "MPU6500/mpu6500_spi.h"

// if this is false then acc gets averaged at the beginning and printed to the console
#define IMU_DO_USE_STATIC_ACC_CALIBRATION true

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

    ImuData update();
    IIR_Filter m_gyro_filter[3];
    IIR_Filter m_acc_filter[3];

private:
    ImuData m_ImuData;
    mpu6500_spi m_ImuMPU6500;
    SPI m_spi;
    Mahony m_Mahony;
};

#endif /* IMU_H_ */
