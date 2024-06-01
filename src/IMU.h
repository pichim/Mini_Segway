#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"
#include "eigen/Dense.h"

#include "config.h"

#include "IIR_Filter.h"
#include "Mahony.h"
#include "MPU6500/mpu6500_spi.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

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
    SPI m_spi;
    mpu6500_spi m_ImuMPU6500;
    Mahony m_Mahony;
    ImuData m_ImuData;
};

#endif /* IMU_H_ */
