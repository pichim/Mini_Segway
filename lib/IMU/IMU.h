#pragma once

#include "mbed.h"
#include <Eigen/Dense>

// #include "config.h"

#include "IIRFilter.h"
#include "Mahony.h"
#include "MPU6500_I2C.h"
#include "config.h"

#ifndef M_PIf
    #define M_PIf 3.14159265358979323846f /* pi */
#endif

class IMU
{
public:
    explicit IMU(PinName pin_sdc, PinName pin_scl);
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
    bool isCalibrated() const { return m_is_calibrated; };

private:
    I2C m_i2c;
    MPU6500_I2C m_ImuMPU6500;
    Mahony m_Mahony;
    ImuData m_ImuData;
    IIRFilter m_gyro_filter[3];
    IIRFilter m_acc_filter[3];

    bool m_is_calibrated{false};
};