#include "IMU.h"

IMU::IMU(PinName pin_mosi,
         PinName pin_miso,
         PinName pin_clk,
         PinName pin_cl) : m_spi(pin_mosi, pin_miso, pin_clk),
                           m_ImuMPU6500(m_spi, pin_cl),
                           m_Mahony(Parameters::kp, Parameters::ki, static_cast<float>(MINI_SEGWAY_PERIOD_US) * 1.0e-6f)
{
    m_ImuMPU6500.init_inav();
    m_ImuMPU6500.configuration();
    m_ImuMPU6500.testConnection();
}

IMU::ImuData IMU::getImuData() const
{
    return m_ImuData;
}

void IMU::update()
{
    static const uint16_t Navg = 100;
    static uint16_t avg_cntr = 0;
    static bool imu_is_calibrated = false;
    static Eigen::Vector3f gyro_offset = (Eigen::Vector3f() << 0.0f, 0.0f, 0.0f).finished();
    static Eigen::Vector3f acc_offset = (Eigen::Vector3f() << 0.0f, 0.0f, 0.0f).finished();

    // update imu
    m_ImuMPU6500.readGyro();
    m_ImuMPU6500.readAcc();
    Eigen::Vector3f gyro(m_ImuMPU6500.gyroX, m_ImuMPU6500.gyroY, m_ImuMPU6500.gyroZ);
    Eigen::Vector3f acc(m_ImuMPU6500.accX, m_ImuMPU6500.accY, m_ImuMPU6500.accZ);

    if (!imu_is_calibrated) {
        gyro_offset += gyro;
        acc_offset += acc;
        avg_cntr++;
        if (avg_cntr == Navg) {
            imu_is_calibrated = true;
            gyro_offset /= avg_cntr;
            acc_offset /= avg_cntr;
            // we have to keep gravity in acc z direction
            acc_offset(2) = 0.0f;
#if IMU_DO_USE_STATIC_ACC_CALIBRATION
            acc_offset = Parameters::b_acc;
#else
            printf("Averaged acc offset: %.7ff, %.7ff, %.7f\n", acc_offset(0), acc_offset(1), acc_offset(2));
#endif
        }
    }

    if (imu_is_calibrated) {
        gyro -= gyro_offset;
        acc -= acc_offset;

        m_Mahony.update(gyro, acc);

        // update data object
        m_ImuData.gyro = gyro;
        m_ImuData.acc = acc;
        m_ImuData.quat = m_Mahony.getOrientationAsQuaternion();
        m_ImuData.rpy = m_Mahony.getOrientationAsRPYAngles();
        m_ImuData.tilt = m_Mahony.getTiltAngle();
    }
}
