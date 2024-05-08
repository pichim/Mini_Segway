#include "IMU.h"

IMU::IMU(PinName pin_mosi,
         PinName pin_miso,
         PinName pin_clk,
         PinName pin_cl) : m_spi(pin_mosi, pin_miso, pin_clk)
                         , m_ImuMPU6500(m_spi, pin_cl)
                         , m_Mahony(MINI_SEGWAY_IMU_KP_XY, MINI_SEGWAY_IMU_KP_XY, MINI_SEGWAY_IMU_KP_Z,
                                    MINI_SEGWAY_IMU_KI_XY, MINI_SEGWAY_IMU_KI_XY, MINI_SEGWAY_IMU_KI_Z,
                                    MINI_SEGWAY_TS)
                         , m_gyro_filter{IIR_Filter(1.0f / (2.0f * M_PI * MINI_SEGWAY_IMU_GYRO_FREQUENCY_HZ), MINI_SEGWAY_TS, 1.0f),
                                         IIR_Filter(1.0f / (2.0f * M_PI * MINI_SEGWAY_IMU_GYRO_FREQUENCY_HZ), MINI_SEGWAY_TS, 1.0f),
                                         IIR_Filter(1.0f / (2.0f * M_PI * MINI_SEGWAY_IMU_GYRO_FREQUENCY_HZ), MINI_SEGWAY_TS, 1.0f)}
                         , m_acc_filter{IIR_Filter(1.0f / (2.0f * M_PI * MINI_SEGWAY_IMU_ACC_FREQUENCY_HZ), MINI_SEGWAY_TS, 1.0f),
                                        IIR_Filter(1.0f / (2.0f * M_PI * MINI_SEGWAY_IMU_ACC_FREQUENCY_HZ), MINI_SEGWAY_TS, 1.0f),
                                        IIR_Filter(1.0f / (2.0f * M_PI * MINI_SEGWAY_IMU_ACC_FREQUENCY_HZ), MINI_SEGWAY_TS, 1.0f)}
{
    m_ImuMPU6500.init();
    m_ImuMPU6500.configuration();
    m_ImuMPU6500.testConnection();
}

IMU::ImuData IMU::update()
{
    static const uint16_t Navg = 200;
    static uint16_t avg_cntr = 0;
    static bool imu_is_calibrated = false;
    static Eigen::Vector3f gyro_offset = (Eigen::Vector3f() << 0.0f, 0.0f, 0.0f).finished();
    static Eigen::Vector3f acc_offset = (Eigen::Vector3f() << 0.0f, 0.0f, 0.0f).finished();

    // update imu
    m_ImuMPU6500.readGyro();
    m_ImuMPU6500.readAcc();
    Eigen::Vector3f gyro(m_ImuMPU6500.gyroZ, m_ImuMPU6500.gyroX, m_ImuMPU6500.gyroY);
    Eigen::Vector3f acc(m_ImuMPU6500.accZ, m_ImuMPU6500.accX, m_ImuMPU6500.accY);

    if (!imu_is_calibrated) {
        avg_cntr++;
        // sum up gyro and acc
        gyro_offset += gyro;
        acc_offset += acc;
        // calculate average
        if (avg_cntr == Navg) {
            imu_is_calibrated = true;
            gyro_offset /= avg_cntr;
            acc_offset /= avg_cntr;
            // we have to keep gravity in acc z direction
            acc_offset(2) = 0.0f;
#if IMU_DO_USE_STATIC_ACC_CALIBRATION
            // TODO: Test if this gives the correct results
            acc_offset = MINI_SEGWAY_IMU_B_ACC;
#else
            printf("Averaged acc offset: %.7ff, %.7ff, %.7f\n", acc_offset(0), acc_offset(1), acc_offset(2));
#endif
        }
    } else {
        // remove static bias
        gyro -= gyro_offset;
        acc -= acc_offset;

#if MINI_SEGWAY_IMU_USE_ADDITIONAL_FILTERS
        static bool is_first_run = true;
        if (is_first_run) {
            is_first_run = false;
            for (uint8_t i = 0; i < 3; i++) {
                m_gyro_filter[i].reset(gyro(i));
                m_acc_filter[i].reset(acc(i));
            }
        }
        // filter gyro and acc data
        for (uint8_t i = 0; i < 3; i++) {
            gyro(i) = m_gyro_filter[i].filter(gyro(i));
            acc(i) = m_acc_filter[i].filter(acc(i));
        }
#endif

        // update mahony
        m_Mahony.update(gyro, acc);

        // update data object
        m_ImuData.gyro = gyro;
        m_ImuData.acc = acc;
        m_ImuData.quat = m_Mahony.getOrientationAsQuaternion();
        m_ImuData.rpy = m_Mahony.getOrientationAsRPYAngles();
        m_ImuData.tilt = m_Mahony.getTiltAngle();
    }

    return m_ImuData;
}
