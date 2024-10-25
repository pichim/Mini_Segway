#include "IMU.h"

IMU::IMU(PinName pin_mosi,
         PinName pin_miso,
         PinName pin_clk,
         PinName pin_cl) : m_spi(pin_mosi, pin_miso, pin_clk)
                         , m_ImuMPU6500(m_spi, pin_cl)
                         , m_Mahony(MINI_SEGWAY_IMU_KP_XY, MINI_SEGWAY_IMU_KP_XY, MINI_SEGWAY_IMU_KP_Z,
                                    MINI_SEGWAY_IMU_KI_XY, MINI_SEGWAY_IMU_KI_XY, MINI_SEGWAY_IMU_KI_Z,
                                    MINI_SEGWAY_TS)
                         
{
    m_gyro_filter[0].lowPass1Init(MINI_SEGWAY_IMU_GYRO_FILTER_FREQUENCY_HZ, MINI_SEGWAY_TS);
    m_gyro_filter[1].lowPass1Init(MINI_SEGWAY_IMU_GYRO_FILTER_FREQUENCY_HZ, MINI_SEGWAY_TS);
    m_gyro_filter[2].lowPass1Init(MINI_SEGWAY_IMU_GYRO_FILTER_FREQUENCY_HZ, MINI_SEGWAY_TS);

    m_acc_filter[0].lowPass1Init(MINI_SEGWAY_IMU_ACC_FILTER_FREQUENCY_HZ, MINI_SEGWAY_TS);
    m_acc_filter[1].lowPass1Init(MINI_SEGWAY_IMU_ACC_FILTER_FREQUENCY_HZ, MINI_SEGWAY_TS);
    m_acc_filter[2].lowPass1Init(MINI_SEGWAY_IMU_ACC_FILTER_FREQUENCY_HZ, MINI_SEGWAY_TS);

    m_ImuMPU6500.init();
    m_ImuMPU6500.configuration();
    m_ImuMPU6500.testConnection();
}

IMU::ImuData IMU::update()
{
    static const uint16_t Nskip = MINI_SEGWAY_IMU_NUM_RUNS_SKIP;
    static uint16_t skip_cntr = 0;
    static const uint16_t Navg = MINI_SEGWAY_IMU_NUM_RUNS_FOR_AVERAGE;
    static uint16_t avg_cntr = 0;
    static Eigen::Vector3f gyro_offset = (Eigen::Vector3f() << 0.0f, 0.0f, 0.0f).finished();
    static Eigen::Vector3f acc_offset = (Eigen::Vector3f() << 0.0f, 0.0f, 0.0f).finished();

    // update imu
    m_ImuMPU6500.readGyro();
    m_ImuMPU6500.readAcc();

    // skip first Nskip runs
    if (skip_cntr < Nskip) {
        skip_cntr++;
        return m_ImuData;
    }

    // segway imu alignment:
    // the alignment was chosen so that roll can be used for controlling
    // the segway since yaw has a singularity at +/-90 deg
    //   when standing upright, the IMU is mounted on the robot with:
    //   - the x-axis pointing to the right
    //   - the y-axis pointing forwards
    //   - the z-axis pointing upwards
    Eigen::Vector3f gyro(m_ImuMPU6500.gyroX, -m_ImuMPU6500.gyroZ, m_ImuMPU6500.gyroY);
    Eigen::Vector3f acc(m_ImuMPU6500.accX, -m_ImuMPU6500.accZ, m_ImuMPU6500.accY);

    if (!m_is_calibrated) {
        avg_cntr++;
        // sum up gyro and acc
        gyro_offset += gyro;
        acc_offset += acc;
        // calculate average
        if (avg_cntr == Navg) {
            m_is_calibrated = true;

            gyro_offset /= avg_cntr;
            acc_offset /= avg_cntr;

            printf("Avg. Gyr offset: %.4f, %.4f, %.4f; ...\n", gyro_offset(0), gyro_offset(1), gyro_offset(2));
            printf("Avg. Acc offset: %.4f, %.4f, %.4f; ...\n", acc_offset(0), acc_offset(1), acc_offset(2));

            // set initial rotation
            // currently only 3 initial orientations are supported:
            // - system is hanging
            // - system is lying and segway forward is facing upwards
            // - system is lying and segway forward is facing downwards
            // system is hanging
            if ( fabs(acc_offset(2)) > fabs(acc_offset(1))) {
                m_Mahony.setInitialOrientation(0.0f, -1.0f, 0.0f, 0.0f); // system is hanging
                // keep gravity
                acc_offset(2) = 0.0f;
            // system is lying
            } else {
                const float reci_sqrt_two = 1.0f / sqrtf(2.0f);
                if (acc_offset(1) < 0.0f) {
                    // segway forward facing upwards
                    m_Mahony.setInitialOrientation(reci_sqrt_two, -reci_sqrt_two, 0.0f, 0.0f); // -90 deg roll
                } else {
                    // segway forward facing downwards
                    m_Mahony.setInitialOrientation(reci_sqrt_two, reci_sqrt_two, 0.0f, 0.0f); // 90 deg roll
                }
                // keep gravity
                acc_offset(1) = 0.0f;
            }

#if MINI_SEGWAY_IMU_DO_USE_STATIC_ACC_CALIBRATION
            acc_offset = MINI_SEGWAY_IMU_B_ACC;
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
            gyro(i) = m_gyro_filter[i].apply(gyro(i));
            acc(i) = m_acc_filter[i].apply(acc(i));
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
