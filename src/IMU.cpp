#include "IMU.h"

IMU::IMU(PinName pin_mosi, PinName pin_miso, PinName pin_clk, PinName pin_cl) : m_spi(pin_mosi, pin_miso, pin_clk),
                                                                                m_ImuMPU6500(m_spi, pin_cl),
                                                                                m_Mahony(Parameters::kp, Parameters::ki, TS),
                                                                                m_Thread(osPriorityAboveNormal, 4096) // PES Board: m_Thread(osPriorityHigh, 4096)
{
    m_ImuMPU6500.init_inav();
    m_ImuMPU6500.configuration();
    m_ImuMPU6500.testConnection();

    m_Timer.start();
    
    // start thread
    m_Thread.start(callback(this, &IMU::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    m_Ticker.attach(callback(this, &IMU::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

IMU::~IMU()
{
    m_Ticker.detach();
    m_Thread.terminate();
}

IMU::ImuData IMU::getImuData() const
{
    return m_ImuData;
}

void IMU::threadTask()
{
    static const uint16_t Navg = 100;
    static uint16_t avg_cntr = 0;
    static bool imu_is_calibrated = false;
    static Eigen::Vector3f gyro_offset;
    static Eigen::Vector3f acc_offset;
    gyro_offset.setZero();
    acc_offset.setZero();
    static microseconds time_previous{0};

    while (true) {
        ThisThread::flags_wait_any(m_ThreadFlag);

        //m_ImuLSM9DS1.updateGyro();
        //m_ImuLSM9DS1.updateAcc();
        Eigen::Vector3f gyro(m_ImuMPU6500.readGyro(0), m_ImuMPU6500.readGyro(1), m_ImuMPU6500.readGyro(2));
        Eigen::Vector3f acc(m_ImuMPU6500.readAcc(0), m_ImuMPU6500.readAcc(1), m_ImuMPU6500.readAcc(2));

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

            // meassure signal period and update sampling time in mahony filter
            const microseconds time = m_Timer.elapsed_time();
            m_period = duration_cast<microseconds>(time - time_previous).count();
            time_previous = time;
            m_Mahony.setSamplingTime(static_cast<float>(m_period * 1.0e-6f));
            m_Mahony.update(gyro, acc);

            // update data object
            m_ImuData.gyro = gyro;
            m_ImuData.acc = acc;
            m_ImuData.quat = m_Mahony.getOrientationAsQuaternion();
            m_ImuData.rpy = m_Mahony.getOrientationAsRPYAngles();
            m_ImuData.tilt = m_Mahony.getTiltAngle();
        }


#if IMU_DO_PRINTF
        static float time_ms_past = 0.0f;
        float time_ms = std::chrono::duration_cast<std::chrono::microseconds>(m_Timer.elapsed_time()).count() * 1.0e-3f;
        const float dtime_ms = time_ms - time_ms_past;
        time_ms_past = time_ms;
        printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, ", m_ImuData.gyro(0), m_ImuData.gyro(1), m_ImuData.gyro(2),
                                                             m_ImuData.acc(0), m_ImuData.acc(1), m_ImuData.acc(2), time_ms);
        printf("%.6f, %.6f, %.6f, %.6f, ", m_ImuData.quat.w(), m_ImuData.quat.x(), m_ImuData.quat.y(), m_ImuData.quat.z());
        printf("%.6f, %.6f, %.6f, ", m_ImuData.rpy(0), m_ImuData.rpy(1), m_ImuData.rpy(2));
        printf("%.6f\n", m_ImuData.tilt);
#endif
    }
}

void IMU::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    m_Thread.flags_set(m_ThreadFlag);
}