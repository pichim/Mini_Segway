#ifndef MAHONY_H_
#define MAHONY_H_

#include "eigen/Dense.h"

class Mahony
{
public:
    explicit Mahony();
    explicit Mahony(float kp_x, float kp_y, float kp_z, float ki_x, float ki_y, float ki_z, float Ts);
    virtual ~Mahony();

    void setup(float kp_x, float kp_y, float kp_z, float ki_x, float ki_y, float ki_z, float Ts);
    void setGains(float kp_x, float kp_y, float kp_z, float ki_x, float ki_y, float ki_z);
    void setSamplingTime(float Ts);
    void update(Eigen::Vector3f gyro, Eigen::Vector3f acc);
    void update(Eigen::Vector3f gyro, Eigen::Vector3f acc, Eigen::Vector3f mag);
    Eigen::Quaternionf getOrientationAsQuaternion() const;
    Eigen::Vector3f getOrientationAsRPYAngles() const;
    float getTiltAngle() const;

private:
    Eigen::Vector3f m_kp;
    Eigen::Vector3f m_ki;
    float m_Ts = 1.0f;
    Eigen::Quaternionf m_quat;
    Eigen::Vector3f m_bias;
    Eigen::Vector3f m_rpy;
    float m_tilt = 0.0f;

    void initialise();
    void updateOrientation(Eigen::Vector3f gyro, Eigen::Vector3f e);
    Eigen::Vector3f quat2rpy(Eigen::Quaternionf quat);
    Eigen::Vector3f calcRotationError(Eigen::Vector3f v1, Eigen::Vector3f v2);
};

#endif /* MAHONY_H_ */
