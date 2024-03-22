#ifndef LINEAR_CHARACTERISTICS3_H_
#define LINEAR_CHARACTERISTICS3_H_

#include "eigen/Dense.h"

class LinearCharacteristics3
{
public:
    LinearCharacteristics3();
    virtual ~LinearCharacteristics3();

    void setCalibrationParameter(Eigen::Matrix3f A, Eigen::Vector3f b);
    void setLimits(float &lowerLimit, float &upperLimit);
    Eigen::Vector3f applyCalibration(Eigen::Vector3f &x);

private:
    Eigen::Matrix3f m_A;
    Eigen::Vector3f m_b;
    float m_lowerLimit = -1.0e6f;
    float m_upperLimit = 1.0e6f;
};

#endif /* LINEAR_CHARACTERISTICS3_H_ */