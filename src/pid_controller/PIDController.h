#pragma once

#include "IIRFilter.h"

class PIDController{
public:
    PIDController() {};
    virtual ~PIDController() = default;

    void pidControllerInit(const float Kp, const float Ki, const float Kd, const float fcutD, const float fcutRollOff, const float Ts);

    void reset(const float ui);
    float apply(const float error);
    float applyConstrained(const float error, const float uMin, const float uMax);

private:
    struct PIDControllerParams{
        float Kp, Ki, Kd, Ts;
        IIRFilter differentiatingLowPass1;
        IIRFilter lowPass1;
        float uiPrevious;
    } pidController;
};