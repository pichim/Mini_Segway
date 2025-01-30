#pragma once

#include "IIRFilter.h"

class PIDController{
public:
    PIDController() {};
    virtual ~PIDController() = default;

    void pidt2ControllerInit(const float Kp, const float Ki, const float Kd, const float fcutD, const float fcutRollOff, const float Ts);
    void piControllerInit(const float Kp, const float Ki, const float Ts);
    void pdt1ControllerInit(const float Kp, const float Kd, const float fcutD, const float Ts);
    void pdt2ControllerInit(const float Kp, const float Kd, const float fcutD, const float fcutRollOff, const float Ts);

    void reset(const float ui);
    float apply(const float error);
    float applyConstrained(const float error, const float uMin, const float uMax);

private:
    struct PIDControllerParams{
        float Kp, Ki, Kd, fcutD, fcutRollOff, Ts;
        IIRFilter differentiatingLowPass1;
        IIRFilter lowPass1;
        float uiPrevious;
    } pidController;
};