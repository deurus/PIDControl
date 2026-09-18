#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <Arduino.h>

enum class PIDType
{
    PID,
    PI_D,
    I_PD
};

enum class PIDAction
{
    DIRECT,
    REVERSE
};

enum class PIDMode
{
    MAN,
    AUTO
};

class PIDControl
{
public:
    PIDControl(double *input, double *output, double *setpoint,
               double kc, double ki, double kd,
               PIDType type = PIDType::PI_D,
               PIDAction action = PIDAction::REVERSE,
               bool pvTracking = true);

    bool Compute();

    void SetMode(PIDMode mode);
    PIDMode GetMode() const;

    void SetManualOutput(double value);
    double GetManualOutput() const;

    void SetTunings(double kc, double ki, double kd);
    double GetKc() const;
    double GetKi() const;
    double GetKd() const;

    void SetStructure(PIDType type);
    PIDType GetStructure() const;

    void SetDirection(PIDAction action);
    PIDAction GetDirection() const;

    void SetPVTracking(bool enable);
    bool GetPVTracking() const;

    void SetSetpoint(double value);
    double GetOperatorSetpoint() const;

    void SetOutputLimits(double minimum, double maximum);
    double GetOutputMinimum() const;
    double GetOutputMaximum() const;

    void SetSampleTime(unsigned long timeMs);
    unsigned long GetSampleTime() const;

private:
    double *Input;
    double *Output;
    double *Setpoint;

    double Kc;
    double Ki;
    double Kd;

    double integral;
    double previousPV;
    double previousError;

    double manualOutput;
    double operatorSetpoint;

    double minOutput;
    double maxOutput;

    PIDType pidType;
    PIDAction pidAction;
    PIDMode pidMode;

    bool pvTrackingEnabled;

    unsigned long sampleTimeMs;
    unsigned long previousTime;

    double ActionSign() const;
    double Clamp(double value) const;
    void InitializeBumpless();
};

#endif
