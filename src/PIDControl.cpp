#include "PIDControl.h"

PIDControl::PIDControl(double *input, double *output, double *setpoint,
                       double kc, double ki, double kd,
                       PIDType type, PIDAction action, bool pvTracking)
    : Input(input), Output(output), Setpoint(setpoint),
      Kc(kc), Ki(ki), Kd(kd),
      integral(0.0), previousPV(*input), previousError(*setpoint - *input),
      manualOutput(*output), operatorSetpoint(*setpoint),
      minOutput(0.0), maxOutput(100.0),
      pidType(type), pidAction(action), pidMode(PIDMode::MAN),
      pvTrackingEnabled(pvTracking),
      sampleTimeMs(100), previousTime(0)
{
    if (Kc < 0.0) Kc = 0.0;
    if (Ki < 0.0) Ki = 0.0;
    if (Kd < 0.0) Kd = 0.0;

    if (pvTrackingEnabled)
        *Setpoint = *Input;
}

bool PIDControl::Compute()
{
    if (pidMode == PIDMode::MAN)
    {
        *Output = Clamp(manualOutput);

        if (pvTrackingEnabled)
            *Setpoint = *Input;

        return false;
    }

    const unsigned long now = millis();
    const unsigned long elapsedMs = now - previousTime;

    if (elapsedMs < sampleTimeMs)
        return false;

    const double Ts = elapsedMs / 1000.0;
    previousTime = now;

    const double error = *Setpoint - *Input;
    const double sign = ActionSign();

    const double P = (pidType == PIDType::I_PD)
        ? -sign * Kc * (*Input)
        :  sign * Kc * error;

    const double D = (pidType == PIDType::PID)
        ?  sign * Kd * (error - previousError) / Ts
        : -sign * Kd * ((*Input) - previousPV) / Ts;

    const double integralIncrement = sign * Ki * error * Ts;
    const double newIntegral = integral + integralIncrement;
    const double unsaturatedOutput = P + newIntegral + D;

    const bool saturatingHigh =
        (unsaturatedOutput > maxOutput && integralIncrement > 0.0);

    const bool saturatingLow =
        (unsaturatedOutput < minOutput && integralIncrement < 0.0);

    if (!(saturatingHigh || saturatingLow))
        integral = newIntegral;

    *Output = Clamp(P + integral + D);

    previousPV = *Input;
    previousError = error;

    return true;
}

void PIDControl::SetMode(PIDMode mode)
{
    if (mode == pidMode)
        return;

    if (mode == PIDMode::MAN)
    {
        manualOutput = Clamp(*Output);
        operatorSetpoint = *Setpoint;
        pidMode = PIDMode::MAN;

        if (pvTrackingEnabled)
            *Setpoint = *Input;

        return;
    }

    *Setpoint = pvTrackingEnabled ? *Input : operatorSetpoint;

    InitializeBumpless();
    pidMode = PIDMode::AUTO;
    previousTime = millis();
}

PIDMode PIDControl::GetMode() const
{
    return pidMode;
}

void PIDControl::SetManualOutput(double value)
{
    manualOutput = Clamp(value);

    if (pidMode == PIDMode::MAN)
        *Output = manualOutput;
}

double PIDControl::GetManualOutput() const
{
    return manualOutput;
}

void PIDControl::SetTunings(double kc, double ki, double kd)
{
    if (kc < 0.0 || ki < 0.0 || kd < 0.0)
        return;

    Kc = kc;
    Ki = ki;
    Kd = kd;

    if (pidMode == PIDMode::AUTO)
        InitializeBumpless();
}

double PIDControl::GetKc() const
{
    return Kc;
}

double PIDControl::GetKi() const
{
    return Ki;
}

double PIDControl::GetKd() const
{
    return Kd;
}

void PIDControl::SetStructure(PIDType type)
{
    if (type == pidType)
        return;

    pidType = type;

    if (pidMode == PIDMode::AUTO)
        InitializeBumpless();
}

PIDType PIDControl::GetStructure() const
{
    return pidType;
}

void PIDControl::SetDirection(PIDAction action)
{
    if (action == pidAction)
        return;

    pidAction = action;

    if (pidMode == PIDMode::AUTO)
        InitializeBumpless();
}

PIDAction PIDControl::GetDirection() const
{
    return pidAction;
}

void PIDControl::SetPVTracking(bool enable)
{
    if (enable == pvTrackingEnabled)
        return;

    if (enable)
    {
        operatorSetpoint = *Setpoint;
        pvTrackingEnabled = true;

        if (pidMode == PIDMode::MAN)
            *Setpoint = *Input;
    }
    else
    {
        pvTrackingEnabled = false;

        if (pidMode == PIDMode::MAN)
            *Setpoint = operatorSetpoint;
    }
}

bool PIDControl::GetPVTracking() const
{
    return pvTrackingEnabled;
}

void PIDControl::SetSetpoint(double value)
{
    operatorSetpoint = value;

    if (pidMode == PIDMode::AUTO || !pvTrackingEnabled)
        *Setpoint = value;
}

double PIDControl::GetOperatorSetpoint() const
{
    return operatorSetpoint;
}

void PIDControl::SetOutputLimits(double minimum, double maximum)
{
    if (minimum >= maximum)
        return;

    minOutput = minimum;
    maxOutput = maximum;

    manualOutput = Clamp(manualOutput);
    *Output = Clamp(*Output);

    if (pidMode == PIDMode::AUTO)
        InitializeBumpless();
}

double PIDControl::GetOutputMinimum() const
{
    return minOutput;
}

double PIDControl::GetOutputMaximum() const
{
    return maxOutput;
}

void PIDControl::SetSampleTime(unsigned long timeMs)
{
    if (timeMs > 0)
        sampleTimeMs = timeMs;
}

unsigned long PIDControl::GetSampleTime() const
{
    return sampleTimeMs;
}

double PIDControl::ActionSign() const
{
    return (pidAction == PIDAction::REVERSE) ? 1.0 : -1.0;
}

double PIDControl::Clamp(double value) const
{
    if (value > maxOutput) return maxOutput;
    if (value < minOutput) return minOutput;
    return value;
}

void PIDControl::InitializeBumpless()
{
    const double error = *Setpoint - *Input;
    const double sign = ActionSign();

    const double P = (pidType == PIDType::I_PD)
        ? -sign * Kc * (*Input)
        :  sign * Kc * error;

    previousPV = *Input;
    previousError = error;

    // Derivative contribution is zero immediately after initialization.
    // Adjust the integral term so the controller output starts from the
    // current output value, minimizing bumps when changing mode or settings.
    integral = *Output - P;
}
