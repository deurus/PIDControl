#include "PIDTuning.h"
#include <math.h>

PIDTuning::PIDTuning(const FOPDTModel &model)
    : processModel(model)
{
}

PIDTuningResult PIDTuning::IMC_PI() const
{
    return IMC_PI(DefaultLambda());
}

PIDTuningResult PIDTuning::IMC_PI(IMCSpeed speed) const
{
    return IMC_PI(LambdaForSpeed(speed));
}

PIDTuningResult PIDTuning::IMC_PI(double lambdaSeconds) const
{
    if (!processModel.valid || fabs(processModel.Kp) < 1.0e-12 ||
        processModel.Tp <= 0.0 || processModel.T0 < 0.0 || lambdaSeconds <= 0.0)
        return InvalidResult(lambdaSeconds);

    const double denominator = fabs(processModel.Kp) * (lambdaSeconds + processModel.T0);
    if (denominator <= 1.0e-12)
        return InvalidResult(lambdaSeconds);

    PIDTuningResult result;
    result.valid = true;
    result.lambda = lambdaSeconds;
    result.Kc = processModel.Tp / denominator;
    result.Ti = processModel.Tp;
    result.Td = 0.0;
    result.Ki = result.Kc / result.Ti;
    result.Kd = 0.0;
    return result;
}

PIDTuningResult PIDTuning::Lambda_PI() const
{
    return IMC_PI();
}

PIDTuningResult PIDTuning::Lambda_PI(double tfSeconds) const
{
    return IMC_PI(tfSeconds);
}

PIDTuningResult PIDTuning::Lambda_PI(IMCSpeed speed) const
{
    return IMC_PI(speed);
}

PIDTuningResult PIDTuning::Lambda_PID() const
{
    return Lambda_PID(DefaultLambda());
}

PIDTuningResult PIDTuning::Lambda_PID(IMCSpeed speed) const
{
    return Lambda_PID(LambdaForSpeed(speed));
}

PIDTuningResult PIDTuning::Lambda_PID(double tfSeconds) const
{
    if (!processModel.valid || fabs(processModel.Kp) < 1.0e-12 ||
        processModel.Tp <= 0.0 || processModel.T0 < 0.0 || tfSeconds <= 0.0)
        return InvalidResult(tfSeconds);

    const double halfT0 = 0.5 * processModel.T0;
    const double denominator = fabs(processModel.Kp) * (tfSeconds + halfT0);
    const double ti = processModel.Tp + halfT0;
    const double tdDenominator = 2.0 * processModel.Tp + processModel.T0;

    if (denominator <= 1.0e-12 || ti <= 1.0e-12 || tdDenominator <= 1.0e-12)
        return InvalidResult(tfSeconds);

    PIDTuningResult result;
    result.valid = true;
    result.lambda = tfSeconds;
    result.Kc = (processModel.Tp + halfT0) / denominator;
    result.Ti = ti;
    result.Td = (processModel.Tp * processModel.T0) / tdDenominator;
    result.Ki = result.Kc / result.Ti;
    result.Kd = result.Kc * result.Td;
    return result;
}

PIDTuningResult PIDTuning::TyreusLuybenPI(double Ku, double Tu)
{
    if (Ku <= 0.0 || Tu <= 0.0)
        return InvalidResult();

    PIDTuningResult result;
    result.valid = true;
    result.lambda = 0.0;
    result.Kc = Ku / 3.2;
    result.Ti = 2.2 * Tu;
    result.Td = 0.0;
    result.Ki = result.Kc / result.Ti;
    result.Kd = 0.0;
    return result;
}

PIDTuningResult PIDTuning::TyreusLuybenPID(double Ku, double Tu)
{
    if (Ku <= 0.0 || Tu <= 0.0)
        return InvalidResult();

    PIDTuningResult result;
    result.valid = true;
    result.lambda = 0.0;
    result.Kc = Ku / 3.3;
    result.Ti = 2.2 * Tu;
    result.Td = Tu / 6.3;
    result.Ki = result.Kc / result.Ti;
    result.Kd = result.Kc * result.Td;
    return result;
}

double PIDTuning::DefaultLambda() const
{
    // PIDControl default preset: lambda/Tf = 2*T0.
    // If the identified dead time is numerically zero, use a conservative
    // fallback so the convenience overloads remain usable.
    if (processModel.T0 > 1.0e-9)
        return 2.0 * processModel.T0;

    return (processModel.Tp > 0.0) ? 0.1 * processModel.Tp : 0.0;
}

const FOPDTModel &PIDTuning::GetModel() const
{
    return processModel;
}

bool PIDTuning::Apply(PIDControl &controller, const PIDTuningResult &result)
{
    if (!result.valid)
        return false;

    controller.SetTunings(result.Kc, result.Ki, result.Kd);
    return true;
}

double PIDTuning::LambdaForSpeed(IMCSpeed speed) const
{
    if (processModel.T0 <= 1.0e-9)
        return DefaultLambda();

    switch (speed)
    {
        case IMCSpeed::AGGRESSIVE: return processModel.T0;
        case IMCSpeed::ROBUST:     return 3.0 * processModel.T0;
        case IMCSpeed::NORMAL:
        default:                   return 2.0 * processModel.T0;
    }
}

PIDTuningResult PIDTuning::InvalidResult(double lambdaSeconds)
{
    PIDTuningResult result;
    result.valid = false;
    result.Kc = 0.0;
    result.Ti = 0.0;
    result.Td = 0.0;
    result.Ki = 0.0;
    result.Kd = 0.0;
    result.lambda = lambdaSeconds;
    return result;
}
