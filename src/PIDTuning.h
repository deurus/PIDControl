#ifndef PIDTUNING_H
#define PIDTUNING_H

#include <Arduino.h>
#include "StepTest.h"
#include "PIDControl.h"

enum class IMCSpeed
{
    AGGRESSIVE,
    NORMAL,
    ROBUST
};

struct PIDTuningResult
{
    bool valid;
    double Kc;
    double Ti;
    double Td;
    double Ki;
    double Kd;

    // Closed-loop tuning parameter. For IMC/Lambda PI this is lambda.
    // For Lambda PID this stores Tf (same role: desired closed-loop speed).
    double lambda;
};

class PIDTuning
{
public:
    explicit PIDTuning(const FOPDTModel &model);

    // IMC/Lambda PI. Default: lambda = 2*T0 (NORMAL preset).
    PIDTuningResult IMC_PI() const;
    PIDTuningResult IMC_PI(double lambdaSeconds) const;
    PIDTuningResult IMC_PI(IMCSpeed speed) const;

    // Explicit Lambda PI aliases kept alongside IMC_PI for terminology.
    PIDTuningResult Lambda_PI() const;
    PIDTuningResult Lambda_PI(double tfSeconds) const;
    PIDTuningResult Lambda_PI(IMCSpeed speed) const;

    // Rivera/Lambda PID from the same FOPDT model.
    // Default Tf follows the same PIDControl presets as lambda.
    PIDTuningResult Lambda_PID() const;
    PIDTuningResult Lambda_PID(double tfSeconds) const;
    PIDTuningResult Lambda_PID(IMCSpeed speed) const;

    // Tyreus-Luyben tuning from ultimate gain/period obtained by RelayTest.
    static PIDTuningResult TyreusLuybenPI(double Ku, double Tu);
    static PIDTuningResult TyreusLuybenPID(double Ku, double Tu);

    double DefaultLambda() const;
    const FOPDTModel &GetModel() const;

    // Convenience helper. It does not change PID structure, mode or action.
    // It only applies Kc/Ki/Kd from a valid result.
    static bool Apply(PIDControl &controller, const PIDTuningResult &result);

private:
    FOPDTModel processModel;

    double LambdaForSpeed(IMCSpeed speed) const;
    static PIDTuningResult InvalidResult(double lambdaSeconds = 0.0);
};

#endif
