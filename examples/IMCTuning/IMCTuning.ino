#include <PIDTuning.h>

void PrintTuning(const char *name, const PIDTuningResult &r)
{
    if (!r.valid)
        return;

    Serial.println(name);
    Serial.print("Kc = "); Serial.println(r.Kc, 6);
    Serial.print("Ti [s] = "); Serial.println(r.Ti, 6);
    Serial.print("Td [s] = "); Serial.println(r.Td, 6);
    Serial.print("Ki [1/s] = "); Serial.println(r.Ki, 6);
    Serial.print("Kd [s] = "); Serial.println(r.Kd, 6);
    Serial.println();
}

void setup()
{
    Serial.begin(115200);

    FOPDTModel model;
    model.valid = true;
    model.Kp = 0.288;
    model.KpNormalizedValid = true;
    model.KpNormalized = 0.288;
    model.T0 = 27.84;
    model.Tp = 166.34;
    model.pvRangeMin = 0.0;
    model.pvRangeMax = 100.0;
    model.outputRangeMin = 0.0;
    model.outputRangeMax = 100.0;

    PIDTuning tuning(model);

    // Default PIDControl preset: lambda/Tf = 2*T0.
    const PIDTuningResult pi = tuning.IMC_PI();
    const PIDTuningResult pid = tuning.Lambda_PID();

    PrintTuning("IMC / Lambda PI", pi);
    PrintTuning("Lambda PID", pid);

    // Explicit tuning speed is also possible.
    const PIDTuningResult robustPid = tuning.Lambda_PID(IMCSpeed::ROBUST);
    (void)robustPid;
}

void loop()
{
}
