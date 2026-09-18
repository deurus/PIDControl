#include <PIDControl.h>
#include <RelayTest.h>
#include <PIDTuning.h>

const int PIN_PV = A0;
const int PIN_OP = 6;

double SP = 50.0;
double PV = 0.0;
double OP = 35.0;

PIDControl pid(
    &PV, &OP, &SP,
    1.0, 0.4, 0.0,
    PIDType::PI_D,
    PIDAction::REVERSE,
    true
);

RelayTest relay(pid, &PV, &OP);
bool resultPrinted = false;

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
}

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_OP, OUTPUT);

    PV = analogRead(PIN_PV) * 100.0 / 1023.0;

    pid.SetSampleTime(100);
    pid.SetOutputLimits(0.0, 100.0);
    pid.SetManualOutput(35.0);
    pid.SetMode(PIDMode::MAN);

    RelayTestConfig cfg;
    cfg.amplitude = 5.0;
    cfg.hysteresis = 0.5;
    cfg.cycles = 5;
    cfg.useCurrentReference = true;
    cfg.maxTestTimeMinutes = 5.0;
    cfg.outputMin = 20.0;
    cfg.outputMax = 80.0;
    cfg.pvMin = 10.0;
    cfg.pvMax = 90.0;

    relay.Start(cfg);
}

void loop()
{
    PV = analogRead(PIN_PV) * 100.0 / 1023.0;

    relay.Update();
    pid.Compute();

    analogWrite(PIN_OP, (int)(OP * 255.0 / 100.0));

    if (relay.IsFinished() && !resultPrinted)
    {
        const RelayTestResult r = relay.GetResult();

        Serial.println("RelayTest finished");
        Serial.print("Ku = "); Serial.println(r.Ku, 6);
        Serial.print("Tu [s] = "); Serial.println(r.Tu, 6);
        Serial.print("PV amplitude = "); Serial.println(r.amplitudePV, 6);
        Serial.print("Cycles = "); Serial.println(r.completedCycles);

        PrintTuning("Tyreus-Luyben PI", PIDTuning::TyreusLuybenPI(r.Ku, r.Tu));
        PrintTuning("Tyreus-Luyben PID", PIDTuning::TyreusLuybenPID(r.Ku, r.Tu));

        resultPrinted = true;
    }

    if (relay.IsAborted() && !resultPrinted)
    {
        Serial.print("RelayTest aborted. Error = ");
        Serial.println((int)relay.GetError());
        resultPrinted = true;
    }
}
