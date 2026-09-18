#include <PIDControl.h>

const int PIN_PV = A0;
const int PIN_OP = 6;

double SP = 50.0;
double PV = 0.0;
double OP = 0.0;

PIDControl pid(
    &PV,
    &OP,
    &SP,
    1.0,
    0.4,
    0.0,
    PIDType::PI_D,
    PIDAction::REVERSE,
    true
);

void setup()
{
    pinMode(PIN_OP, OUTPUT);

    pid.SetSampleTime(100);
    pid.SetOutputLimits(0.0, 100.0);
    pid.SetManualOutput(35.0);
    pid.SetMode(PIDMode::MAN);
}

void loop()
{
    PV = analogRead(PIN_PV) * 100.0 / 1023.0;

    // Example: switch from MAN to AUTO after 10 seconds.
    if (millis() >= 10000UL && pid.GetMode() == PIDMode::MAN)
    {
        pid.SetMode(PIDMode::AUTO);

        // With PV tracking enabled, AUTO starts with SP = PV.
        // The operator setpoint can then be applied explicitly.
        pid.SetSetpoint(50.0);
    }

    pid.Compute();

    analogWrite(PIN_OP, (int)(OP * 255.0 / 100.0));
}
