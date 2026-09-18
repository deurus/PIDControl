#include <PIDControl.h>

const int PIN_PV = A0;
const int PIN_OP = 6;
const unsigned long Ts_ms = 100;

double SP = 50.0;
double PV = 0.0;
double OP = 0.0;

double Kc = 1.0;
double Ki = 0.4;
double Kd = 0.0;

PIDControl pid(
    &PV,
    &OP,
    &SP,
    Kc,
    Ki,
    Kd,
    PIDType::PI_D,
    PIDAction::REVERSE,
    true
);

void setup()
{
    pinMode(PIN_OP, OUTPUT);

    pid.SetSampleTime(Ts_ms);
    pid.SetOutputLimits(0.0, 100.0);
    pid.SetManualOutput(35.0);
    pid.SetMode(PIDMode::MAN);
}

void loop()
{
    PV = analogRead(PIN_PV) * 100.0 / 1023.0;

    pid.Compute();

    analogWrite(PIN_OP, (int)(OP * 255.0 / 100.0));
}
