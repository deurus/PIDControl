#include <PIDControl.h>
#include <StepTest.h>

// Hardware mapping stays outside StepTest. This keeps the identification
// algorithm reusable with ADC/PWM, I2C, SPI, Modbus, simulators, etc.
const uint8_t PIN_PV = A0;
const uint8_t PIN_OP = 6;

double SP = 50.0;
double PV = 0.0;
double OP = 0.0;

PIDControl pid(&PV, &OP, &SP, 1.0, 0.1, 0.0,
               PIDType::PI_D, PIDAction::REVERSE, true);
StepTest test(pid, &PV, &OP);

void setup()
{
    pinMode(PIN_OP, OUTPUT);

    pid.SetOutputLimits(0.0, 100.0);
    pid.SetMode(PIDMode::MAN);
    pid.SetManualOutput(20.0);

    // Full engineering ranges used only for normalized Kp.
    test.SetPVRange(0.0, 100.0);
    test.SetOutputRange(0.0, 100.0);

    // Test safety limits can be narrower than the ranges.
    test.SetPVLimits(10.0, 90.0);
    test.SetOutputLimits(0.0, 60.0);
    test.SetStepSize(10.0);
    test.SetNumberOfSteps(2);
    test.SetProcessGain(ProcessGain::POSITIVE);
    test.SetStabilityCriteria(0.3, 60.0, 0.1);
    test.SetMaxTestTime(30.0);

    test.Start();
}

void loop()
{
    // Example scaling. Replace it with the real sensor conversion.
    PV = analogRead(PIN_PV) * (100.0 / 1023.0);

    test.Update();
    pid.Compute();

    // Example 0..100 % to 0..255 PWM conversion.
    analogWrite(PIN_OP, (int)(OP * 255.0 / 100.0));
}
