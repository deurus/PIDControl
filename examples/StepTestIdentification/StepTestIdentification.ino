#include <PIDControl.h>
#include <StepTest.h>
#include <PIDTuning.h>

const int PIN_PV = A0;
const int PIN_OP = 6;

// Process variables / Variables de proceso
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

StepTest test(pid, &PV, &OP);

bool resultPrinted = false;

void setup()
{
    Serial.begin(115200);
    pinMode(PIN_OP, OUTPUT);

    PV = analogRead(PIN_PV) * 100.0 / 1023.0;

    pid.SetSampleTime(100);
    pid.SetOutputLimits(0.0, 100.0);
    pid.SetManualOutput(35.0);
    pid.SetMode(PIDMode::MAN);

    // Safety and test configuration / Configuracion de seguridad y ensayo
    test.SetOutputLimits(20.0, 80.0);
    test.SetPVLimits(10.0, 90.0);
    test.SetOutputRange(0.0, 100.0);
    test.SetPVRange(0.0, 100.0);
    test.SetStepSize(5.0);
    test.SetNumberOfSteps(4);
    test.SetFirstStepUp(true);
    test.SetProcessGain(ProcessGain::POSITIVE);

    // Stability = PV half-band, observation time [s], max slope [units/min].
    // Estabilidad = semibanda PV, tiempo de observacion [s], pendiente max. [unidades/min].
    test.SetStabilityCriteria(0.3, 60.0, 0.10);
    test.SetResponseThreshold(0.4);

    // Minutes. Internally converted to milliseconds.
    // Minutos. Se convierten internamente a milisegundos.
    test.SetMaxTestTime(30.0);

    test.Start();
}

void loop()
{
    PV = analogRead(PIN_PV) * 100.0 / 1023.0;

    test.Update();
    pid.Compute();

    analogWrite(PIN_OP, (int)(OP * 255.0 / 100.0));

    if (test.IsFinished() && !resultPrinted)
    {
        StepTestResult r = test.GetResult();

        Serial.println("StepTest finished / StepTest finalizado");
        Serial.print("Kp = "); Serial.println(r.Kp, 6);
        if (r.KpNormalizedValid)
        {
            Serial.print("Kp normalized [%PV/%OP] = ");
            Serial.println(r.KpNormalized, 6);
        }
        Serial.print("T0 [s] = "); Serial.println(r.T0, 6);
        Serial.print("Tp [s] = "); Serial.println(r.Tp, 6);
        Serial.print("Valid steps / Saltos validos = "); Serial.println(r.validSteps);

        FOPDTModel model = test.GetModel();
        PIDTuning tuning(model);
        PIDTuningResult imc = tuning.IMC_PI(); // default lambda = 2*T0
        PIDTuningResult lambdaPid = tuning.Lambda_PID(); // default Tf = 2*T0
        if (imc.valid)
        {
            Serial.print("IMC lambda [s] = "); Serial.println(imc.lambda, 6);
            Serial.print("IMC Kc = "); Serial.println(imc.Kc, 6);
            Serial.print("IMC Ti [s] = "); Serial.println(imc.Ti, 6);
            Serial.print("IMC Ki [1/s] = "); Serial.println(imc.Ki, 6);
        }

        if (lambdaPid.valid)
        {
            Serial.println("Lambda PID");
            Serial.print("Kc = "); Serial.println(lambdaPid.Kc, 6);
            Serial.print("Ti [s] = "); Serial.println(lambdaPid.Ti, 6);
            Serial.print("Td [s] = "); Serial.println(lambdaPid.Td, 6);
            Serial.print("Ki [1/s] = "); Serial.println(lambdaPid.Ki, 6);
            Serial.print("Kd [s] = "); Serial.println(lambdaPid.Kd, 6);
        }

        resultPrinted = true;
    }

    if (test.IsAborted() && !resultPrinted)
    {
        Serial.print("StepTest aborted / StepTest abortado: ");
        Serial.println(StepTest::ErrorName(test.GetError()));
        resultPrinted = true;
    }
}
