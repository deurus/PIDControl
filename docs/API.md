# PIDControl API / API de PIDControl

## PIDControl

```cpp
PIDControl(double *input, double *output, double *setpoint,
           double kc, double ki, double kd,
           PIDType type = PIDType::PID,
           PIDAction action = PIDAction::REVERSE,
           bool pvTracking = false);
```

Main methods / Metodos principales:

```cpp
bool Compute();
void SetMode(PIDMode mode);
PIDMode GetMode() const;
void SetManualOutput(double value);
double GetManualOutput() const;
void SetTunings(double kc, double ki, double kd);
void SetStructure(PIDType type);
void SetDirection(PIDAction action);
void SetPVTracking(bool enable);
void SetSetpoint(double value);
void SetOutputLimits(double minimum, double maximum);
void SetSampleTime(unsigned long timeMs);
```

## StepTest

```cpp
StepTest(PIDControl &controller, double *input, double *output);
```

```cpp
void SetOutputLimits(double minimum, double maximum);
void SetPVLimits(double minimum, double maximum);
void SetOutputRange(double minimum, double maximum);
void SetPVRange(double minimum, double maximum);
bool HasEngineeringRanges() const;

void SetStepSize(double value);
void SetNumberOfSteps(uint8_t steps);
void SetFirstStepUp(bool up);
void SetProcessGain(ProcessGain gain);
void SetStabilityCriteria(double band, double timeSeconds);
void SetStabilityCriteria(double band, double timeSeconds, double maxSlopePerMinute);
void SetMaxStabilitySlope(double maxSlopePerMinute);
void SetResponseThreshold(double value);
void SetMaxTestTime(double minutes);

bool Start();
void Update();
void Abort();
void Reset();
StepTestResult GetResult() const;
FOPDTModel GetModel() const;
bool GetStepResult(uint8_t index, StepTestStepResult &result) const;
```

`StepTestResult` contains engineering `Kp`, optional normalized `Kp`, `T0`, `Tp`, completed steps and valid steps. Version 1.4.2 also fixes preservation of the previous final steady-state mean as the initial PV of the next step in multi-step tests.

## RelayTest

```cpp
RelayTest(PIDControl &controller, double *input, double *output);
```

```cpp
void SetOutputLimits(double minimum, double maximum);
void SetPVLimits(double minimum, double maximum);
void SetRelayAmplitude(double value);
void SetHysteresis(double value);
void SetCycles(uint8_t cycles);
void SetReference(double value);
void UseCurrentPVAsReference();
void SetMaxTestTime(double minutes);

bool Start();
bool Start(const RelayTestConfig &config);
void Update();
void Abort();
void Reset();
RelayTestResult GetResult() const;
```

Compact API:

```cpp
RelayTestConfig cfg;
cfg.amplitude = 10.0;
cfg.hysteresis = 0.5;
cfg.cycles = 3;
cfg.useCurrentReference = true;
cfg.maxTestTimeMinutes = 15.0;
cfg.outputMin = 20.0;
cfg.outputMax = 60.0;
cfg.pvMin = 15.0;
cfg.pvMax = 48.0;
relay.Start(cfg);
```

## PIDTuning

FOPDT tuning:

```cpp
PIDTuning tuning(model);

PIDTuningResult a = tuning.IMC_PI();
PIDTuningResult b = tuning.Lambda_PI();
PIDTuningResult c = tuning.Lambda_PID();

PIDTuningResult d = tuning.IMC_PI(60.0);
PIDTuningResult e = tuning.Lambda_PID(60.0);

PIDTuningResult f = tuning.IMC_PI(IMCSpeed::ROBUST);
PIDTuningResult g = tuning.Lambda_PID(IMCSpeed::ROBUST);
```

Relay-result tuning:

```cpp
PIDTuningResult pi  = PIDTuning::TyreusLuybenPI(Ku, Tu);
PIDTuningResult pid = PIDTuning::TyreusLuybenPID(Ku, Tu);
```

Returned fields / Campos devueltos:

```text
valid, Kc, Ti, Td, Ki, Kd, lambda
```

For `Lambda_PID`, `lambda` stores `Tf`.

Apply explicitly / Aplicacion explicita:

```cpp
PIDTuning::Apply(controller, result);
```

## Filters / Filtros

```cpp
#include <PIDControlFilters.h>
using namespace PIDControlFilters;

MovingAverageFilter<10> movingAverage;
MedianFilter<5> median;
LowPassFilter lowPass(1.0);
ComplementaryFilter complementary(0.98);
```
