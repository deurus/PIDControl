# PIDControl 1.4.2

Arduino process-control library covering signal conditioning, closed-loop PID control, process identification and tuning.

## Modules / Modulos

- `PIDControl`: PID, PI-D and I-PD; MAN/AUTO; DIRECT/REVERSE; PV Tracking; bumpless transfer; conditional-integration anti-windup.
- `StepTest`: non-blocking FOPDT identification (`Kp`, `T0`, `Tp`) with robust steady-state and directional-response detection.
- `RelayTest`: non-blocking relay identification (`Ku`, `Tu`) and compact `RelayTestConfig` API.
- `PIDTuning`: IMC/Lambda PI, Lambda PID and Tyreus-Luyben PI/PID calculations.
- `PIDControlFilters`: moving average, median, first-order low-pass and complementary filters.

## Identification and tuning / Identificacion y sintonia

### StepTest -> FOPDT -> PI/PID

```cpp
FOPDTModel model = stepTest.GetModel();
PIDTuning tuning(model);

PIDTuningResult pi  = tuning.IMC_PI();      // default lambda = 2*T0
PIDTuningResult pid = tuning.Lambda_PID();  // default Tf = 2*T0
```

IMC/Lambda PI:

```text
Kc = Tp / (abs(Kp)*(lambda + T0))
Ti = Tp
Ki = Kc/Ti
```

Lambda PID:

```text
Kc = (Tp + T0/2) / (abs(Kp)*(Tf + T0/2))
Ti = Tp + T0/2
Td = Tp*T0/(2*Tp + T0)
Ki = Kc/Ti
Kd = Kc*Td
```

Presets for `lambda/Tf`: `AGGRESSIVE=T0`, `NORMAL=2*T0`, `ROBUST=3*T0`.

### RelayTest -> Ku/Tu -> Tyreus-Luyben

```cpp
RelayTestResult r = relay.GetResult();
PIDTuningResult pi  = PIDTuning::TyreusLuybenPI(r.Ku, r.Tu);
PIDTuningResult pid = PIDTuning::TyreusLuybenPID(r.Ku, r.Tu);
```

The library never auto-applies a proposed tuning. Use `PIDTuning::Apply()` or `SetTunings()` explicitly.

## StepTest normalized process gain / Ganancia normalizada

Safety limits and engineering ranges are deliberately separate:

```cpp
stepTest.SetOutputLimits(0, 60);
stepTest.SetPVLimits(15, 45);
stepTest.SetOutputRange(0, 100);
stepTest.SetPVRange(0, 80);
```

```text
KpNormalized = Kp * OPspan / PVspan
```

## RelayTest compact API

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

## TCLab reference sketch

`extras/PIDControl_TCLab_UNO_R4_Test_v1.4.2.ino` demonstrates the complete chain on Arduino UNO R4 WiFi + TCLab.

Compact examples:

```text
STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45,0,80,0,100)
RELAY(10,0.5,3,CURRENT,15,20,60,15,48)
```

Identification results can be reprinted while they remain in RAM:

```text
IDENT
IDENT STEP
IDENT RELAY
```

See `docs/TCLAB_TEST.md` for the full command interface.

## Filters / Filtros

```cpp
#include <PIDControlFilters.h>
using namespace PIDControlFilters;

MovingAverageFilter<10> movingAverage;
MedianFilter<5> median;
LowPassFilter lowPass(1.0);
ComplementaryFilter complementary(0.98);
```

## Documentation

- `docs/API.md`
- `docs/STEP_TEST.md`
- `docs/RELAY_TEST.md`
- `docs/PID_TUNING.md`
- `docs/FILTERS.md`
- `docs/TCLAB_TEST.md`
- `docs/TERMINOLOGIA.md`
