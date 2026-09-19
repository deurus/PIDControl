# PIDControl 1.4.2

Arduino process-control library covering the complete control chain from signal conditioning to closed-loop PID control.

Spanish documentation is available in [README_ES.md](README_ES.md).

## Control workflow

PIDControl is organized around a simple process-control workflow:

```text
Signal acquisition
       |
       v
Signal filtering
       |
       v
Process identification
       |
       v
Controller tuning
       |
       v
Closed-loop control
```

The library can also be used only as a PID controller without using the filtering, identification or tuning modules.

## 1. Signal filtering

`PIDControlFilters` provides reusable filters for conditioning the process variable before it reaches the controller or the identification algorithms.

Available filters:

- `MovingAverageFilter<N>` for random measurement noise.
- `MedianFilter<N>` for isolated spikes and outliers.
- `LowPassFilter` for continuous analog noise.
- `ComplementaryFilter` for combining two estimates of the same variable.

Example:

```cpp
#include <PIDControlFilters.h>
using namespace PIDControlFilters;

MedianFilter<5> median;
LowPassFilter lowPass(1.0);

double pvMedian = median.Update(PVRaw);
double PV = lowPass.Update(pvMedian, dt);
```

A median filter followed by a first-order low-pass filter is useful when the signal contains both isolated spikes and continuous noise.

## 2. Process identification

PIDControl provides two non-blocking identification methods.

### StepTest

`StepTest` identifies a first-order plus dead-time model and returns:

- `Kp` process gain
- `T0` dead time
- `Tp` process time constant

```cpp
FOPDTModel model = stepTest.GetModel();
```

Engineering ranges can also be defined to calculate normalized process gain:

```cpp
stepTest.SetOutputLimits(0, 60);
stepTest.SetPVLimits(15, 45);

stepTest.SetOutputRange(0, 100);
stepTest.SetPVRange(0, 80);
```

```text
KpNormalized = Kp * OPspan / PVspan
```

The safety limits used during the test are deliberately separated from the full engineering ranges.

### RelayTest

`RelayTest` performs a non-blocking relay experiment and returns:

- `Ku` ultimate gain
- `Tu` ultimate period

Compact API example:

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

## 3. Controller tuning

`PIDTuning` converts the identification results into PI or PID tuning proposals.

The library never auto-applies a proposed tuning. The user must apply it explicitly with `PIDTuning::Apply()` or `SetTunings()`.

### StepTest -> FOPDT -> PI/PID

```cpp
FOPDTModel model = stepTest.GetModel();
PIDTuning tuning(model);

PIDTuningResult pi  = tuning.IMC_PI();      // default lambda = 2*T0
PIDTuningResult pid = tuning.Lambda_PID();  // default Tf = 2*T0
```

Available presets for `lambda/Tf`:

```text
AGGRESSIVE = T0
NORMAL     = 2*T0
ROBUST     = 3*T0
```

### RelayTest -> Ku/Tu -> Tyreus-Luyben

```cpp
RelayTestResult r = relay.GetResult();

PIDTuningResult pi =
    PIDTuning::TyreusLuybenPI(r.Ku, r.Tu);

PIDTuningResult pid =
    PIDTuning::TyreusLuybenPID(r.Ku, r.Tu);
```

## 4. Closed-loop control

`PIDControl` supports:

- PID, PI-D and I-PD structures
- MAN/AUTO operation
- DIRECT and REVERSE control action
- PV Tracking
- bumpless transfer
- conditional-integration anti-windup
- configurable output limits
- configurable sample time

A minimal controller can be created with:

```cpp
PIDControl pid(&PV, &OP, &SP, Kc, Ki, Kd);
```

The default configuration is:

```text
Mode            MAN
Structure       PI_D
Action          REVERSE
PV Tracking     ON
Output limits   0..100
Sample time     100 ms
```

A complete declaration can also be used when the configuration must be explicit:

```cpp
PIDControl pid(
    &PV, &OP, &SP,
    Kc, Ki, Kd,
    PIDType::PI_D,
    PIDAction::REVERSE,
    true
);
```

Typical loop:

```cpp
void loop()
{
    PV = ReadProcessVariable();

    pid.Compute();

    WriteOutput(OP);
}
```

PIDControl can be used directly from this point without using the filtering, identification or tuning modules if suitable `Kc`, `Ki` and `Kd` values are already known.

## TCLab reference sketch

`extras/PIDControl_TCLab_UNO_R4_Test_v1.4.2.ino` demonstrates the complete chain on Arduino UNO R4 WiFi + TCLab.

The serial interface provides compact identification commands:

```text
STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45,0,80,0,100)

RELAY(10,0.5,3,CURRENT,15,20,60,15,48)
```

Stored identification results can be reprinted while they remain in RAM:

```text
IDENT
IDENT STEP
IDENT RELAY
```

See `docs/TCLAB_TEST.md` for the full command interface.

## Modules

- `PIDControlFilters`
- `StepTest`
- `RelayTest`
- `PIDTuning`
- `PIDControl`

## Documentation

- `docs/FILTERS.md`
- `docs/STEP_TEST.md`
- `docs/RELAY_TEST.md`
- `docs/PID_TUNING.md`
- `docs/API.md`
- `docs/TCLAB_TEST.md`
- `docs/TERMINOLOGIA.md`

## License

PIDControl is released under the MIT License.

Copyright (c) 2026 Garikoitz Martinez.
