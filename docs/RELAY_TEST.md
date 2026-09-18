# RelayTest

## English

`RelayTest` performs a non-blocking relay experiment inspired by the Astrom-Hagglund relay autotuning principle. Its identification result is `Ku` and `Tu`; tuning remains in `PIDTuning`.

### Operation

The current OP is captured as `biasOutput`. The relay alternates:

```text
OPhigh = bias + d
OPlow  = bias - d
```

Typical configuration:

```cpp
relay.SetRelayAmplitude(10.0);
relay.SetHysteresis(0.5);
relay.SetCycles(3);
relay.UseCurrentPVAsReference();
relay.SetMaxTestTime(15.0);     // minutes
relay.SetOutputLimits(20, 60);
relay.SetPVLimits(15, 48);
relay.Start();
```

For positive process gain associated with `PIDAction::REVERSE`:

```text
PV >= reference + hysteresis -> OPlow
PV <= reference - hysteresis -> OPhigh
```

For `PIDAction::DIRECT` the switching sense is reversed.

### Compact configuration API

Since 1.4.1 a full configuration can be passed in one call:

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

### Calculation

The first complete oscillation is discarded as startup transient. For every subsequent valid cycle:

```text
a  = (PVmax - PVmin) / 2
Ku = 4*d / (pi*a)
Tu = average period between equivalent relay transitions
```

The result remains available through `relay.GetResult()` until the object is reset or another relay test overwrites it.

### Tyreus-Luyben tuning

Version 1.4.2 adds integrated tuning helpers after identification:

```cpp
RelayTestResult r = relay.GetResult();
PIDTuningResult pi  = PIDTuning::TyreusLuybenPI(r.Ku, r.Tu);
PIDTuningResult pid = PIDTuning::TyreusLuybenPID(r.Ku, r.Tu);
```

The tuning is not auto-applied.

### Safety and completion

At finish or abort the original OP bias is restored and the PID remains in MAN. `Reset()` clears stored state/results; configuration values are preserved.

---

## Espanol

`RelayTest` realiza un ensayo no bloqueante mediante rele inspirado en el principio de autotuning de Astrom-Hagglund. El resultado de identificacion es `Ku` y `Tu`; la sintonizacion se mantiene separada en `PIDTuning`.

### Funcionamiento

La OP actual se guarda como `biasOutput`. El rele alterna:

```text
OPhigh = bias + d
OPlow  = bias - d
```

Configuracion tipica:

```cpp
relay.SetRelayAmplitude(10.0);
relay.SetHysteresis(0.5);
relay.SetCycles(3);
relay.UseCurrentPVAsReference();
relay.SetMaxTestTime(15.0);     // minutos
relay.SetOutputLimits(20, 60);
relay.SetPVLimits(15, 48);
relay.Start();
```

Para una ganancia positiva asociada a `PIDAction::REVERSE`:

```text
PV >= referencia + histeresis -> OPlow
PV <= referencia - histeresis -> OPhigh
```

Con `PIDAction::DIRECT` se invierte el sentido de conmutacion.

### API de configuracion compacta

Desde 1.4.1 puede configurarse y arrancarse el ensayo en una llamada:

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

### Calculo

La primera oscilacion completa se descarta como transitorio de arranque. Para cada ciclo valido posterior:

```text
a  = (PVmax - PVmin) / 2
Ku = 4*d / (pi*a)
Tu = periodo medio entre transiciones equivalentes del rele
```

El resultado permanece disponible mediante `relay.GetResult()` hasta resetear el objeto o iniciar otro ensayo de rele que lo sobrescriba.

### Sintonizacion Tyreus-Luyben

La version 1.4.2 incorpora helpers de sintonizacion tras la identificacion:

```cpp
RelayTestResult r = relay.GetResult();
PIDTuningResult pi  = PIDTuning::TyreusLuybenPI(r.Ku, r.Tu);
PIDTuningResult pid = PIDTuning::TyreusLuybenPID(r.Ku, r.Tu);
```

La sintonizacion no se aplica automaticamente.

### Seguridad y finalizacion

Al finalizar o abortar se restaura el bias original de OP y el PID permanece en MAN. `Reset()` borra estado/resultados almacenados y conserva la configuracion.
