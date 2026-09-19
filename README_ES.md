# PIDControl 1.4.2

Librería de control de procesos para Arduino que cubre toda la cadena de control desde el tratamiento de la señal hasta el control PID en lazo cerrado.

La documentación principal en inglés está disponible en [README.md](README.md).

Como introducción resumida al funcionamiento de la librería también puedes consultar:

[PIDControl, librería de control para Arduino](https://garikoitz.info/blog/2026/09/pidcontrol-libreria-de-control-para-arduino/)

## Flujo de trabajo

PIDControl está organizada alrededor de una cadena de trabajo sencilla:

```text
Adquisición de señal
        |
        v
Filtrado
        |
        v
Identificación
        |
        v
Sintonización
        |
        v
Control en lazo cerrado
```

La librería también puede utilizarse únicamente como controlador PID sin necesidad de utilizar los módulos de filtrado, identificación o sintonización.

## 1. Filtrado de la señal

`PIDControlFilters` incorpora filtros reutilizables para acondicionar la variable de proceso antes de entregarla al controlador o a los algoritmos de identificación.

Filtros disponibles:

- `MovingAverageFilter<N>` para ruido aleatorio de medida.
- `MedianFilter<N>` para picos aislados y valores espurios.
- `LowPassFilter` para ruido continuo en señales analógicas.
- `ComplementaryFilter` para combinar dos estimaciones de una misma variable.

Ejemplo:

```cpp
#include <PIDControlFilters.h>
using namespace PIDControlFilters;

MedianFilter<5> median;
LowPassFilter lowPass(1.0);

double pvMedian = median.Update(PVRaw);
double PV = lowPass.Update(pvMedian, dt);
```

La combinación de una mediana seguida de un paso bajo resulta útil cuando la señal contiene tanto picos aislados como ruido continuo.

## 2. Identificación del proceso

PIDControl incorpora dos métodos de identificación no bloqueantes.

### StepTest

`StepTest` identifica un modelo de primer orden con tiempo muerto y devuelve:

- `Kp` ganancia del proceso
- `T0` tiempo muerto
- `Tp` constante de tiempo del proceso

```cpp
FOPDTModel model = stepTest.GetModel();
```

También pueden definirse los rangos de ingeniería para calcular la ganancia normalizada:

```cpp
stepTest.SetOutputLimits(0, 60);
stepTest.SetPVLimits(15, 45);

stepTest.SetOutputRange(0, 100);
stepTest.SetPVRange(0, 80);
```

```text
KpNormalized = Kp * OPspan / PVspan
```

Los límites de seguridad utilizados durante el ensayo se mantienen separados de los rangos completos de ingeniería.

### RelayTest

`RelayTest` realiza un ensayo de relé no bloqueante y devuelve:

- `Ku` ganancia última
- `Tu` periodo último

Ejemplo utilizando la API compacta:

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

## 3. Sintonización del controlador

`PIDTuning` convierte los resultados de identificación en propuestas de sintonización PI o PID.

La librería nunca aplica automáticamente una sintonización calculada. El usuario debe aplicarla de forma explícita mediante `PIDTuning::Apply()` o `SetTunings()`.

### StepTest -> FOPDT -> PI/PID

```cpp
FOPDTModel model = stepTest.GetModel();
PIDTuning tuning(model);

PIDTuningResult pi  = tuning.IMC_PI();      // lambda por defecto = 2*T0
PIDTuningResult pid = tuning.Lambda_PID();  // Tf por defecto = 2*T0
```

Presets disponibles para `lambda/Tf`:

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

## 4. Control en lazo cerrado

`PIDControl` permite trabajar con:

- estructuras PID, PI-D e I-PD
- modos MAN/AUTO
- acción DIRECT y REVERSE
- PV Tracking
- transferencia bumpless
- anti-windup mediante integración condicional
- límites configurables de salida
- tiempo de muestreo configurable

La declaración mínima del controlador es:

```cpp
PIDControl pid(&PV, &OP, &SP, Kc, Ki, Kd);
```

La configuración por defecto es:

```text
Modo                MAN
Estructura          PI_D
Acción              REVERSE
PV Tracking         ON
Límites de salida   0..100
Tiempo de muestreo  100 ms
```

También puede utilizarse la declaración completa cuando queramos dejar toda la configuración de forma explícita:

```cpp
PIDControl pid(
    &PV, &OP, &SP,
    Kc, Ki, Kd,
    PIDType::PI_D,
    PIDAction::REVERSE,
    true
);
```

Un `loop()` típico puede reducirse prácticamente a:

```cpp
void loop()
{
    PV = LeerVariableProceso();

    pid.Compute();

    EscribirSalida(OP);
}
```

PIDControl puede utilizarse directamente desde este punto sin pasar previamente por los módulos de filtrado, identificación o sintonización si ya disponemos de valores adecuados de `Kc`, `Ki` y `Kd`.

## Sketch de referencia TCLab

`extras/PIDControl_TCLab_UNO_R4_Test_v1.4.2.ino` demuestra la cadena completa utilizando Arduino UNO R4 WiFi + TCLab.

La interfaz serie incluye comandos compactos para lanzar los ensayos de identificación:

```text
STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45,0,80,0,100)

RELAY(10,0.5,3,CURRENT,15,20,60,15,48)
```

Los resultados de identificación pueden volver a mostrarse mientras permanezcan almacenados en RAM:

```text
IDENT
IDENT STEP
IDENT RELAY
```

Consulta `docs/TCLAB_TEST.md` para ver la interfaz completa de comandos.

## Módulos

- `PIDControlFilters`
- `StepTest`
- `RelayTest`
- `PIDTuning`
- `PIDControl`

## Documentación

- `docs/FILTERS.md`
- `docs/STEP_TEST.md`
- `docs/RELAY_TEST.md`
- `docs/PID_TUNING.md`
- `docs/API.md`
- `docs/TCLAB_TEST.md`
- `docs/TERMINOLOGIA.md`

## Licencia

PIDControl se distribuye bajo licencia MIT.

Copyright (c) 2026 Garikoitz Martinez.
