# StepTest

## English

`StepTest` performs a non-blocking open-loop step-response test and estimates a First Order Plus Dead Time (FOPDT) model:

```text
G(s) = Kp * exp(-T0*s) / (Tp*s + 1)
```

The class remains hardware-independent: it receives pointers to the already-scaled `PV` and `OP` variables. Pins, ADC conversion, PWM, I2C, SPI, Modbus or other hardware belong outside `StepTest`.

### Safety limits versus engineering ranges

They are intentionally different concepts.

```cpp
test.SetOutputLimits(0.0, 60.0); // safety during the test
test.SetPVLimits(15.0, 45.0);    // safety during the test

test.SetOutputRange(0.0, 100.0); // full OP engineering range
test.SetPVRange(0.0, 100.0);     // full PV engineering range
```

Safety limits can be narrower than the full ranges. Engineering ranges are used only to calculate normalized gain.

### Process gain

Engineering gain:

```text
Kp = DeltaPV / DeltaOP
```

Normalized gain:

```text
KpNormalized = (DeltaPV/PVspan) / (DeltaOP/OPspan)
             = Kp * OPspan / PVspan
```

`KpNormalized` therefore represents `%PV/%OP`. It is reported only when both PV and OP engineering ranges have been configured.

### Robust steady state

```cpp
test.SetStabilityCriteria(0.30, 60.0, 0.10);
```

A window is stable only when both conditions are met:

```text
PVmax - PVmin <= 2*band
abs(regression slope) <= maxSlope
```

The slope is obtained by least-squares linear regression and is expressed in PV engineering-units per minute.

### Directional response

```cpp
test.SetProcessGain(ProcessGain::POSITIVE);
```

For `POSITIVE`, OP up must eventually produce PV up and OP down must produce PV down. For `NEGATIVE` the relationship is reversed. This avoids accepting residual inertia in the wrong direction as a valid response.

### FOPDT estimation

The 28.3 % and 63.2 % crossing method is used:

```text
Tp = 1.5*(t63.2 - t28.3)
T0 = 1.5*t28.3 - 0.5*t63.2
```

### Model object

```cpp
FOPDTModel model = test.GetModel();
```

The model contains `Kp`, optional normalized `Kp`, `T0`, `Tp` and the engineering ranges used for normalization. It can be passed directly to `PIDTuning`.

### Sequence and safety behavior

In multi-step tests, version 1.4.2 preserves the final stable mean of one step as the initial stable PV of the next step before applying the next OP target.


`StepTest` forces MAN, stores the current OP as bias, waits for initial stability, performs the configured alternating steps, estimates each model and averages valid results. At finish or abort it returns OP to the original bias and leaves the controller in MAN.

The full-test maximum time is configured in minutes:

```cpp
test.SetMaxTestTime(30.0);
```

### Limitation

The method assumes an approximately monotonic FOPDT response in the tested operating region. Integrating, strongly oscillatory, strongly nonlinear or inverse-response processes can require another identification method.

---

## Español

`StepTest` realiza un ensayo no bloqueante de respuesta a escalón en lazo abierto y estima un modelo de primer orden con tiempo muerto (FOPDT):

```text
G(s) = Kp * exp(-T0*s) / (Tp*s + 1)
```

La clase sigue siendo independiente del hardware: recibe punteros a las variables `PV` y `OP` ya escaladas. Los pines, conversión ADC, PWM, I2C, SPI, Modbus u otro hardware quedan fuera de `StepTest`.

### Límites de seguridad frente a rangos de ingeniería

Son conceptos intencionadamente diferentes.

```cpp
test.SetOutputLimits(0.0, 60.0); // seguridad durante el ensayo
test.SetPVLimits(15.0, 45.0);    // seguridad durante el ensayo

test.SetOutputRange(0.0, 100.0); // rango completo de OP
test.SetPVRange(0.0, 100.0);     // rango completo de PV
```

Los límites de seguridad pueden ser más estrechos que los rangos completos. Los rangos de ingeniería solo se utilizan para calcular la ganancia normalizada.

### Ganancia de proceso

Ganancia en unidades de ingeniería:

```text
Kp = DeltaPV / DeltaOP
```

Ganancia normalizada:

```text
KpNormalized = (DeltaPV/PVspan) / (DeltaOP/OPspan)
             = Kp * OPspan / PVspan
```

`KpNormalized` representa por tanto `%PV/%OP`. Solo se informa cuando se han configurado los rangos completos de PV y OP.

### Estado estacionario robusto

```cpp
test.SetStabilityCriteria(0.30, 60.0, 0.10);
```

Una ventana solo se considera estable cuando se cumplen simultáneamente:

```text
PVmax - PVmin <= 2*band
abs(pendiente de regresion) <= maxSlope
```

La pendiente se obtiene mediante regresión lineal por mínimos cuadrados y se expresa en unidades de ingeniería de PV por minuto.

### Respuesta direccional

```cpp
test.SetProcessGain(ProcessGain::POSITIVE);
```

Con `POSITIVE`, una subida de OP debe terminar provocando una subida de PV, y una bajada de OP una bajada de PV. Con `NEGATIVE` la relación es inversa. Esto evita aceptar la inercia residual en sentido incorrecto como una respuesta válida.

### Estimación FOPDT

Se utiliza el método de cruces al 28,3 % y 63,2 %:

```text
Tp = 1.5*(t63.2 - t28.3)
T0 = 1.5*t28.3 - 0.5*t63.2
```

### Objeto de modelo

```cpp
FOPDTModel model = test.GetModel();
```

El modelo contiene `Kp`, `Kp` normalizado opcional, `T0`, `Tp` y los rangos utilizados para normalización. Puede pasarse directamente a `PIDTuning`.

### Secuencia y comportamiento seguro

En ensayos multisalto, la version 1.4.2 conserva la media estacionaria final de un salto como PV inicial estable del siguiente antes de aplicar el nuevo objetivo de OP.


`StepTest` fuerza MAN, guarda la OP actual como bias, espera estabilidad inicial, ejecuta los saltos alternos configurados, estima cada modelo y promedia los resultados válidos. Al finalizar o abortar devuelve OP al bias original y deja el controlador en MAN.

El tiempo máximo del ensayo completo se configura en minutos:

```cpp
test.SetMaxTestTime(30.0);
```

### Limitación

El método supone una respuesta aproximadamente monótona y FOPDT en la región de operación ensayada. Los procesos integradores, fuertemente oscilatorios, muy no lineales o con respuesta inversa pueden requerir otro método de identificación.
