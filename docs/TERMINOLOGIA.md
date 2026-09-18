# Terminology / Terminología

## English

### PV - Process Variable
Measured process value to be controlled.

### SP - Setpoint
Desired value of the process variable.

### OP - Output
Controller output sent to the final control element.

### PID structures

`PIDType::PID`: P, I and D act on error.  
`PIDType::PI_D`: P and I act on error; D acts on PV.  
`PIDType::I_PD`: I acts on error; P and D act on PV.

For `PIDAction::REVERSE`, the implemented forms are:

```text
PID:
P = Kc * e
I(k) = I(k-1) + Ki * e * Ts
D = Kd * (e - ePrevious) / Ts

PI-D:
P = Kc * e
I(k) = I(k-1) + Ki * e * Ts
D = -Kd * (PV - PVPrevious) / Ts

I-PD:
P = -Kc * PV
I(k) = I(k-1) + Ki * e * Ts
D = -Kd * (PV - PVPrevious) / Ts
```

where `e = SP - PV`.

### Tuning parameter conversion

If tuning is expressed as `Kc`, integral time `Ti` and derivative time `Td`:

```text
Ki = Kc / Ti
Kd = Kc * Td
```

`Ti`, `Td` and the internal sample time must use consistent time units. `PIDControl` uses seconds in the control equations.

### MAN / AUTO

`PIDMode::MAN`: OP is provided by `SetManualOutput()`.  
`PIDMode::AUTO`: OP is calculated by the controller.

### PV Tracking
When enabled in MAN, active SP follows PV. The operator setpoint is stored separately.

### Bumpless transfer
When changing to AUTO or changing controller parameters in AUTO, the integral term is initialized so that the new calculation starts close to the current OP.

### Anti-windup
Conditional integration stops integral accumulation when the unsaturated OP is outside a limit and integration would drive it farther into saturation.

### Process gain sign in StepTest
`ProcessGain::POSITIVE`: increasing OP eventually increases PV.
`ProcessGain::NEGATIVE`: increasing OP eventually decreases PV.

`StepTest` uses this sign to validate the direction of the measured response after each OP step.

### Steady-state slope in StepTest
The stability slope is the least-squares linear-regression slope of PV over the active stability window, expressed in PV engineering units per minute. A steady state must satisfy both the configured PV range and the maximum absolute slope.

---

## Español

### PV - Variable de proceso
Valor medido del proceso que se desea controlar.

### SP - Setpoint o consigna
Valor deseado de la variable de proceso.

### OP - Salida
Salida del controlador enviada al elemento final de control.

### Estructuras PID

`PIDType::PID`: P, I y D actúan sobre el error.  
`PIDType::PI_D`: P e I actúan sobre el error; D actúa sobre PV.  
`PIDType::I_PD`: I actúa sobre el error; P y D actúan sobre PV.

Para `PIDAction::REVERSE`, las formas implementadas son:

```text
PID:
P = Kc * e
I(k) = I(k-1) + Ki * e * Ts
D = Kd * (e - eAnterior) / Ts

PI-D:
P = Kc * e
I(k) = I(k-1) + Ki * e * Ts
D = -Kd * (PV - PVAnterior) / Ts

I-PD:
P = -Kc * PV
I(k) = I(k-1) + Ki * e * Ts
D = -Kd * (PV - PVAnterior) / Ts
```

donde `e = SP - PV`.

### Conversión de parámetros de sintonía

Si la sintonía está expresada mediante `Kc`, tiempo integral `Ti` y tiempo derivativo `Td`:

```text
Ki = Kc / Ti
Kd = Kc * Td
```

`Ti`, `Td` y el tiempo de muestreo interno deben utilizar unidades temporales coherentes. `PIDControl` usa segundos en las ecuaciones de control.

### MAN / AUTO

`PIDMode::MAN`: OP se fija mediante `SetManualOutput()`.  
`PIDMode::AUTO`: OP es calculada por el controlador.

### PV Tracking
Cuando está activo en MAN, el SP activo sigue a PV. La consigna del operador se conserva por separado.

### Transferencia bumpless
Al pasar a AUTO o modificar parámetros del controlador en AUTO, el término integral se inicializa para que el nuevo cálculo comience cerca de la OP actual.

### Anti-windup
La integración condicional detiene la acumulación integral cuando la OP sin limitar está fuera de límites y la integral la empujaría aún más hacia la saturación.

### Signo de la ganancia del proceso en StepTest
`ProcessGain::POSITIVE`: al aumentar OP, PV termina aumentando.
`ProcessGain::NEGATIVE`: al aumentar OP, PV termina disminuyendo.

`StepTest` utiliza este signo para validar el sentido de la respuesta medida después de cada salto de OP.

### Pendiente de estado estacionario en StepTest
La pendiente de estabilidad es la pendiente de la regresión lineal por mínimos cuadrados de PV durante la ventana activa de estabilidad, expresada en unidades de ingeniería de PV por minuto. Para aceptar un estado estacionario deben cumplirse simultáneamente el rango de PV configurado y la pendiente absoluta máxima.
