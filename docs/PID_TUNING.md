# PIDTuning / Sintonizacion PID

## English

`PIDTuning` converts identification results into controller parameters without applying them automatically. Version 1.4.2 supports two FOPDT-based families and Tyreus-Luyben rules from relay-test results.

### IMC / Lambda PI from FOPDT

```cpp
FOPDTModel model = stepTest.GetModel();
PIDTuning tuning(model);
PIDTuningResult pi = tuning.IMC_PI();
```

The implemented PI rule is:

```text
Kc = Tp / (abs(Kp) * (lambda + T0))
Ti = Tp
Td = 0
Ki = Kc / Ti
Kd = 0
```

`Lambda_PI()` is an explicit terminology alias for the same rule:

```cpp
PIDTuningResult pi2 = tuning.Lambda_PI();
```

### Lambda PID from FOPDT

```cpp
PIDTuningResult pid = tuning.Lambda_PID();
```

The implemented Rivera/Lambda PID equations are:

```text
Kc = (Tp + T0/2) / (abs(Kp) * (Tf + T0/2))
Ti = Tp + T0/2
Td = Tp*T0 / (2*Tp + T0)
Ki = Kc / Ti
Kd = Kc * Td
```

The `lambda` field in `PIDTuningResult` stores `Tf` for a Lambda PID result.

### Default closed-loop speed presets

PIDControl deliberately uses the same presets for Lambda PI and Lambda PID:

```text
AGGRESSIVE -> lambda/Tf = 1*T0
NORMAL     -> lambda/Tf = 2*T0
ROBUST     -> lambda/Tf = 3*T0
```

Examples:

```cpp
PIDTuningResult piRobust  = tuning.IMC_PI(IMCSpeed::ROBUST);
PIDTuningResult pidRobust = tuning.Lambda_PID(IMCSpeed::ROBUST);
PIDTuningResult pidCustom = tuning.Lambda_PID(60.0); // Tf = 60 s
```

These presets are PIDControl defaults, not universal tuning rules. If `T0` is numerically zero, the no-argument convenience overloads use `0.1*Tp` as fallback.

`Kc` is returned positive. Process direction remains a separate `PIDAction` setting.

### Tyreus-Luyben from Ku/Tu

A `RelayTest` returns `Ku` and `Tu`. PIDControl 1.4.2 can transform those values directly:

```cpp
PIDTuningResult pi  = PIDTuning::TyreusLuybenPI(Ku, Tu);
PIDTuningResult pid = PIDTuning::TyreusLuybenPID(Ku, Tu);
```

PI:

```text
Kc = Ku / 3.2
Ti = 2.2*Tu
Td = 0
Ki = Kc / Ti
Kd = 0
```

PID:

```text
Kc = Ku / 3.3
Ti = 2.2*Tu
Td = Tu / 6.3
Ki = Kc / Ti
Kd = Kc * Td
```

### Applying a result

Application remains explicit:

```cpp
if (pid.valid)
    PIDTuning::Apply(controller, pid);
```

`Apply()` only calls `SetTunings(Kc, Ki, Kd)`. It does not change MAN/AUTO, action, structure or setpoint.

---

## Espanol

`PIDTuning` convierte resultados de identificacion en parametros del controlador sin aplicarlos automaticamente. La version 1.4.2 incorpora dos familias basadas en FOPDT y las reglas Tyreus-Luyben a partir del ensayo de rele.

### IMC / Lambda PI a partir de FOPDT

```cpp
FOPDTModel model = stepTest.GetModel();
PIDTuning tuning(model);
PIDTuningResult pi = tuning.IMC_PI();
```

Regla PI implementada:

```text
Kc = Tp / (abs(Kp) * (lambda + T0))
Ti = Tp
Td = 0
Ki = Kc / Ti
Kd = 0
```

`Lambda_PI()` es un alias explicito de terminologia para la misma regla:

```cpp
PIDTuningResult pi2 = tuning.Lambda_PI();
```

### Lambda PID a partir de FOPDT

```cpp
PIDTuningResult pid = tuning.Lambda_PID();
```

Ecuaciones Rivera/Lambda PID implementadas:

```text
Kc = (Tp + T0/2) / (abs(Kp) * (Tf + T0/2))
Ti = Tp + T0/2
Td = Tp*T0 / (2*Tp + T0)
Ki = Kc / Ti
Kd = Kc * Td
```

En un resultado Lambda PID, el campo `lambda` de `PIDTuningResult` almacena `Tf`.

### Presets de velocidad de lazo cerrado

PIDControl utiliza deliberadamente los mismos presets para Lambda PI y Lambda PID:

```text
AGGRESSIVE -> lambda/Tf = 1*T0
NORMAL     -> lambda/Tf = 2*T0
ROBUST     -> lambda/Tf = 3*T0
```

Ejemplos:

```cpp
PIDTuningResult piRobust  = tuning.IMC_PI(IMCSpeed::ROBUST);
PIDTuningResult pidRobust = tuning.Lambda_PID(IMCSpeed::ROBUST);
PIDTuningResult pidCustom = tuning.Lambda_PID(60.0); // Tf = 60 s
```

Estos presets son valores por defecto de PIDControl, no reglas universales de sintonizacion. Si `T0` es numericamente cero, las sobrecargas sin parametro usan `0.1*Tp` como fallback.

`Kc` se devuelve positivo. El sentido del proceso sigue gestionandose de forma independiente mediante `PIDAction`.

### Tyreus-Luyben a partir de Ku/Tu

Un `RelayTest` devuelve `Ku` y `Tu`. PIDControl 1.4.2 puede convertir directamente esos valores:

```cpp
PIDTuningResult pi  = PIDTuning::TyreusLuybenPI(Ku, Tu);
PIDTuningResult pid = PIDTuning::TyreusLuybenPID(Ku, Tu);
```

PI:

```text
Kc = Ku / 3.2
Ti = 2.2*Tu
Td = 0
Ki = Kc / Ti
Kd = 0
```

PID:

```text
Kc = Ku / 3.3
Ti = 2.2*Tu
Td = Tu / 6.3
Ki = Kc / Ti
Kd = Kc * Td
```

### Aplicar una sintonizacion

La aplicacion continua siendo explicita:

```cpp
if (pid.valid)
    PIDTuning::Apply(controller, pid);
```

`Apply()` solo ejecuta `SetTunings(Kc, Ki, Kd)`. No cambia MAN/AUTO, accion, estructura ni setpoint.
