# PIDControl Filters / Filtros PIDControl

## English

`PIDControlFilters.h` contains reusable signal-conditioning blocks independent from `PIDControl`, `StepTest` and the hardware layer.

```cpp
#include <PIDControlFilters.h>
using namespace PIDControlFilters;
```

### MovingAverageFilter<N>

```cpp
MovingAverageFilter<10> filter;
double pv = filter.Update(rawPV);
```

Equation:

```text
y[k] = (1/N) * sum(x[k-i]), i=0..N-1
```

A moving average reduces random noise but adds delay. Use a window appropriate to the process dynamics.

### MedianFilter<N>

```cpp
MedianFilter<5> filter;
double pv = filter.Update(rawPV);
```

A median filter is especially useful against isolated spikes or outliers. Small odd windows such as 3 or 5 samples are normally appropriate on microcontrollers.

### LowPassFilter

```cpp
LowPassFilter filter(1.0); // tau = 1 s
double pv = filter.Update(rawPV, dtSeconds);
```

The exact first-order discrete coefficient is:

```text
alpha = 1 - exp(-dt/tau)
y[k] = y[k-1] + alpha*(x[k]-y[k-1])
```

The public parameter is the physical time constant `tau`, not an arbitrary smoothing factor.

### ComplementaryFilter

A complementary filter is intended to fuse two estimates of the same physical variable that have complementary frequency characteristics. It is not simply another low-pass filter.

Generic fusion:

```cpp
ComplementaryFilter filter(0.98);
double value = filter.Update(fastEstimate, slowReference);
```

Integrated-rate form, common for IMUs:

```cpp
double angle = filter.UpdateIntegrated(gyroRate, accelAngle, dtSeconds);
```

Equation:

```text
y[k] = alpha*(y[k-1] + rate*dt) + (1-alpha)*reference
```

### Common methods

All filter classes provide a consistent small API where applicable:

```cpp
filter.Reset();
filter.Reset(initialValue);
filter.Update(...);
filter.Value();
filter.IsInitialized();
```

### PID and identification impact

Filtering improves signal quality but also changes dynamics. Excessive filtering adds apparent delay and can increase the `T0` identified by `StepTest`. For slow thermal systems, a small filter time constant compared with the process time constant is usually acceptable.

---

## Español

`PIDControlFilters.h` contiene bloques reutilizables de acondicionamiento de señal, independientes de `PIDControl`, `StepTest` y del hardware.

```cpp
#include <PIDControlFilters.h>
using namespace PIDControlFilters;
```

### MovingAverageFilter<N>

```cpp
MovingAverageFilter<10> filter;
double pv = filter.Update(rawPV);
```

Ecuación:

```text
y[k] = (1/N) * sum(x[k-i]), i=0..N-1
```

La media móvil reduce ruido aleatorio, pero introduce retardo. La ventana debe elegirse de acuerdo con la dinámica del proceso.

### MedianFilter<N>

```cpp
MedianFilter<5> filter;
double pv = filter.Update(rawPV);
```

La mediana es especialmente útil para eliminar picos aislados y valores espurios. En microcontroladores suelen ser adecuadas ventanas impares pequeñas, por ejemplo 3 o 5 muestras.

### LowPassFilter

```cpp
LowPassFilter filter(1.0); // tau = 1 s
double pv = filter.Update(rawPV, dtSeconds);
```

El coeficiente discreto exacto de primer orden es:

```text
alpha = 1 - exp(-dt/tau)
y[k] = y[k-1] + alpha*(x[k]-y[k-1])
```

El parámetro público es la constante de tiempo física `tau`, no un factor de suavizado arbitrario.

### ComplementaryFilter

El filtro complementario está pensado para fusionar dos estimaciones de una misma magnitud con comportamiento complementario en frecuencia. No debe utilizarse como si fuera simplemente otro filtro paso bajo.

Fusión genérica:

```cpp
ComplementaryFilter filter(0.98);
double value = filter.Update(fastEstimate, slowReference);
```

Forma con integración de una velocidad, habitual en IMU:

```cpp
double angle = filter.UpdateIntegrated(gyroRate, accelAngle, dtSeconds);
```

Ecuación:

```text
y[k] = alpha*(y[k-1] + rate*dt) + (1-alpha)*reference
```

### Métodos comunes

Las clases mantienen una API pequeña y coherente cuando aplica:

```cpp
filter.Reset();
filter.Reset(initialValue);
filter.Update(...);
filter.Value();
filter.IsInitialized();
```

### Impacto en PID e identificación

El filtrado mejora la calidad de señal, pero también modifica la dinámica observada. Un filtrado excesivo añade retardo aparente y puede aumentar el `T0` identificado por `StepTest`. En procesos térmicos lentos, una constante de filtro pequeña respecto a la constante de tiempo del proceso suele ser aceptable.
