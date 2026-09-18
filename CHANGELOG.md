# Changelog

## 1.4.2

### English
- Added `PIDTuning::Lambda_PI()` aliases and Rivera/Lambda PID tuning from `FOPDTModel`.
- Lambda PID returns `Kc`, `Ti`, `Td`, `Ki`, `Kd` and the selected `Tf` in the existing `lambda` field.
- Added `PIDTuning::TyreusLuybenPI(Ku, Tu)` and `TyreusLuybenPID(Ku, Tu)`.
- Fixed multi-step `StepTest` transition so the previous final stable mean is preserved as the next step initial PV.
- TCLab reference sketch updated to operating SP 10..80 C and independent 80 C hard trip.
- Added compact serial `RELAY(...)` example mapped to `RelayTestConfig`.
- Added `IDENT`, `IDENT STEP` and `IDENT RELAY` to reprint identification data retained in RAM after the serial log is cleared.
- TCLab StepTest now reports IMC/Lambda PI and Lambda PID; RelayTest reports Tyreus-Luyben PI and PID. No tuning is auto-applied.

### Espanol
- Anadidos alias `PIDTuning::Lambda_PI()` y sintonia Rivera/Lambda PID a partir de `FOPDTModel`.
- Lambda PID devuelve `Kc`, `Ti`, `Td`, `Ki`, `Kd` y el `Tf` seleccionado en el campo existente `lambda`.
- Anadidos `PIDTuning::TyreusLuybenPI(Ku, Tu)` y `TyreusLuybenPID(Ku, Tu)`.
- Corregida la transicion de `StepTest` multisalto para conservar la media estacionaria final del salto anterior como PV inicial del siguiente.
- Sketch TCLab actualizado a SP operativo 10..80 C y proteccion independiente de 80 C.
- Anadido comando serie compacto `RELAY(...)` basado en `RelayTestConfig`.
- Anadidos `IDENT`, `IDENT STEP` e `IDENT RELAY` para volver a mostrar la identificacion conservada en RAM si se borra el log serie.
- El StepTest TCLab informa IMC/Lambda PI y Lambda PID; RelayTest informa Tyreus-Luyben PI y PID. Ninguna sintonia se aplica automaticamente.

## 1.4.1

- Added `RelayTestConfig` for one-call relay test configuration and start.
- Added `RelayTest::Start(const RelayTestConfig&)`.
- Relay test configuration/validation remains inside PIDControl; serial command parsing remains an application-layer concern.
- Updated RelayAutoTune example and relay documentation.

 / Registro de cambios

## 1.4.0

### English
- Added `FOPDTModel` and `StepTest::GetModel()`.
- Added independent PV/OP engineering ranges to `StepTest`.
- Added normalized process gain `KpNormalized` in `%PV/%OP`.
- Added `PIDTuning` with IMC-PI tuning.
- Default IMC-PI lambda is `2*T0`; explicit lambda and AGGRESSIVE/NORMAL/ROBUST presets are supported.
- Added `PIDControlFilters.h`: moving average, median, first-order low-pass and complementary filters.
- TCLab example now uses the library filter classes and reports IMC-PI tuning after a valid StepTest.
- Compact `STEP(...)` supports optional engineering ranges as a 17-parameter form.

### Español
- Añadidos `FOPDTModel` y `StepTest::GetModel()`.
- Añadidos rangos de ingeniería de PV/OP independientes de los límites de seguridad.
- Añadida ganancia de proceso normalizada `KpNormalized` en `%PV/%OP`.
- Añadida clase `PIDTuning` con sintonización IMC-PI.
- Lambda IMC-PI por defecto = `2*T0`; se admiten lambda explícito y presets AGGRESSIVE/NORMAL/ROBUST.
- Añadido `PIDControlFilters.h`: media móvil, mediana, paso bajo de primer orden y filtro complementario.
- El ejemplo TCLab utiliza ahora los filtros de la librería e informa de la sintonía IMC-PI tras un StepTest válido.
- El comando compacto `STEP(...)` admite rangos de ingeniería opcionales en formato de 17 parámetros.

# 1.3.0

## English
- `StepTest` steady-state detection now combines a PV range criterion with a least-squares linear-regression slope criterion.
- Added `ProcessGain::POSITIVE` / `ProcessGain::NEGATIVE` and `SetProcessGain()` for directional response validation.
- Downward steps no longer accept continuing PV rise from residual inertia as a valid response in positive-gain processes.
- Added three-argument `SetStabilityCriteria(band, timeSeconds, maxSlopePerMinute)` and `SetMaxStabilitySlope()`.
- Added human-readable `StateName()`, `ErrorName()` and `ProcessGainName()`.
- TCLab compact command expanded to include process-gain direction and maximum stability slope. Legacy 11-parameter syntax remains accepted.
- TCLab StepTest defaults changed to 60 s stability observation, 0.10 C/min maximum slope and 30 min total timeout.

## Espanol
- La deteccion de estado estacionario de `StepTest` combina ahora un criterio de rango de PV con un criterio de pendiente calculada mediante regresion lineal por minimos cuadrados.
- Añadidos `ProcessGain::POSITIVE` / `ProcessGain::NEGATIVE` y `SetProcessGain()` para validar el sentido de la respuesta.
- En procesos de ganancia positiva, los saltos descendentes ya no aceptan como respuesta valida que PV siga aumentando por la inercia residual.
- Añadidos `SetStabilityCriteria(band, timeSeconds, maxSlopePerMinute)` y `SetMaxStabilitySlope()`.
- Añadidos nombres legibles mediante `StateName()`, `ErrorName()` y `ProcessGainName()`.
- El comando compacto TCLab incluye ahora el sentido de la ganancia y la pendiente maxima de estabilidad. Se conserva la sintaxis anterior de 11 parametros.
- Los valores por defecto del StepTest para TCLab pasan a 60 s de observacion estable, 0.10 C/min de pendiente maxima y 30 min de timeout total.

# 1.2.1

## English
- `StepTest::Reset()` and `RelayTest::Reset()` are now implemented inline in their headers.
- This avoids `undefined reference` linker errors caused by stale Arduino build objects or mixed library files after an update.
- No behavioral/API change relative to 1.2.0.

## Español
- `StepTest::Reset()` y `RelayTest::Reset()` se implementan ahora inline en sus cabeceras.
- Esto evita errores de enlazado `undefined reference` provocados por objetos de compilación en caché o archivos mezclados tras actualizar la librería.
- No hay cambios de comportamiento/API respecto a 1.2.0.

# Changelog / Registro de cambios

## 1.2.0

### English
- Added `StepTest::Reset()` and `RelayTest::Reset()` to clear runtime state, errors and stored results and return to `IDLE`.
- `Reset()` safely returns a running identification test to MAN at its original bias before clearing state.

### Español
- Añadidos `StepTest::Reset()` y `RelayTest::Reset()` para borrar el estado de ejecución, errores y resultados almacenados y volver a `IDLE`.
- `Reset()` devuelve de forma segura un ensayo de identificación en ejecución a MAN con su bias original antes de borrar el estado.

## English

### 1.1.0

- Added `StepTest.h/.cpp` for non-blocking FOPDT step-test identification.
- Added configurable number of steps, OP/PV safety limits, stability criteria and response threshold.
- `StepTest::SetMaxTestTime(double minutes)` accepts minutes and converts them internally to milliseconds.
- Added `RelayTest.h/.cpp` for non-blocking relay testing.
- Added relay amplitude, hysteresis, cycle count, OP/PV safety limits and internal relay reference.
- `RelayTest::SetMaxTestTime(double minutes)` accepts minutes and converts them internally to milliseconds.
- Both tests leave the PID in MAN and restore the initial OP bias when finished or aborted.
- Added bilingual English/Spanish documentation and examples.

### 1.0.0

- Initial PIDControl implementation.
- PID, PI-D and I-PD structures.
- DIRECT/REVERSE action.
- MAN/AUTO modes.
- PV Tracking, bumpless transfer and conditional-integration anti-windup.

---

## Español

### 1.1.0

- Añadidos `StepTest.h/.cpp` para identificación FOPDT no bloqueante mediante ensayo de escalón.
- Añadidos número configurable de saltos, límites de seguridad OP/PV, criterios de estabilidad y umbral de respuesta.
- `StepTest::SetMaxTestTime(double minutes)` recibe minutos y los convierte internamente a milisegundos.
- Añadidos `RelayTest.h/.cpp` para ensayo no bloqueante mediante relé.
- Añadidos amplitud de relé, histéresis, número de ciclos, límites de seguridad OP/PV y referencia interna del relé.
- `RelayTest::SetMaxTestTime(double minutes)` recibe minutos y los convierte internamente a milisegundos.
- Ambos ensayos dejan el PID en MAN y restauran el bias inicial de OP al finalizar o abortar.
- Añadida documentación y ejemplos bilingües inglés/español.

### 1.0.0

- Implementación inicial de PIDControl.
- Estructuras PID, PI-D e I-PD.
- Acción DIRECT/REVERSE.
- Modos MAN/AUTO.
- PV Tracking, transferencia bumpless y anti-windup por integración condicional.
