# TCLab test sketch / Sketch de prueba TCLab

The `extras/PIDControl_TCLab_UNO_R4_Test_v1.4.2.ino` sketch is the reference hardware-validation console for PIDControl. The serial parser belongs to the sketch, not to the generic library.

## Hardware / Hardware

```text
Arduino UNO R4 WiFi
T1 = A0
T2 = A2
Q1 = D3
Q2 = D5
Serial = 115200 baud
```

The acquisition path is:

```text
ADC average (10) -> MedianFilter<5> -> LowPassFilter(tau) -> PV
```

Plot frames are:

```text
#SP PV OP T2
```

## Normal operation / Operacion normal

The TCLab application limits operator SP to `10..80 C`. The independent hardware/application trip is also configured in the sketch and remains separate from StepTest/RelayTest limits.

Typical normal-operation commands:

```text
MAN 40
TUNE 6.404957 0.042629 0
AUTO 50
```

## Compact StepTest / StepTest compacto

Standard format:

```text
STEP(size,n,UP|DOWN,POS|NEG,stableBand,stableTime,maxSlope,response,maxMin,opMin,opMax,pvMin,pvMax)
```

Example:

```text
STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45)
```

With engineering ranges for normalized Kp:

```text
STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45,0,80,0,100)
```

Parameter meaning:

```text
10       OP step size [%]
2        number of steps
UP       first step direction
POS      process gain sign
0.3      steady-state half-band [C]
60       steady-state observation [s]
0.10     max regression slope [C/min]
0.4      minimum directional response [C]
30       complete-test timeout [min]
0,60     test OP limits [%]
15,45    test PV limits [C]
0,80     optional full PV engineering range
0,100    optional full OP engineering range
```

## Compact RelayTest / RelayTest compacto

Version 1.4.2 includes a serial compact form that maps directly to `RelayTestConfig`:

```text
RELAY(amplitude,hysteresis,cycles,CURRENT|reference,maxMin,opMin,opMax,pvMin,pvMax)
```

Validated TCLab example:

```text
MAN 40
RELAY(10,0.5,3,CURRENT,15,20,60,15,48)
```

Meaning:

```text
10       relay amplitude [%OP]
0.5      PV hysteresis [C]
3        valid cycles used in the result
CURRENT  capture current PV as relay reference
15       timeout [min]
20,60    test OP limits [%]
15,48    test PV limits [C]
```

A numeric reference can replace `CURRENT`.

## IDENT command / Comando IDENT

After a successful StepTest or RelayTest the calculated result stays in RAM. If the serial log has been cleared, it can be printed again:

```text
IDENT
IDENT STEP
IDENT RELAY
```

`IDENT` prints every identification currently stored. StepTest memory includes `Kp`, optional normalized Kp, `T0`, `Tp` and the current IMC/Lambda PI + Lambda PID `TUNE` proposals. Relay memory includes `Ku`, `Tu` and Tyreus-Luyben PI/PID `TUNE` proposals.

The stored result is volatile. It is lost by `RESET`, board reset/power loss, or when a new test of the same type overwrites its runtime state.

## Result tuning / Sintonizacion tras el resultado

A valid StepTest reports:

```text
FOPDT -> IMC/Lambda PI
      -> Lambda PID
```

A valid RelayTest reports:

```text
Ku, Tu -> Tyreus-Luyben PI
       -> Tyreus-Luyben PID
```

No proposed tuning is auto-applied. The user may apply the printed `TUNE Kc Ki Kd` command explicitly.
