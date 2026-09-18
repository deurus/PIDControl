#include <Arduino.h>
#include <PIDControl.h>
#include <StepTest.h>
#include <RelayTest.h>
#include <PIDControlFilters.h>
#include <PIDTuning.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ============================================================
// TCLab + Arduino UNO R4 WiFi
// PIDControl 1.4.2 complete functional test - operating limits 10..80 C
// ============================================================

// TCLab shield pinout (channel 1 used for control)
const uint8_t PIN_T1 = A0;
const uint8_t PIN_T2 = A2;
const uint8_t PIN_Q1 = 3;
const uint8_t PIN_Q2 = 5;

// Serial / Arduino COM Plotter
const unsigned long SERIAL_BAUD = 115200;
const unsigned long PLOT_PERIOD_MS = 500;
const unsigned long PROCESS_IO_PERIOD_MS = 100;

// UNO R4 ADC. The standard TCLab shield uses an external AREF.
// Measure AREF-GND on your board and correct this value if necessary.
const uint8_t ADC_BITS = 14;
const double ADC_REFERENCE_V = 3.300;
const uint8_t TEMP_SAMPLES = 10;

// TCLab standard firmware uses P1=200 and P2=100 (PWM 0..255).
// Keeping Q1 at 200 avoids applying more heater power than the
// standard TCLab firmware when OP=100%.
const uint8_t Q1_PWM_MAX = 200;
const uint8_t Q2_PWM_MAX = 100;

// Normal operating setpoint limits for the TCLab.
// These limits apply to operator SP commands in normal PID operation.
const double OPERATING_SP_MIN_C = 10.0;
const double OPERATING_SP_MAX_C = 80.0;

// Independent hard temperature protection. This is intentionally separate
// from StepTest / RelayTest PV limits and remains active in MAN, AUTO and
// during identification tests.
const double HARD_TEMP_HIGH_C = 80.0;
const double HARD_TEMP_RAW_MARGIN_C = 2.0;
const double HARD_TEMP_RESET_MARGIN_C = 2.0;
const double HARD_TEMP_RAW_HIGH_C = HARD_TEMP_HIGH_C + HARD_TEMP_RAW_MARGIN_C;
const double HARD_TEMP_RESET_C = HARD_TEMP_HIGH_C - HARD_TEMP_RESET_MARGIN_C;

// PV filtering. A complementary filter is intended to fuse two different
// measurements of the same variable. TCLab has one temperature measurement
// per channel, so a robust median + first-order low-pass filter is more
// appropriate. The median stage removes isolated spikes; the low-pass stage
// smooths the remaining ADC noise.
const double DEFAULT_FILTER_TAU_S = 1.0;

// PID sample time
const unsigned long PID_SAMPLE_TIME_MS = 100;

// Default test configuration. RESET restores these values.
const double DEFAULT_SP_C = 35.0;
const double DEFAULT_KC = 5.0;
const double DEFAULT_KI = 0.05;
const double DEFAULT_KD = 0.0;

const double DEFAULT_PID_OP_MIN = 0.0;
const double DEFAULT_PID_OP_MAX = 100.0;

const double DEFAULT_STEP_OP_MIN = 0.0;
const double DEFAULT_STEP_OP_MAX = 60.0;
const double DEFAULT_STEP_PV_MIN = 15.0;
const double DEFAULT_STEP_PV_MAX = 45.0;
const double DEFAULT_STEP_PV_RANGE_MIN = 0.0;
const double DEFAULT_STEP_PV_RANGE_MAX = 100.0;
const double DEFAULT_STEP_OP_RANGE_MIN = 0.0;
const double DEFAULT_STEP_OP_RANGE_MAX = 100.0;
const double DEFAULT_STEP_SIZE = 10.0;
const uint8_t DEFAULT_STEP_COUNT = 2;
const double DEFAULT_STEP_STABLE_BAND_C = 0.30;
const double DEFAULT_STEP_STABLE_TIME_S = 60.0;
const double DEFAULT_STEP_STABLE_SLOPE_C_MIN = 0.10;
const double DEFAULT_STEP_RESPONSE_C = 0.40;
const double DEFAULT_STEP_MAX_TIME_MIN = 30.0;

const double DEFAULT_RELAY_OP_MIN = 0.0;
const double DEFAULT_RELAY_OP_MAX = 60.0;
const double DEFAULT_RELAY_PV_MIN = 15.0;
const double DEFAULT_RELAY_PV_MAX = 45.0;
const double DEFAULT_RELAY_AMPLITUDE = 10.0;
const double DEFAULT_RELAY_HYST_C = 0.50;
const uint8_t DEFAULT_RELAY_CYCLES = 3;
const double DEFAULT_RELAY_MAX_TIME_MIN = 15.0;

// Initial process values
// Kc/Ki/Kd are deliberately conservative starting values, not a
// validated tuning for your particular TCLab.
double SP = DEFAULT_SP_C;
double PV = 0.0;          // Filtered T1 used by PID / StepTest / RelayTest
double PVRaw = 0.0;       // Unfiltered T1 for diagnostics
double OP = 0.0;
double T2 = 0.0;          // Filtered T2
double T2Raw = 0.0;       // Unfiltered T2 for diagnostics

double Kc = DEFAULT_KC;
double Ki = DEFAULT_KI;
double Kd = DEFAULT_KD;

bool lowPassFilterEnabled = true;
double filterTauS = DEFAULT_FILTER_TAU_S;

PIDControl pid(
    &PV, &OP, &SP,
    Kc, Ki, Kd,
    PIDType::PI_D,
    PIDAction::REVERSE,
    true
);

StepTest stepTest(pid, &PV, &OP);
RelayTest relayTest(pid, &PV, &OP);

// Serial command parser
char commandBuffer[128];
uint8_t commandIndex = 0;

void PrintIdentificationMemory(const char *selector);

// Plot / reporting state
unsigned long previousPlotMs = 0;
unsigned long previousProcessIoMs = 0;
bool stepResultReported = false;
bool relayResultReported = false;
StepTestState previousStepState = StepTestState::IDLE;
bool temperatureTrip = false;

// ============================================================
// PV FILTERING - PIDControl 1.4.2 filter module
// ============================================================

using PIDControlFilters::MedianFilter;
using PIDControlFilters::LowPassFilter;

MedianFilter<5> pvMedian;
MedianFilter<5> t2Median;
LowPassFilter pvLowPass(DEFAULT_FILTER_TAU_S);
LowPassFilter t2LowPass(DEFAULT_FILTER_TAU_S);

// ============================================================
// TCLAB I/O
// ============================================================

double ReadTemperatureC(uint8_t pin)
{
    uint32_t sum = 0;

    // Discard the first conversion after selecting the ADC channel. This
    // reduces residual charge/crosstalk when alternating between T1 and T2.
    (void)analogRead(pin);

    for (uint8_t i = 0; i < TEMP_SAMPLES; ++i)
        sum += analogRead(pin);

    const double adc = (double)sum / TEMP_SAMPLES;
    const double adcMax = (double)((1UL << ADC_BITS) - 1UL);
    const double voltage = adc * ADC_REFERENCE_V / adcMax;

    // TCLab temperature sensors use the TMP36 transfer function:
    // T[degC] = (V - 0.5) * 100
    return (voltage - 0.5) * 100.0;
}

void SetHeaterPercent(uint8_t pin, double percent, uint8_t pwmMax)
{
    if (percent < 0.0) percent = 0.0;
    if (percent > 100.0) percent = 100.0;

    const int pwm = (int)lround(percent * pwmMax / 100.0);
    analogWrite(pin, pwm);
}

void ResetSignalFilters()
{
    pvMedian.Reset(PVRaw);
    t2Median.Reset(T2Raw);
    pvLowPass.SetTimeConstant(filterTauS);
    t2LowPass.SetTimeConstant(filterTauS);
    pvLowPass.Reset(PVRaw);
    t2LowPass.Reset(T2Raw);
    PV = PVRaw;
    T2 = T2Raw;
    previousProcessIoMs = millis();
}

void UpdateProcessIO(bool force = false)
{
    const unsigned long now = millis();

    if (!force && previousProcessIoMs != 0 &&
        (unsigned long)(now - previousProcessIoMs) < PROCESS_IO_PERIOD_MS)
        return;

    double dtSeconds = PROCESS_IO_PERIOD_MS / 1000.0;
    if (previousProcessIoMs != 0)
        dtSeconds = (now - previousProcessIoMs) / 1000.0;

    previousProcessIoMs = now;

    PVRaw = ReadTemperatureC(PIN_T1);
    T2Raw = ReadTemperatureC(PIN_T2);

    const double pvMedianValue = pvMedian.Update(PVRaw);
    const double t2MedianValue = t2Median.Update(T2Raw);

    if (lowPassFilterEnabled)
    {
        pvLowPass.SetTimeConstant(filterTauS);
        t2LowPass.SetTimeConstant(filterTauS);
        PV = pvLowPass.Update(pvMedianValue, dtSeconds);
        T2 = t2LowPass.Update(t2MedianValue, dtSeconds);
    }
    else
    {
        PV = pvMedianValue;
        T2 = t2MedianValue;
        // Keep the low-pass internal state aligned for a bumpless FILTER ON.
        pvLowPass.Reset(PV);
        t2LowPass.Reset(T2);
    }
}

void WriteProcessOutput()
{
    SetHeaterPercent(PIN_Q1, OP, Q1_PWM_MAX);
    SetHeaterPercent(PIN_Q2, 0.0, Q2_PWM_MAX);
}

// ============================================================
// SAFETY
// ============================================================

bool AnyTestRunning()
{
    return stepTest.IsRunning() || relayTest.IsRunning();
}

void AbortTests()
{
    if (stepTest.IsRunning())
        stepTest.Abort();

    if (relayTest.IsRunning())
        relayTest.Abort();
}

void EmergencyStop(const char *reason)
{
    AbortTests();
    pid.SetMode(PIDMode::MAN);
    pid.SetManualOutput(0.0);
    OP = 0.0;
    SetHeaterPercent(PIN_Q1, 0.0, Q1_PWM_MAX);
    SetHeaterPercent(PIN_Q2, 0.0, Q2_PWM_MAX);

    if (!temperatureTrip)
    {
        Serial.print("! EMERGENCY STOP: ");
        Serial.println(reason);
    }

    temperatureTrip = true;
}

void CheckHardSafety()
{
    if (PV >= HARD_TEMP_HIGH_C || T2 >= HARD_TEMP_HIGH_C ||
        PVRaw >= HARD_TEMP_RAW_HIGH_C || T2Raw >= HARD_TEMP_RAW_HIGH_C)
    {
        EmergencyStop("TCLab high temperature");
        return;
    }

    if (temperatureTrip && PV < HARD_TEMP_RESET_C && T2 < HARD_TEMP_RESET_C)
    {
        // Do not restore AUTO. Only clear the alarm indication.
        temperatureTrip = false;
        Serial.println("! Temperature back inside safe range. PID remains in MAN at OP=0.");
    }
}

// ============================================================
// ARDUINO COM PLOTTER
// ============================================================

void SendPlotFrame()
{
    const unsigned long now = millis();
    if ((unsigned long)(now - previousPlotMs) < PLOT_PERIOD_MS)
        return;

    previousPlotMs = now;

    // Arduino COM Plotter configuration:
    // Start character: #
    // Separator: space
    // Variables: 4
    // 1=SP, 2=T1/PV, 3=Q1/OP, 4=T2
    Serial.print('#');
    Serial.print(SP, 2);
    Serial.print(' ');
    Serial.print(PV, 2);
    Serial.print(' ');
    Serial.print(OP, 2);
    Serial.print(' ');
    Serial.println(T2, 2);
}

// ============================================================
// TEXT HELPERS
// ============================================================

const char *ModeName(PIDMode mode)
{
    return mode == PIDMode::AUTO ? "AUTO" : "MAN";
}

const char *TypeName(PIDType type)
{
    switch (type)
    {
        case PIDType::PID:  return "PID";
        case PIDType::PI_D: return "PI_D";
        case PIDType::I_PD: return "I_PD";
        default:            return "?";
    }
}

const char *ActionName(PIDAction action)
{
    return action == PIDAction::REVERSE ? "REVERSE" : "DIRECT";
}

void PrintStatus()
{
    Serial.println("--- PIDControl status ---");
    Serial.print("Mode: "); Serial.println(ModeName(pid.GetMode()));
    Serial.print("Structure: "); Serial.println(TypeName(pid.GetStructure()));
    Serial.print("Action: "); Serial.println(ActionName(pid.GetDirection()));
    Serial.print("PV tracking: "); Serial.println(pid.GetPVTracking() ? "ON" : "OFF");

    Serial.print("SP internal [C]: "); Serial.println(SP, 3);
    Serial.print("SP operator [C]: "); Serial.println(pid.GetOperatorSetpoint(), 3);
    Serial.print("PV T1 filtered [C]: "); Serial.println(PV, 3);
    Serial.print("PV T1 raw [C]: "); Serial.println(PVRaw, 3);
    Serial.print("T2 filtered [C]: "); Serial.println(T2, 3);
    Serial.print("T2 raw [C]: "); Serial.println(T2Raw, 3);
    Serial.print("PV low-pass filter: "); Serial.println(lowPassFilterEnabled ? "ON" : "OFF (median only)");
    Serial.print("Filter tau [s]: "); Serial.println(filterTauS, 3);
    Serial.print("OP [%]: "); Serial.println(OP, 3);
    Serial.print("Manual OP [%]: "); Serial.println(pid.GetManualOutput(), 3);

    Serial.print("Kc: "); Serial.println(pid.GetKc(), 6);
    Serial.print("Ki [1/s]: "); Serial.println(pid.GetKi(), 6);
    Serial.print("Kd [s]: "); Serial.println(pid.GetKd(), 6);

    Serial.print("Output limits [%]: ");
    Serial.print(pid.GetOutputMinimum(), 2);
    Serial.print(" .. ");
    Serial.println(pid.GetOutputMaximum(), 2);

    Serial.print("Sample time [ms]: "); Serial.println(pid.GetSampleTime());
    Serial.print("Operating SP limits [C]: "); Serial.print(OPERATING_SP_MIN_C, 2); Serial.print(" .. "); Serial.println(OPERATING_SP_MAX_C, 2);
    Serial.print("Hard temperature trip [C]: "); Serial.println(HARD_TEMP_HIGH_C, 2);
    Serial.print("Hard raw temperature trip [C]: "); Serial.println(HARD_TEMP_RAW_HIGH_C, 2);
    Serial.print("Hard temperature reset [C]: "); Serial.println(HARD_TEMP_RESET_C, 2);
    Serial.print("StepTest process gain: "); Serial.println(StepTest::ProcessGainName(stepTest.GetProcessGain()));
    Serial.print("StepTest PV range: "); Serial.print(stepTest.GetPVRangeMinimum(), 2); Serial.print(" .. "); Serial.println(stepTest.GetPVRangeMaximum(), 2);
    Serial.print("StepTest OP range: "); Serial.print(stepTest.GetOutputRangeMinimum(), 2); Serial.print(" .. "); Serial.println(stepTest.GetOutputRangeMaximum(), 2);
    Serial.print("StepTest stability: +/-"); Serial.print(stepTest.GetStabilityBand(), 3);
    Serial.print(" C / "); Serial.print(stepTest.GetStabilityTime(), 1);
    Serial.print(" s, max slope "); Serial.print(stepTest.GetMaxStabilitySlope(), 3); Serial.println(" C/min");
    Serial.print("StepTest current slope [C/min]: "); Serial.println(stepTest.GetCurrentStabilitySlope(), 4);
    Serial.print("StepTest max time [min]: "); Serial.println(stepTest.GetMaxTestTime(), 3);
    Serial.print("RelayTest max time [min]: "); Serial.println(relayTest.GetMaxTestTime(), 3);
    Serial.print("StepTest state/error: ");
    Serial.print(StepTest::StateName(stepTest.GetState())); Serial.print('/'); Serial.println(StepTest::ErrorName(stepTest.GetError()));
    Serial.print("RelayTest state/error: ");
    Serial.print((int)relayTest.GetState()); Serial.print('/'); Serial.println((int)relayTest.GetError());
    Serial.println("-------------------------");
}

void PrintHelp()
{
    Serial.println("=== PIDControl TCLab test commands ===");
    Serial.println("HELP");
    Serial.println("STATUS");
    Serial.println("IDENT [STEP|RELAY]           -> print stored identification results");
    Serial.println("RESET                        -> MAN, OP=0 and restore all defaults");
    Serial.println("FILTER <tau_s>               -> PIDControl Median(5) + LowPass(tau)");
    Serial.println("FILTER OFF                   -> median(5) only");
    Serial.println("STOP                         -> abort tests, MAN, OP=0");
    Serial.println("MAN [op]                     -> MAN mode; optional OP %");
    Serial.println("AUTO [sp]                    -> AUTO mode; optional SP C (10..80)");
    Serial.println("SP <value>                   -> operator setpoint C (10..80)");
    Serial.println("OP <value>                   -> manual output %");
    Serial.println("TUNE <Kc> <Ki> <Kd>");
    Serial.println("STRUCT PID|PI_D|I_PD");
    Serial.println("ACTION DIRECT|REVERSE");
    Serial.println("TRACK ON|OFF");
    Serial.println("OUTLIMITS <min> <max>");
    Serial.println("TS <milliseconds>");
    Serial.println();
    Serial.println("STEP START | STEP ABORT");
    Serial.println("STEP(size,n,UP|DOWN,POS|NEG,stableBand,stableTime,maxSlope,response,maxMin,opMin,opMax,pvMin,pvMax)");
    Serial.println("Optional ranges: append pvRangeMin,pvRangeMax,opRangeMin,opRangeMax (17 parameters)");
    Serial.println("  Example: STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45)");
    Serial.println("  With ranges: STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45,0,80,0,100)");
    Serial.println("  Legacy 11-param STEP(...) is also accepted as POS with default maxSlope.");
    Serial.println("STEP SIZE <percent>");
    Serial.println("STEP N <1..10>");
    Serial.println("STEP DIR UP|DOWN");
    Serial.println("STEP GAIN POS|NEG");
    Serial.println("STEP STABLE <band_C> <time_s> [maxSlope_C_min]");
    Serial.println("STEP RESPONSE <delta_C>");
    Serial.println("STEP TIME <minutes>");
    Serial.println("STEP LIMITS <opMin> <opMax> <pvMin_C> <pvMax_C>");
    Serial.println("STEP RANGE <pvRangeMin> <pvRangeMax> <opRangeMin> <opRangeMax>");
    Serial.println();
    Serial.println("RELAY(amp,hyst,cycles,CURRENT|ref,maxMin,opMin,opMax,pvMin,pvMax)");
    Serial.println("  Example: RELAY(10,0.5,3,CURRENT,15,20,60,15,48)");
    Serial.println("RELAY START | RELAY ABORT");
    Serial.println("RELAY AMP <percent>");
    Serial.println("RELAY HYST <degC>");
    Serial.println("RELAY CYCLES <n>");
    Serial.println("RELAY REF CURRENT | RELAY REF <degC>");
    Serial.println("RELAY TIME <minutes>");
    Serial.println("RELAY LIMITS <opMin> <opMax> <pvMin_C> <pvMax_C>");
    Serial.println();
    Serial.println("Arduino COM Plotter: 115200 baud, start '#', separator space, 4 variables");
    Serial.println("Variables: SP  T1_PV  Q1_OP  T2");
    Serial.print("Normal operating SP range [C]: "); Serial.print(OPERATING_SP_MIN_C, 1); Serial.print(" .. "); Serial.println(OPERATING_SP_MAX_C, 1);
    Serial.print("Independent hard temperature trip [C]: "); Serial.println(HARD_TEMP_HIGH_C, 1);
    Serial.println("======================================");
}

bool ParseDouble(const char *text, double &value)
{
    if (text == nullptr) return false;
    char *endPtr = nullptr;
    value = strtod(text, &endPtr);
    return endPtr != text && *endPtr == '\0';
}

bool ParseUnsignedLong(const char *text, unsigned long &value)
{
    if (text == nullptr) return false;
    char *endPtr = nullptr;
    value = strtoul(text, &endPtr, 10);
    return endPtr != text && *endPtr == '\0';
}

void TrimCommand(char *text)
{
    if (text == nullptr) return;

    // Remove leading spaces, tabs and control characters.
    char *start = text;
    while (*start && (unsigned char)*start <= ' ')
        ++start;

    if (start != text)
        memmove(text, start, strlen(start) + 1);

    // Remove trailing spaces, tabs and control characters.
    size_t len = strlen(text);
    while (len > 0 && (unsigned char)text[len - 1] <= ' ')
        text[--len] = '\0';
}

void Uppercase(char *text)
{
    while (*text)
    {
        *text = (char)toupper((unsigned char)*text);
        ++text;
    }
}

bool IsOperatingSetpointValid(double value)
{
    return value >= OPERATING_SP_MIN_C && value <= OPERATING_SP_MAX_C;
}

bool RejectIfTestRunning()
{
    if (!AnyTestRunning())
        return false;

    Serial.println("! Command rejected: a StepTest or RelayTest is running. Use ABORT, RESET or STOP first.");
    return true;
}

void ApplyDefaultConfiguration(bool announce)
{
    // Clear identification state first. If a test is running, Reset() safely
    // returns to MAN at the original bias before clearing state/results.
    stepTest.Reset();
    relayTest.Reset();
    stepResultReported = false;
    relayResultReported = false;
    previousStepState = StepTestState::IDLE;

    pid.SetMode(PIDMode::MAN);
    pid.SetOutputLimits(DEFAULT_PID_OP_MIN, DEFAULT_PID_OP_MAX);
    pid.SetSampleTime(PID_SAMPLE_TIME_MS);
    pid.SetStructure(PIDType::PI_D);
    pid.SetDirection(PIDAction::REVERSE);
    pid.SetTunings(DEFAULT_KC, DEFAULT_KI, DEFAULT_KD);
    pid.SetPVTracking(true);
    pid.SetSetpoint(DEFAULT_SP_C);       // Stored as operator SP in MAN+tracking.
    pid.SetManualOutput(0.0);
    pid.Compute();                       // Applies OP=0 and SP=PV in MAN.

    stepTest.SetOutputLimits(DEFAULT_STEP_OP_MIN, DEFAULT_STEP_OP_MAX);
    stepTest.SetPVLimits(DEFAULT_STEP_PV_MIN, DEFAULT_STEP_PV_MAX);
    stepTest.SetOutputRange(DEFAULT_STEP_OP_RANGE_MIN, DEFAULT_STEP_OP_RANGE_MAX);
    stepTest.SetPVRange(DEFAULT_STEP_PV_RANGE_MIN, DEFAULT_STEP_PV_RANGE_MAX);
    stepTest.SetStepSize(DEFAULT_STEP_SIZE);
    stepTest.SetNumberOfSteps(DEFAULT_STEP_COUNT);
    stepTest.SetFirstStepUp(true);
    stepTest.SetProcessGain(ProcessGain::POSITIVE);
    stepTest.SetStabilityCriteria(DEFAULT_STEP_STABLE_BAND_C, DEFAULT_STEP_STABLE_TIME_S, DEFAULT_STEP_STABLE_SLOPE_C_MIN);
    stepTest.SetResponseThreshold(DEFAULT_STEP_RESPONSE_C);
    stepTest.SetMaxTestTime(DEFAULT_STEP_MAX_TIME_MIN);

    relayTest.SetOutputLimits(DEFAULT_RELAY_OP_MIN, DEFAULT_RELAY_OP_MAX);
    relayTest.SetPVLimits(DEFAULT_RELAY_PV_MIN, DEFAULT_RELAY_PV_MAX);
    relayTest.SetRelayAmplitude(DEFAULT_RELAY_AMPLITUDE);
    relayTest.SetHysteresis(DEFAULT_RELAY_HYST_C);
    relayTest.SetCycles(DEFAULT_RELAY_CYCLES);
    relayTest.UseCurrentPVAsReference();
    relayTest.SetMaxTestTime(DEFAULT_RELAY_MAX_TIME_MIN);

    lowPassFilterEnabled = true;
    filterTauS = DEFAULT_FILTER_TAU_S;
    ResetSignalFilters();

    OP = 0.0;
    WriteProcessOutput();

    if (announce)
    {
        Serial.println("! RESET: defaults restored. PID=MAN, OP=0, limits/tunings/tests/filter reset.");
        Serial.print("! Operating SP range [C]: "); Serial.print(OPERATING_SP_MIN_C, 1); Serial.print(" .. "); Serial.println(OPERATING_SP_MAX_C, 1);
        Serial.print("! Hard temperature protection remains active at [C]: "); Serial.println(HARD_TEMP_HIGH_C, 1);
    }
}

void ProcessStepInlineCommand(char *line)
{
    // Standard format (13 parameters):
    // STEP(size,n,UP|DOWN,POS|NEG,stableBand,stableTime,maxSlope,response,maxMin,opMin,opMax,pvMin,pvMax)
    // Optional engineering ranges (17 parameters): append
    // pvRangeMin,pvRangeMax,opRangeMin,opRangeMax.
    // Example TCLab:
    // STEP(10,2,UP,POS,0.3,60,0.10,0.4,30,0,60,15,45,0,100,0,100)
    //
    // Legacy 11-parameter format is accepted for compatibility and uses
    // POSITIVE process gain plus DEFAULT_STEP_STABLE_SLOPE_C_MIN.

    if (relayTest.IsRunning() || stepTest.IsRunning())
    {
        Serial.println("! STEP(...) rejected: an identification test is already running.");
        return;
    }

    char *open = strchr(line, '(');
    char *close = strrchr(line, ')');
    if (open == nullptr || close == nullptr || close <= open)
    {
        Serial.println("! Invalid STEP(...) syntax. Use HELP.");
        return;
    }

    *close = '\0';
    char *args = open + 1;

    const uint8_t MAX_TOKENS = 18;
    char *tokens[MAX_TOKENS];
    uint8_t count = 0;
    char *save = nullptr;
    char *tok = strtok_r(args, " ,\t", &save);
    while (tok != nullptr && count < MAX_TOKENS)
    {
        tokens[count++] = tok;
        tok = strtok_r(nullptr, " ,\t", &save);
    }

    const bool legacy = (count == 11);
    const bool hasRanges = (count == 17);
    if (!legacy && count != 13 && !hasRanges)
    {
        Serial.println("! STEP(...) requires 13 or 17 parameters (11 legacy also accepted).");
        Serial.println("! STEP(size,n,UP|DOWN,POS|NEG,stableBand,stableTime,maxSlope,response,maxMin,opMin,opMax,pvMin,pvMax[,pvRangeMin,pvRangeMax,opRangeMin,opRangeMax])");
        return;
    }

    double size, stableBand, stableTime, maxSlope, response, maxMinutes;
    double opMin, opMax, pvMin, pvMax;
    double pvRangeMin = stepTest.GetPVRangeMinimum();
    double pvRangeMax = stepTest.GetPVRangeMaximum();
    double opRangeMin = stepTest.GetOutputRangeMinimum();
    double opRangeMax = stepTest.GetOutputRangeMaximum();
    unsigned long stepsUL;
    bool firstUp;
    ProcessGain gain = ProcessGain::POSITIVE;

    if (!ParseDouble(tokens[0], size) || !ParseUnsignedLong(tokens[1], stepsUL))
    {
        Serial.println("! STEP(...) has an invalid size or step count.");
        return;
    }

    if (strcmp(tokens[2], "UP") == 0) firstUp = true;
    else if (strcmp(tokens[2], "DOWN") == 0) firstUp = false;
    else
    {
        Serial.println("! STEP direction must be UP or DOWN.");
        return;
    }

    if (legacy)
    {
        maxSlope = DEFAULT_STEP_STABLE_SLOPE_C_MIN;
        if (!ParseDouble(tokens[3], stableBand) ||
            !ParseDouble(tokens[4], stableTime) ||
            !ParseDouble(tokens[5], response) ||
            !ParseDouble(tokens[6], maxMinutes) ||
            !ParseDouble(tokens[7], opMin) ||
            !ParseDouble(tokens[8], opMax) ||
            !ParseDouble(tokens[9], pvMin) ||
            !ParseDouble(tokens[10], pvMax))
        {
            Serial.println("! STEP(...) has an invalid numeric parameter.");
            return;
        }
    }
    else
    {
        if (strcmp(tokens[3], "POS") == 0 || strcmp(tokens[3], "POSITIVE") == 0)
            gain = ProcessGain::POSITIVE;
        else if (strcmp(tokens[3], "NEG") == 0 || strcmp(tokens[3], "NEGATIVE") == 0)
            gain = ProcessGain::NEGATIVE;
        else
        {
            Serial.println("! STEP process gain must be POS or NEG.");
            return;
        }

        if (!ParseDouble(tokens[4], stableBand) ||
            !ParseDouble(tokens[5], stableTime) ||
            !ParseDouble(tokens[6], maxSlope) ||
            !ParseDouble(tokens[7], response) ||
            !ParseDouble(tokens[8], maxMinutes) ||
            !ParseDouble(tokens[9], opMin) ||
            !ParseDouble(tokens[10], opMax) ||
            !ParseDouble(tokens[11], pvMin) ||
            !ParseDouble(tokens[12], pvMax))
        {
            Serial.println("! STEP(...) has an invalid numeric parameter.");
            return;
        }

        if (hasRanges &&
            (!ParseDouble(tokens[13], pvRangeMin) ||
             !ParseDouble(tokens[14], pvRangeMax) ||
             !ParseDouble(tokens[15], opRangeMin) ||
             !ParseDouble(tokens[16], opRangeMax)))
        {
            Serial.println("! STEP(...) has an invalid engineering range.");
            return;
        }
    }

    if (size <= 0.0 || stepsUL < 1 || stepsUL > PIDCONTROL_STEPTEST_MAX_STEPS ||
        stableBand <= 0.0 || stableTime <= 0.0 || maxSlope <= 0.0 || response < 0.0 ||
        maxMinutes <= 0.0 || opMin >= opMax || pvMin >= pvMax ||
        pvRangeMin >= pvRangeMax || opRangeMin >= opRangeMax ||
        opMin < pid.GetOutputMinimum() || opMax > pid.GetOutputMaximum())
    {
        Serial.println("! STEP(...) configuration is outside valid limits.");
        return;
    }

    stepTest.Reset();
    stepTest.SetStepSize(size);
    stepTest.SetNumberOfSteps((uint8_t)stepsUL);
    stepTest.SetFirstStepUp(firstUp);
    stepTest.SetProcessGain(gain);
    stepTest.SetStabilityCriteria(stableBand, stableTime, maxSlope);
    stepTest.SetResponseThreshold(response);
    stepTest.SetMaxTestTime(maxMinutes);
    stepTest.SetOutputLimits(opMin, opMax);
    stepTest.SetPVLimits(pvMin, pvMax);
    stepTest.SetPVRange(pvRangeMin, pvRangeMax);
    stepTest.SetOutputRange(opRangeMin, opRangeMax);

    stepResultReported = false;
    previousStepState = StepTestState::IDLE;

    if (stepTest.Start())
    {
        Serial.println("! StepTest configured and started from STEP(...). PID forced to MAN.");
        Serial.print("! STEP: size="); Serial.print(size, 3);
        Serial.print(" n="); Serial.print(stepsUL);
        Serial.print(" dir="); Serial.print(firstUp ? "UP" : "DOWN");
        Serial.print(" gain="); Serial.print(StepTest::ProcessGainName(gain));
        Serial.print(" stable=+/-"); Serial.print(stableBand, 3);
        Serial.print("C/"); Serial.print(stableTime, 1); Serial.print("s");
        Serial.print(" slope<="); Serial.print(maxSlope, 3); Serial.print("C/min");
        Serial.print(" response="); Serial.print(response, 3);
        Serial.print("C max="); Serial.print(maxMinutes, 2); Serial.println("min");
        Serial.print("! STEP limits: OP "); Serial.print(opMin, 2); Serial.print(".."); Serial.print(opMax, 2);
        Serial.print(" %, PV "); Serial.print(pvMin, 2); Serial.print(".."); Serial.print(pvMax, 2); Serial.println(" C");
        Serial.print("! STEP ranges: PV "); Serial.print(pvRangeMin, 2); Serial.print(".."); Serial.print(pvRangeMax, 2);
        Serial.print(" C, OP "); Serial.print(opRangeMin, 2); Serial.print(".."); Serial.print(opRangeMax, 2); Serial.println(" %");
    }
    else
    {
        Serial.print("! StepTest could not start. Error=");
        Serial.println(StepTest::ErrorName(stepTest.GetError()));
    }
}

// ============================================================
// STEP TEST COMMANDS
// ============================================================

void ProcessStepCommand(char *sub, char *arg1, char *arg2, char *arg3, char *arg4)
{
    if (sub == nullptr)
    {
        Serial.println("! STEP command missing. Use HELP.");
        return;
    }

    if (strcmp(sub, "START") == 0)
    {
        if (relayTest.IsRunning())
        {
            Serial.println("! RelayTest is running.");
            return;
        }

        stepResultReported = false;
        previousStepState = StepTestState::IDLE;
        if (stepTest.Start())
        {
            Serial.println("! StepTest started. PID forced to MAN.");
            Serial.print("! Max test time [min]: ");
            Serial.println(stepTest.GetMaxTestTime(), 3);
        }
        else
        {
            Serial.print("! StepTest could not start. Error=");
            Serial.println(StepTest::ErrorName(stepTest.GetError()));
        }
        return;
    }

    if (strcmp(sub, "ABORT") == 0)
    {
        stepTest.Abort();
        Serial.println("! StepTest abort requested.");
        return;
    }

    if (RejectIfTestRunning()) return;

    double a, b, c, d;

    if (strcmp(sub, "SIZE") == 0 && ParseDouble(arg1, a))
    {
        stepTest.SetStepSize(a);
        Serial.println("! StepTest step size updated.");
    }
    else if (strcmp(sub, "N") == 0 && ParseDouble(arg1, a))
    {
        stepTest.SetNumberOfSteps((uint8_t)a);
        Serial.println("! StepTest number of steps updated.");
    }
    else if (strcmp(sub, "DIR") == 0 && arg1 != nullptr)
    {
        if (strcmp(arg1, "UP") == 0) stepTest.SetFirstStepUp(true);
        else if (strcmp(arg1, "DOWN") == 0) stepTest.SetFirstStepUp(false);
        else { Serial.println("! Use STEP DIR UP or STEP DIR DOWN."); return; }
        Serial.println("! StepTest first-step direction updated.");
    }
    else if (strcmp(sub, "GAIN") == 0 && arg1 != nullptr)
    {
        if (strcmp(arg1, "POS") == 0 || strcmp(arg1, "POSITIVE") == 0)
            stepTest.SetProcessGain(ProcessGain::POSITIVE);
        else if (strcmp(arg1, "NEG") == 0 || strcmp(arg1, "NEGATIVE") == 0)
            stepTest.SetProcessGain(ProcessGain::NEGATIVE);
        else
        {
            Serial.println("! Use STEP GAIN POS or STEP GAIN NEG.");
            return;
        }
        Serial.print("! StepTest process gain = ");
        Serial.println(StepTest::ProcessGainName(stepTest.GetProcessGain()));
    }
    else if (strcmp(sub, "STABLE") == 0 && ParseDouble(arg1, a) && ParseDouble(arg2, b))
    {
        if (arg3 != nullptr && ParseDouble(arg3, c))
            stepTest.SetStabilityCriteria(a, b, c);
        else
            stepTest.SetStabilityCriteria(a, b);
        Serial.print("! StepTest stability = +/-"); Serial.print(stepTest.GetStabilityBand(), 3);
        Serial.print(" C / "); Serial.print(stepTest.GetStabilityTime(), 1);
        Serial.print(" s, max slope "); Serial.print(stepTest.GetMaxStabilitySlope(), 3); Serial.println(" C/min");
    }
    else if (strcmp(sub, "RESPONSE") == 0 && ParseDouble(arg1, a))
    {
        stepTest.SetResponseThreshold(a);
        Serial.println("! StepTest response threshold updated.");
    }
    else if (strcmp(sub, "TIME") == 0 && ParseDouble(arg1, a))
    {
        stepTest.SetMaxTestTime(a);
        Serial.print("! StepTest max time [min]: ");
        Serial.println(stepTest.GetMaxTestTime(), 3);
    }
    else if (strcmp(sub, "RANGE") == 0 &&
             ParseDouble(arg1, a) && ParseDouble(arg2, b) &&
             ParseDouble(arg3, c) && ParseDouble(arg4, d))
    {
        if (a >= b || c >= d)
        {
            Serial.println("! STEP RANGE requires pvMin<pvMax and opMin<opMax.");
            return;
        }
        stepTest.SetPVRange(a, b);
        stepTest.SetOutputRange(c, d);
        Serial.println("! StepTest engineering ranges updated for normalized Kp.");
    }
    else if (strcmp(sub, "LIMITS") == 0 &&
             ParseDouble(arg1, a) && ParseDouble(arg2, b) &&
             ParseDouble(arg3, c) && ParseDouble(arg4, d))
    {
        stepTest.SetOutputLimits(a, b);
        stepTest.SetPVLimits(c, d);
        Serial.println("! StepTest OP/PV limits updated.");
    }
    else
    {
        Serial.println("! Invalid STEP command. Use HELP.");
    }
}

// Compact serial syntax is only a UI/parser layer. The complete relay
// configuration and validation are passed to PIDControl through RelayTestConfig.
void ProcessRelayInlineCommand(char *line)
{
    if (stepTest.IsRunning() || relayTest.IsRunning())
    {
        Serial.println("! An identification test is already running.");
        return;
    }

    char *open = strchr(line, '(');
    char *close = strrchr(line, ')');
    if (open == nullptr || close == nullptr || close <= open)
    {
        Serial.println("! Invalid RELAY(...) syntax.");
        return;
    }

    *close = '\0';
    char *payload = open + 1;

    char *args[9] = { nullptr };
    uint8_t count = 0;
    char *save = nullptr;
    char *tok = strtok_r(payload, ", \t", &save);
    while (tok != nullptr && count < 9)
    {
        args[count++] = tok;
        tok = strtok_r(nullptr, ", \t", &save);
    }

    if (count != 9 || tok != nullptr)
    {
        Serial.println("! RELAY(...) requires 9 parameters.");
        return;
    }

    double amp, hyst, cyclesD, maxMin, opMin, opMax, pvMin, pvMax;
    if (!ParseDouble(args[0], amp) || !ParseDouble(args[1], hyst) ||
        !ParseDouble(args[2], cyclesD) || !ParseDouble(args[4], maxMin) ||
        !ParseDouble(args[5], opMin) || !ParseDouble(args[6], opMax) ||
        !ParseDouble(args[7], pvMin) || !ParseDouble(args[8], pvMax))
    {
        Serial.println("! Invalid numeric parameter in RELAY(...).");
        return;
    }

    RelayTestConfig cfg;
    cfg.amplitude = amp;
    cfg.hysteresis = hyst;
    cfg.cycles = (uint8_t)cyclesD;
    cfg.maxTestTimeMinutes = maxMin;
    cfg.outputMin = opMin;
    cfg.outputMax = opMax;
    cfg.pvMin = pvMin;
    cfg.pvMax = pvMax;

    if (strcmp(args[3], "CURRENT") == 0)
    {
        cfg.useCurrentReference = true;
    }
    else
    {
        double ref;
        if (!ParseDouble(args[3], ref))
        {
            Serial.println("! RELAY reference must be CURRENT or a numeric PV.");
            return;
        }
        cfg.useCurrentReference = false;
        cfg.referencePV = ref;
    }

    relayResultReported = false;
    if (relayTest.Start(cfg))
    {
        Serial.println("! RelayTest configured and started from RELAY(...). PID forced to MAN.");
        Serial.print("! RELAY: amp=+/-"); Serial.print(cfg.amplitude, 3);
        Serial.print(" hyst=+/-"); Serial.print(cfg.hysteresis, 3);
        Serial.print(" cycles="); Serial.print(cfg.cycles);
        Serial.print(" max="); Serial.print(cfg.maxTestTimeMinutes, 2); Serial.println(" min");
        Serial.print("! RELAY limits: OP "); Serial.print(cfg.outputMin, 2);
        Serial.print(".."); Serial.print(cfg.outputMax, 2);
        Serial.print(" %, PV "); Serial.print(cfg.pvMin, 2);
        Serial.print(".."); Serial.print(cfg.pvMax, 2); Serial.println(" C");
    }
    else
    {
        Serial.print("! RelayTest could not start. Error=");
        Serial.println((int)relayTest.GetError());
    }
}

// ============================================================
// RELAY TEST COMMANDS
// ============================================================

void ProcessRelayCommand(char *sub, char *arg1, char *arg2, char *arg3, char *arg4)
{
    if (sub == nullptr)
    {
        Serial.println("! RELAY command missing. Use HELP.");
        return;
    }

    if (strcmp(sub, "START") == 0)
    {
        if (stepTest.IsRunning())
        {
            Serial.println("! StepTest is running.");
            return;
        }

        relayResultReported = false;
        if (relayTest.Start())
        {
            Serial.println("! RelayTest started. PID forced to MAN.");
            Serial.print("! Max test time [min]: ");
            Serial.println(relayTest.GetMaxTestTime(), 3);
        }
        else
        {
            Serial.print("! RelayTest could not start. Error=");
            Serial.println((int)relayTest.GetError());
        }
        return;
    }

    if (strcmp(sub, "ABORT") == 0)
    {
        relayTest.Abort();
        Serial.println("! RelayTest abort requested.");
        return;
    }

    if (RejectIfTestRunning()) return;

    double a, b, c, d;

    if (strcmp(sub, "AMP") == 0 && ParseDouble(arg1, a))
    {
        relayTest.SetRelayAmplitude(a);
        Serial.println("! Relay amplitude updated.");
    }
    else if (strcmp(sub, "HYST") == 0 && ParseDouble(arg1, a))
    {
        relayTest.SetHysteresis(a);
        Serial.println("! Relay hysteresis updated.");
    }
    else if (strcmp(sub, "CYCLES") == 0 && ParseDouble(arg1, a))
    {
        relayTest.SetCycles((uint8_t)a);
        Serial.println("! Relay cycle count updated.");
    }
    else if (strcmp(sub, "REF") == 0 && arg1 != nullptr)
    {
        if (strcmp(arg1, "CURRENT") == 0)
        {
            relayTest.UseCurrentPVAsReference();
            Serial.println("! Relay reference will use current PV at START.");
        }
        else if (ParseDouble(arg1, a))
        {
            relayTest.SetReference(a);
            Serial.println("! Relay fixed reference updated.");
        }
        else
        {
            Serial.println("! Use RELAY REF CURRENT or RELAY REF <degC>.");
        }
    }
    else if (strcmp(sub, "TIME") == 0 && ParseDouble(arg1, a))
    {
        relayTest.SetMaxTestTime(a);
        Serial.print("! RelayTest max time [min]: ");
        Serial.println(relayTest.GetMaxTestTime(), 3);
    }
    else if (strcmp(sub, "LIMITS") == 0 &&
             ParseDouble(arg1, a) && ParseDouble(arg2, b) &&
             ParseDouble(arg3, c) && ParseDouble(arg4, d))
    {
        relayTest.SetOutputLimits(a, b);
        relayTest.SetPVLimits(c, d);
        Serial.println("! RelayTest OP/PV limits updated.");
    }
    else
    {
        Serial.println("! Invalid RELAY command. Use HELP.");
    }
}

// ============================================================
// MAIN COMMAND PROCESSOR
// ============================================================

void ProcessCommand(char *line)
{
    TrimCommand(line);
    if (*line == '\0') return;

    Uppercase(line);

    // Compact one-line RelayTest configuration. The parser only builds a
    // RelayTestConfig; execution and validation remain inside PIDControl.
    if (strncmp(line, "RELAY(", 6) == 0)
    {
        ProcessRelayInlineCommand(line);
        return;
    }

    // Compact one-line StepTest configuration. Process before generic tokenization
    // because STEP(...) does not contain a space after STEP.
    if (strncmp(line, "STEP(", 5) == 0)
    {
        ProcessStepInlineCommand(line);
        return;
    }

    char *savePtr = nullptr;
    char *cmd  = strtok_r(line, " \t", &savePtr);
    char *a1   = strtok_r(nullptr, " \t", &savePtr);
    char *a2   = strtok_r(nullptr, " \t", &savePtr);
    char *a3   = strtok_r(nullptr, " \t", &savePtr);
    char *a4   = strtok_r(nullptr, " \t", &savePtr);
    char *a5   = strtok_r(nullptr, " \t", &savePtr);

    if (cmd == nullptr) return;

    if (strcmp(cmd, "HELP") == 0)
    {
        PrintHelp();
        return;
    }

    if (strcmp(cmd, "STATUS") == 0)
    {
        PrintStatus();
        return;
    }

    if (strcmp(cmd, "IDENT") == 0)
    {
        PrintIdentificationMemory(a1);
        return;
    }

    if (strcmp(cmd, "RESET") == 0)
    {
        ApplyDefaultConfiguration(true);
        return;
    }

    if (strcmp(cmd, "STOP") == 0)
    {
        AbortTests();
        pid.SetMode(PIDMode::MAN);
        pid.SetManualOutput(0.0);
        Serial.println("! STOP: PID in MAN, OP=0.");
        return;
    }

    if (strcmp(cmd, "FILTER") == 0)
    {
        if (a1 != nullptr && strcmp(a1, "OFF") == 0)
        {
            lowPassFilterEnabled = false;
            ResetSignalFilters();
            Serial.println("! PV low-pass filter OFF. Median-of-5 spike filter remains active.");
            return;
        }

        double tau;
        if (a1 != nullptr && strcmp(a1, "ON") == 0)
        {
            lowPassFilterEnabled = true;
            filterTauS = DEFAULT_FILTER_TAU_S;
            ResetSignalFilters();
            Serial.print("! PV low-pass filter ON. Tau [s] = ");
            Serial.println(filterTauS, 3);
            return;
        }
        else if (ParseDouble(a1, tau) && tau > 0.0)
        {
            lowPassFilterEnabled = true;
            filterTauS = tau;
            ResetSignalFilters();
            Serial.print("! PV low-pass filter ON. Tau [s] = ");
            Serial.println(filterTauS, 3);
            return;
        }

        Serial.println("! Use FILTER <tau_s>, FILTER ON or FILTER OFF.");
        return;
    }

    if (strcmp(cmd, "STEP") == 0)
    {
        ProcessStepCommand(a1, a2, a3, a4, a5);
        return;
    }

    if (strcmp(cmd, "RELAY") == 0)
    {
        ProcessRelayCommand(a1, a2, a3, a4, a5);
        return;
    }

    // MAN and AUTO intentionally abort identification tests.
    if (strcmp(cmd, "MAN") == 0)
    {
        AbortTests();
        pid.SetMode(PIDMode::MAN);

        double value;
        if (ParseDouble(a1, value))
            pid.SetManualOutput(value);

        Serial.println("! PID mode = MAN.");
        return;
    }

    if (strcmp(cmd, "AUTO") == 0)
    {
        AbortTests();

        double value = 0.0;
        const bool hasArgument = (a1 != nullptr);
        const bool hasSetpoint = hasArgument && ParseDouble(a1, value);

        if (hasArgument && !hasSetpoint)
        {
            Serial.println("! AUTO requires a numeric SP or no parameter.");
            return;
        }

        if (hasSetpoint && !IsOperatingSetpointValid(value))
        {
            Serial.print("! AUTO SP outside operating range [C]: ");
            Serial.print(OPERATING_SP_MIN_C, 1);
            Serial.print(" .. ");
            Serial.println(OPERATING_SP_MAX_C, 1);
            return;
        }

        if (temperatureTrip)
        {
            Serial.println("! AUTO rejected: hard temperature protection is active.");
            return;
        }

        // With PV tracking enabled, SetMode(AUTO) intentionally starts
        // at SP=PV for a bumpless transfer. Apply the operator target
        // afterwards when AUTO <sp> is explicitly requested.
        pid.SetMode(PIDMode::AUTO);
        if (hasSetpoint)
            pid.SetSetpoint(value);

        Serial.println("! PID mode = AUTO.");
        return;
    }

    if (RejectIfTestRunning()) return;

    double x, y, z;
    unsigned long ul;

    if (strcmp(cmd, "SP") == 0 && ParseDouble(a1, x))
    {
        if (!IsOperatingSetpointValid(x))
        {
            Serial.print("! SP outside operating range [C]: ");
            Serial.print(OPERATING_SP_MIN_C, 1);
            Serial.print(" .. ");
            Serial.println(OPERATING_SP_MAX_C, 1);
            return;
        }

        pid.SetSetpoint(x);
        Serial.println("! Operator SP updated.");
    }
    else if (strcmp(cmd, "OP") == 0 && ParseDouble(a1, x))
    {
        pid.SetManualOutput(x);
        Serial.println("! Manual OP updated.");
    }
    else if (strcmp(cmd, "TUNE") == 0 &&
             ParseDouble(a1, x) && ParseDouble(a2, y) && ParseDouble(a3, z))
    {
        pid.SetTunings(x, y, z);
        Serial.println("! Kc/Ki/Kd updated.");
    }
    else if (strcmp(cmd, "STRUCT") == 0 && a1 != nullptr)
    {
        if      (strcmp(a1, "PID") == 0)  pid.SetStructure(PIDType::PID);
        else if (strcmp(a1, "PI_D") == 0) pid.SetStructure(PIDType::PI_D);
        else if (strcmp(a1, "I_PD") == 0) pid.SetStructure(PIDType::I_PD);
        else { Serial.println("! Use STRUCT PID, PI_D or I_PD."); return; }

        Serial.print("! PID structure = ");
        Serial.println(TypeName(pid.GetStructure()));
    }
    else if (strcmp(cmd, "ACTION") == 0 && a1 != nullptr)
    {
        if      (strcmp(a1, "DIRECT") == 0)  pid.SetDirection(PIDAction::DIRECT);
        else if (strcmp(a1, "REVERSE") == 0) pid.SetDirection(PIDAction::REVERSE);
        else { Serial.println("! Use ACTION DIRECT or ACTION REVERSE."); return; }

        Serial.print("! PID action = ");
        Serial.println(ActionName(pid.GetDirection()));
    }
    else if (strcmp(cmd, "TRACK") == 0 && a1 != nullptr)
    {
        if      (strcmp(a1, "ON") == 0)  pid.SetPVTracking(true);
        else if (strcmp(a1, "OFF") == 0) pid.SetPVTracking(false);
        else { Serial.println("! Use TRACK ON or TRACK OFF."); return; }

        Serial.print("! PV tracking = ");
        Serial.println(pid.GetPVTracking() ? "ON" : "OFF");
    }
    else if (strcmp(cmd, "OUTLIMITS") == 0 && ParseDouble(a1, x) && ParseDouble(a2, y))
    {
        pid.SetOutputLimits(x, y);
        Serial.println("! PID output limits updated.");
    }
    else if (strcmp(cmd, "TS") == 0 && ParseUnsignedLong(a1, ul))
    {
        pid.SetSampleTime(ul);
        Serial.print("! PID sample time [ms] = ");
        Serial.println(pid.GetSampleTime());
    }
    else
    {
        Serial.println("! Unknown or invalid command. Use HELP.");
    }
}

void ReadSerialCommands()
{
    while (Serial.available() > 0)
    {
        const char c = (char)Serial.read();

        // Accept CR, LF or CR+LF. If CR+LF is received, the second
        // terminator sees an empty buffer and is simply ignored.
        if (c == '\r' || c == '\n')
        {
            commandBuffer[commandIndex] = '\0';

            if (commandIndex > 0)
                ProcessCommand(commandBuffer);

            commandIndex = 0;
            continue;
        }

        // Ignore other non-printable control characters.
        if ((unsigned char)c < 32 && c != '\t')
            continue;

        if (commandIndex < sizeof(commandBuffer) - 1)
        {
            commandBuffer[commandIndex++] = c;
        }
        else
        {
            // Discard an overlong command safely.
            commandIndex = 0;
            Serial.println("! Command too long. Buffer cleared.");
        }
    }
}

// ============================================================
// IDENTIFICATION MEMORY
// ============================================================

void PrintIdentificationMemory(const char *selector)
{
    const bool wantStep = (selector == nullptr || strcmp(selector, "STEP") == 0);
    const bool wantRelay = (selector == nullptr || strcmp(selector, "RELAY") == 0);

    if (!wantStep && !wantRelay)
    {
        Serial.println("! Use IDENT, IDENT STEP or IDENT RELAY.");
        return;
    }

    bool printed = false;
    Serial.println("! ===== Identification memory =====");

    if (wantStep)
    {
        const StepTestResult r = stepTest.GetResult();
        if (r.validSteps > 0)
        {
            printed = true;
            Serial.println("! --- Stored StepTest ---");
            Serial.print("! Complete valid model: "); Serial.println(r.valid ? "YES" : "NO (partial/previous state)");
            Serial.print("! Kp [C/%OP]: "); Serial.println(r.Kp, 6);
            if (r.KpNormalizedValid)
            {
                Serial.print("! Kp normalized [%PV/%OP]: "); Serial.println(r.KpNormalized, 6);
            }
            Serial.print("! T0 [s]: "); Serial.println(r.T0, 6);
            Serial.print("! Tp [s]: "); Serial.println(r.Tp, 6);
            Serial.print("! Valid steps: "); Serial.println(r.validSteps);

            if (r.valid)
            {
                const FOPDTModel model = stepTest.GetModel();
                PIDTuning tuning(model);
                const PIDTuningResult pi = tuning.IMC_PI();
                const PIDTuningResult pidResult = tuning.Lambda_PID();
                if (pi.valid)
                {
                    Serial.print("! IMC/Lambda PI TUNE: ");
                    Serial.print(pi.Kc, 6); Serial.print(' ');
                    Serial.print(pi.Ki, 6); Serial.println(" 0");
                }
                if (pidResult.valid)
                {
                    Serial.print("! Lambda PID TUNE: ");
                    Serial.print(pidResult.Kc, 6); Serial.print(' ');
                    Serial.print(pidResult.Ki, 6); Serial.print(' ');
                    Serial.println(pidResult.Kd, 6);
                }
            }
        }
        else
        {
            Serial.println("! StepTest: no stored identification.");
        }
    }

    if (wantRelay)
    {
        const RelayTestResult r = relayTest.GetResult();
        if (r.completedCycles > 0)
        {
            printed = true;
            Serial.println("! --- Stored RelayTest ---");
            Serial.print("! Complete valid result: "); Serial.println(r.valid ? "YES" : "NO (partial/previous state)");
            Serial.print("! Ku: "); Serial.println(r.Ku, 6);
            Serial.print("! Tu [s]: "); Serial.println(r.Tu, 6);
            Serial.print("! PV amplitude: "); Serial.println(r.amplitudePV, 6);
            Serial.print("! Completed cycles: "); Serial.println(r.completedCycles);

            const PIDTuningResult pi = PIDTuning::TyreusLuybenPI(r.Ku, r.Tu);
            const PIDTuningResult pidResult = PIDTuning::TyreusLuybenPID(r.Ku, r.Tu);
            if (pi.valid)
            {
                Serial.print("! Tyreus-Luyben PI TUNE: ");
                Serial.print(pi.Kc, 6); Serial.print(' ');
                Serial.print(pi.Ki, 6); Serial.println(" 0");
            }
            if (pidResult.valid)
            {
                Serial.print("! Tyreus-Luyben PID TUNE: ");
                Serial.print(pidResult.Kc, 6); Serial.print(' ');
                Serial.print(pidResult.Ki, 6); Serial.print(' ');
                Serial.println(pidResult.Kd, 6);
            }
        }
        else
        {
            Serial.println("! RelayTest: no stored identification.");
        }
    }

    if (!printed && selector == nullptr)
        Serial.println("! No identification result is currently stored in RAM.");

    Serial.println("! Results remain available until RESET, board reset/power loss, or a new test of the same type overwrites them.");
    Serial.println("! =================================");
}

// ============================================================
// TEST RESULT REPORTING
// ============================================================

void ReportStepTestStateTransitions()
{
    const StepTestState current = stepTest.GetState();
    if (current == previousStepState)
        return;

    previousStepState = current;

    switch (current)
    {
        case StepTestState::WAITING_STABLE:
            Serial.println("! StepTest: waiting initial stability (band + regression slope).");
            break;

        case StepTestState::RUNNING_STEP:
            Serial.print("! StepTest: step ");
            Serial.print((unsigned int)stepTest.GetCurrentStepIndex() + 1U);
            Serial.print(" applied. OP target=");
            Serial.print(stepTest.GetCurrentTargetOutput(), 2);
            Serial.println(" %.");
            break;

        case StepTestState::WAITING_FINAL_STABLE:
            Serial.print("! StepTest: directional response detected for step ");
            Serial.print((unsigned int)stepTest.GetCurrentStepIndex() + 1U);
            Serial.println(". Waiting final stability.");
            break;

        case StepTestState::FINISHED:
            Serial.println("! StepTest: identification sequence finished.");
            break;

        case StepTestState::ABORTED:
            Serial.print("! StepTest: aborted -> ");
            Serial.println(StepTest::ErrorName(stepTest.GetError()));
            break;

        default:
            break;
    }
}

void ReportStepTest()
{
    if (stepResultReported)
        return;

    if (stepTest.IsFinished())
    {
        const StepTestResult result = stepTest.GetResult();

        Serial.println("! ===== StepTest finished =====");
        Serial.print("! Valid model: "); Serial.println(result.valid ? "YES" : "NO");
        Serial.println("! --- FOPDT model ---");
        Serial.print("! Kp [C/%OP]: "); Serial.println(result.Kp, 6);
        if (result.KpNormalizedValid)
        {
            Serial.print("! Kp normalized [%PV/%OP]: "); Serial.println(result.KpNormalized, 6);
        }
        else
        {
            Serial.println("! Kp normalized: N/A (configure PV/OP engineering ranges)");
        }
        Serial.print("! T0 [s]: "); Serial.println(result.T0, 6);
        Serial.print("! Tp [s]: "); Serial.println(result.Tp, 6);
        Serial.print("! Completed steps: "); Serial.println(result.completedSteps);
        Serial.print("! Valid steps: "); Serial.println(result.validSteps);

        for (uint8_t i = 0; i < result.completedSteps; ++i)
        {
            StepTestStepResult step;
            if (!stepTest.GetStepResult(i, step))
                continue;

            Serial.print("! Step "); Serial.print(i + 1);
            Serial.print(" valid="); Serial.print(step.valid ? "YES" : "NO");
            Serial.print(" OP "); Serial.print(step.initialOP, 2);
            Serial.print("->"); Serial.print(step.finalOP, 2);
            Serial.print(" PV "); Serial.print(step.initialPV, 3);
            Serial.print("->"); Serial.print(step.finalPV, 3);
            Serial.print(" Kp="); Serial.print(step.Kp, 6);
            if (step.KpNormalizedValid)
            {
                Serial.print(" Kp%="); Serial.print(step.KpNormalized, 6);
            }
            Serial.print(" T0="); Serial.print(step.T0, 6);
            Serial.print(" Tp="); Serial.println(step.Tp, 6);
        }

        const FOPDTModel model = stepTest.GetModel();
        PIDTuning tuning(model);
        const PIDTuningResult imc = tuning.IMC_PI();
        const PIDTuningResult lambdaPid = tuning.Lambda_PID();
        if (imc.valid)
        {
            Serial.println("! --- IMC / Lambda PI (PIDControl default) ---");
            Serial.print("! Lambda [s]: "); Serial.print(imc.lambda, 6); Serial.println(" = 2*T0");
            Serial.print("! Kc: "); Serial.println(imc.Kc, 6);
            Serial.print("! Ti [s]: "); Serial.println(imc.Ti, 6);
            Serial.print("! Ki [1/s]: "); Serial.println(imc.Ki, 6);
            Serial.print("! PIDControl command: TUNE ");
            Serial.print(imc.Kc, 6); Serial.print(' ');
            Serial.print(imc.Ki, 6); Serial.println(" 0");
        }
        else
        {
            Serial.println("! IMC / Lambda PI: unavailable for this model.");
        }

        if (lambdaPid.valid)
        {
            Serial.println("! --- Lambda PID (PIDControl default) ---");
            Serial.print("! Tf [s]: "); Serial.print(lambdaPid.lambda, 6); Serial.println(" = 2*T0");
            Serial.print("! Kc: "); Serial.println(lambdaPid.Kc, 6);
            Serial.print("! Ti [s]: "); Serial.println(lambdaPid.Ti, 6);
            Serial.print("! Td [s]: "); Serial.println(lambdaPid.Td, 6);
            Serial.print("! Ki [1/s]: "); Serial.println(lambdaPid.Ki, 6);
            Serial.print("! Kd [s]: "); Serial.println(lambdaPid.Kd, 6);
            Serial.print("! PIDControl command: TUNE ");
            Serial.print(lambdaPid.Kc, 6); Serial.print(' ');
            Serial.print(lambdaPid.Ki, 6); Serial.print(' ');
            Serial.println(lambdaPid.Kd, 6);
        }
        else
        {
            Serial.println("! Lambda PID: unavailable for this model.");
        }

        Serial.println("! PID remains in MAN at the original bias OP.");
        Serial.println("! =============================");
        stepResultReported = true;
    }
    else if (stepTest.IsAborted())
    {
        Serial.print("! StepTest aborted: ");
        Serial.println(StepTest::ErrorName(stepTest.GetError()));
        stepResultReported = true;
    }
}

void ReportRelayTest()
{
    if (relayResultReported)
        return;

    if (relayTest.IsFinished())
    {
        const RelayTestResult result = relayTest.GetResult();

        Serial.println("! ===== RelayTest finished =====");
        Serial.print("! Valid result: "); Serial.println(result.valid ? "YES" : "NO");
        Serial.print("! Ku: "); Serial.println(result.Ku, 6);
        Serial.print("! Tu [s]: "); Serial.println(result.Tu, 6);
        Serial.print("! PV amplitude [C]: "); Serial.println(result.amplitudePV, 6);
        Serial.print("! Relay amplitude [%]: "); Serial.println(result.relayAmplitude, 6);
        Serial.print("! Completed cycles: "); Serial.println(result.completedCycles);

        const PIDTuningResult tlPi = PIDTuning::TyreusLuybenPI(result.Ku, result.Tu);
        const PIDTuningResult tlPid = PIDTuning::TyreusLuybenPID(result.Ku, result.Tu);

        if (tlPi.valid)
        {
            Serial.println("! --- Tyreus-Luyben PI ---");
            Serial.print("! Kc: "); Serial.println(tlPi.Kc, 6);
            Serial.print("! Ti [s]: "); Serial.println(tlPi.Ti, 6);
            Serial.print("! Ki [1/s]: "); Serial.println(tlPi.Ki, 6);
            Serial.print("! PIDControl command: TUNE ");
            Serial.print(tlPi.Kc, 6); Serial.print(' ');
            Serial.print(tlPi.Ki, 6); Serial.println(" 0");
        }

        if (tlPid.valid)
        {
            Serial.println("! --- Tyreus-Luyben PID ---");
            Serial.print("! Kc: "); Serial.println(tlPid.Kc, 6);
            Serial.print("! Ti [s]: "); Serial.println(tlPid.Ti, 6);
            Serial.print("! Td [s]: "); Serial.println(tlPid.Td, 6);
            Serial.print("! Ki [1/s]: "); Serial.println(tlPid.Ki, 6);
            Serial.print("! Kd [s]: "); Serial.println(tlPid.Kd, 6);
            Serial.print("! PIDControl command: TUNE ");
            Serial.print(tlPid.Kc, 6); Serial.print(' ');
            Serial.print(tlPid.Ki, 6); Serial.print(' ');
            Serial.println(tlPid.Kd, 6);
        }

        Serial.println("! PID remains in MAN at the original bias OP.");
        Serial.println("! ==============================");
        relayResultReported = true;
    }
    else if (relayTest.IsAborted())
    {
        Serial.print("! RelayTest aborted. Error=");
        Serial.println((int)relayTest.GetError());
        relayResultReported = true;
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    pinMode(PIN_Q1, OUTPUT);
    pinMode(PIN_Q2, OUTPUT);

    SetHeaterPercent(PIN_Q1, 0.0, Q1_PWM_MAX);
    SetHeaterPercent(PIN_Q2, 0.0, Q2_PWM_MAX);

    Serial.begin(SERIAL_BAUD);

    // TCLab normally provides its analog reference through AREF.
    // UNO R4 uses AR_EXTERNAL for an external reference.
    analogReference(AR_EXTERNAL);
    analogReadResolution(ADC_BITS);

    delay(50);
    UpdateProcessIO(true);
    ResetSignalFilters();
    ApplyDefaultConfiguration(false);

    Serial.println();
    Serial.println("PIDControl 1.4.2 - TCLab / Arduino UNO R4 WiFi test");
    Serial.println("Type HELP for commands.");
    Serial.print("Normal operating SP range: "); Serial.print(OPERATING_SP_MIN_C, 1); Serial.print(".. "); Serial.print(OPERATING_SP_MAX_C, 1); Serial.println(" C.");
    Serial.print("Independent hard temperature trip: "); Serial.print(HARD_TEMP_HIGH_C, 1); Serial.println(" C.");
    Serial.println("Arduino COM Plotter frames start with '#'.");
    PrintStatus();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    ReadSerialCommands();
    UpdateProcessIO();
    CheckHardSafety();

    // Identification classes are non-blocking. They manipulate the
    // PID manual output while running.
    stepTest.Update();
    relayTest.Update();

    // Compute() must also be called in MAN so manual OP and PV tracking
    // remain active.
    pid.Compute();

    // Independent final safety clamp before writing hardware.
    if (PV >= HARD_TEMP_HIGH_C || T2 >= HARD_TEMP_HIGH_C ||
        PVRaw >= HARD_TEMP_RAW_HIGH_C || T2Raw >= HARD_TEMP_RAW_HIGH_C)
        OP = 0.0;

    WriteProcessOutput();

    ReportStepTestStateTransitions();
    ReportStepTest();
    ReportRelayTest();
    SendPlotFrame();
}
