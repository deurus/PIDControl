#include "RelayTest.h"
#include <math.h>
#include <limits.h>

RelayTest::RelayTest(PIDControl &controller, double *input, double *output)
    : pid(controller), Input(input), Output(output),
      minTestOutput(controller.GetOutputMinimum()),
      maxTestOutput(controller.GetOutputMaximum()),
      minPV(-1.0e30), maxPV(1.0e30),
      relayAmplitude(5.0), hysteresis(0.5), requiredCycles(5),
      maxTestTimeMs(10UL * 60UL * 1000UL),
      useCurrentReference(true), referencePV(0.0), biasOutput(0.0),
      highOutput(0.0), lowOutput(0.0), relayHigh(true), processSign(1),
      state(RelayTestState::IDLE), error(RelayTestError::NONE),
      testStartMs(0), previousHighSwitchMs(0), haveCycleStart(false),
      warmupDiscarded(false), cycleMinPV(0.0), cycleMaxPV(0.0),
      sumKu(0.0), sumTu(0.0), sumAmplitudePV(0.0), completedCycles(0)
{
}

void RelayTest::SetOutputLimits(double minimum, double maximum)
{
    if (minimum >= maximum)
        return;

    if (minimum < pid.GetOutputMinimum())
        minimum = pid.GetOutputMinimum();

    if (maximum > pid.GetOutputMaximum())
        maximum = pid.GetOutputMaximum();

    if (minimum < maximum)
    {
        minTestOutput = minimum;
        maxTestOutput = maximum;
    }
}

void RelayTest::SetPVLimits(double minimum, double maximum)
{
    if (minimum < maximum)
    {
        minPV = minimum;
        maxPV = maximum;
    }
}

void RelayTest::SetRelayAmplitude(double value)
{
    if (value > 0.0)
        relayAmplitude = value;
}

void RelayTest::SetHysteresis(double value)
{
    if (value >= 0.0)
        hysteresis = value;
}

void RelayTest::SetCycles(uint8_t cycles)
{
    if (cycles > 0)
        requiredCycles = cycles;
}

void RelayTest::SetReference(double value)
{
    referencePV = value;
    useCurrentReference = false;
}

void RelayTest::UseCurrentPVAsReference()
{
    useCurrentReference = true;
}

void RelayTest::SetMaxTestTime(double minutes)
{
    if (minutes > 0.0)
        maxTestTimeMs = MinutesToMillis(minutes);
}

double RelayTest::GetMaxTestTime() const
{
    return maxTestTimeMs / 60000.0;
}

bool RelayTest::Start(const RelayTestConfig &config)
{
    if (IsRunning())
        return false;

    if (config.amplitude <= 0.0 || config.hysteresis < 0.0 ||
        config.cycles == 0 || config.maxTestTimeMinutes <= 0.0 ||
        config.outputMin >= config.outputMax || config.pvMin >= config.pvMax)
    {
        error = RelayTestError::INVALID_CONFIG;
        state = RelayTestState::ABORTED;
        return false;
    }

    minTestOutput = config.outputMin;
    maxTestOutput = config.outputMax;
    minPV = config.pvMin;
    maxPV = config.pvMax;
    relayAmplitude = config.amplitude;
    hysteresis = config.hysteresis;
    requiredCycles = config.cycles;
    maxTestTimeMs = MinutesToMillis(config.maxTestTimeMinutes);
    useCurrentReference = config.useCurrentReference;
    referencePV = config.referencePV;

    return Start();
}

bool RelayTest::Start()
{
    if (IsRunning())
        return false;

    error = RelayTestError::NONE;
    state = RelayTestState::IDLE;

    if (minTestOutput < pid.GetOutputMinimum())
        minTestOutput = pid.GetOutputMinimum();
    if (maxTestOutput > pid.GetOutputMaximum())
        maxTestOutput = pid.GetOutputMaximum();

    if (Input == 0 || Output == 0 || relayAmplitude <= 0.0 ||
        requiredCycles == 0 || minTestOutput >= maxTestOutput || minPV >= maxPV)
    {
        error = RelayTestError::INVALID_CONFIG;
        state = RelayTestState::ABORTED;
        return false;
    }

    biasOutput = *Output;

    if (*Input < minPV)
    {
        error = RelayTestError::PV_LOW;
        state = RelayTestState::ABORTED;
        return false;
    }

    if (*Input > maxPV)
    {
        error = RelayTestError::PV_HIGH;
        state = RelayTestState::ABORTED;
        return false;
    }

    highOutput = biasOutput + relayAmplitude;
    lowOutput = biasOutput - relayAmplitude;

    if (biasOutput < minTestOutput || biasOutput > maxTestOutput ||
        highOutput > maxTestOutput || lowOutput < minTestOutput)
    {
        AbortWithError(RelayTestError::OP_LIMIT);
        return false;
    }

    if (useCurrentReference)
        referencePV = *Input;

    if (referencePV < minPV || referencePV > maxPV)
    {
        AbortWithError(RelayTestError::INVALID_CONFIG);
        return false;
    }

    // With the PIDControl convention, REVERSE normally corresponds to a
    // positive process gain (higher OP -> higher PV), DIRECT to a negative one.
    processSign = (pid.GetDirection() == PIDAction::REVERSE) ? 1 : -1;

    pid.SetMode(PIDMode::MAN);

    relayHigh = true;
    pid.SetManualOutput(highOutput);

    const unsigned long now = millis();
    testStartMs = now;
    previousHighSwitchMs = 0;
    haveCycleStart = false;
    warmupDiscarded = false;
    completedCycles = 0;
    sumKu = 0.0;
    sumTu = 0.0;
    sumAmplitudePV = 0.0;
    cycleMinPV = *Input;
    cycleMaxPV = *Input;

    state = RelayTestState::RUNNING;
    return true;
}

void RelayTest::Update()
{
    if (!IsRunning())
        return;

    const unsigned long now = millis();

    if ((unsigned long)(now - testStartMs) >= maxTestTimeMs)
    {
        AbortWithError(completedCycles == 0
            ? RelayTestError::NO_OSCILLATION
            : RelayTestError::TIMEOUT);
        return;
    }

    if (!CheckSafety())
        return;

    if (*Input < cycleMinPV) cycleMinPV = *Input;
    if (*Input > cycleMaxPV) cycleMaxPV = *Input;

    if (processSign > 0)
    {
        if (relayHigh && *Input >= referencePV + hysteresis)
            SwitchLow();
        else if (!relayHigh && *Input <= referencePV - hysteresis)
            SwitchHigh(now);
    }
    else
    {
        if (relayHigh && *Input <= referencePV - hysteresis)
            SwitchLow();
        else if (!relayHigh && *Input >= referencePV + hysteresis)
            SwitchHigh(now);
    }
}

void RelayTest::Abort()
{
    if (IsRunning())
        AbortWithError(RelayTestError::USER_ABORT);
}

bool RelayTest::IsRunning() const
{
    return state == RelayTestState::RUNNING;
}

bool RelayTest::IsFinished() const
{
    return state == RelayTestState::FINISHED;
}

bool RelayTest::IsAborted() const
{
    return state == RelayTestState::ABORTED;
}

RelayTestState RelayTest::GetState() const
{
    return state;
}

RelayTestError RelayTest::GetError() const
{
    return error;
}

RelayTestResult RelayTest::GetResult() const
{
    RelayTestResult result;
    result.completedCycles = completedCycles;
    result.relayAmplitude = relayAmplitude;
    result.valid = (state == RelayTestState::FINISHED && completedCycles > 0);

    if (completedCycles > 0)
    {
        result.Ku = sumKu / completedCycles;
        result.Tu = sumTu / completedCycles;
        result.amplitudePV = sumAmplitudePV / completedCycles;
    }
    else
    {
        result.Ku = 0.0;
        result.Tu = 0.0;
        result.amplitudePV = 0.0;
    }

    return result;
}

unsigned long RelayTest::MinutesToMillis(double minutes)
{
    const double ms = minutes * 60000.0;
    if (ms >= (double)ULONG_MAX)
        return ULONG_MAX;

    if (ms <= 1.0)
        return 1UL;

    return (unsigned long)ms;
}

bool RelayTest::CheckSafety()
{
    if (*Input < minPV)
    {
        AbortWithError(RelayTestError::PV_LOW);
        return false;
    }

    if (*Input > maxPV)
    {
        AbortWithError(RelayTestError::PV_HIGH);
        return false;
    }

    if (*Output < minTestOutput || *Output > maxTestOutput)
    {
        AbortWithError(RelayTestError::OP_LIMIT);
        return false;
    }

    return true;
}

void RelayTest::SwitchHigh(unsigned long now)
{
    if (haveCycleStart)
    {
        CompleteCycle(now);
        if (state != RelayTestState::RUNNING)
            return;
    }
    else
    {
        previousHighSwitchMs = now;
        haveCycleStart = true;
    }

    relayHigh = true;
    pid.SetManualOutput(highOutput);
    cycleMinPV = *Input;
    cycleMaxPV = *Input;
}

void RelayTest::SwitchLow()
{
    relayHigh = false;
    pid.SetManualOutput(lowOutput);
}

void RelayTest::CompleteCycle(unsigned long now)
{
    const double amplitude = (cycleMaxPV - cycleMinPV) / 2.0;
    const double periodSeconds = (now - previousHighSwitchMs) / 1000.0;
    previousHighSwitchMs = now;

    if (amplitude <= 1.0e-12 || periodSeconds <= 0.0)
        return;

    // Discard the first complete cycle to reduce startup-transient influence.
    if (!warmupDiscarded)
    {
        warmupDiscarded = true;
        return;
    }

    const double pi = 3.14159265358979323846;
    const double ku = (4.0 * relayAmplitude) / (pi * amplitude);

    sumKu += ku;
    sumTu += periodSeconds;
    sumAmplitudePV += amplitude;
    ++completedCycles;

    if (completedCycles >= requiredCycles)
        FinishTest();
}

void RelayTest::FinishTest()
{
    pid.SetMode(PIDMode::MAN);
    pid.SetManualOutput(biasOutput);
    state = RelayTestState::FINISHED;
    error = RelayTestError::NONE;
}

void RelayTest::AbortWithError(RelayTestError reason)
{
    pid.SetMode(PIDMode::MAN);
    pid.SetManualOutput(biasOutput);
    error = reason;
    state = RelayTestState::ABORTED;
}
