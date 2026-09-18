#include "StepTest.h"
#include <math.h>
#include <limits.h>

StepTest::StepTest(PIDControl &controller, double *input, double *output)
    : pid(controller), Input(input), Output(output),
      minTestOutput(controller.GetOutputMinimum()),
      maxTestOutput(controller.GetOutputMaximum()),
      minPV(-1.0e30), maxPV(1.0e30),
      outputRangeMin(0.0), outputRangeMax(0.0),
      pvRangeMin(0.0), pvRangeMax(0.0),
      outputRangeConfigured(false), pvRangeConfigured(false),
      stepSize(5.0), numberOfSteps(2), firstStepUp(true),
      processGain(ProcessGain::POSITIVE),
      stabilityBand(0.2), stabilityTimeMs(10000UL),
      maxStabilitySlopePerMinute(0.10),
      responseThreshold(0.0), maxTestTimeMs(10UL * 60UL * 1000UL),
      state(StepTestState::IDLE), error(StepTestError::NONE),
      testStartMs(0), stepStartMs(0), stabilityStartMs(0),
      stabilityMinPV(0.0), stabilityMaxPV(0.0), stabilitySumPV(0.0),
      stabilitySumT(0.0), stabilitySumTT(0.0), stabilitySumTPV(0.0), stabilityCount(0),
      biasOutput(0.0), stepInitialOP(0.0), stepInitialPV(0.0),
      currentTargetOP(0.0), responseDetected(false),
      currentStep(0), sumKp(0.0), sumT0(0.0), sumTp(0.0), validSteps(0),
      sampleCount(0), sampleIntervalMs(100UL), lastSampleElapsedMs(0)
{
    for (uint8_t i = 0; i < PIDCONTROL_STEPTEST_MAX_STEPS; ++i)
    {
        stepResults[i].valid = false;
        stepResults[i].initialOP = 0.0;
        stepResults[i].finalOP = 0.0;
        stepResults[i].initialPV = 0.0;
        stepResults[i].finalPV = 0.0;
        stepResults[i].Kp = 0.0;
        stepResults[i].KpNormalizedValid = false;
        stepResults[i].KpNormalized = 0.0;
        stepResults[i].T0 = 0.0;
        stepResults[i].Tp = 0.0;
    }
}

void StepTest::SetOutputLimits(double minimum, double maximum)
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

void StepTest::SetPVLimits(double minimum, double maximum)
{
    if (minimum < maximum)
    {
        minPV = minimum;
        maxPV = maximum;
    }
}


void StepTest::SetOutputRange(double minimum, double maximum)
{
    if (minimum < maximum)
    {
        outputRangeMin = minimum;
        outputRangeMax = maximum;
        outputRangeConfigured = true;
    }
}

void StepTest::SetPVRange(double minimum, double maximum)
{
    if (minimum < maximum)
    {
        pvRangeMin = minimum;
        pvRangeMax = maximum;
        pvRangeConfigured = true;
    }
}

bool StepTest::HasEngineeringRanges() const
{
    return outputRangeConfigured && pvRangeConfigured &&
           outputRangeMax > outputRangeMin && pvRangeMax > pvRangeMin;
}

double StepTest::GetOutputRangeMinimum() const
{
    return outputRangeMin;
}

double StepTest::GetOutputRangeMaximum() const
{
    return outputRangeMax;
}

double StepTest::GetPVRangeMinimum() const
{
    return pvRangeMin;
}

double StepTest::GetPVRangeMaximum() const
{
    return pvRangeMax;
}

void StepTest::SetStepSize(double value)
{
    if (value > 0.0)
        stepSize = value;
}

void StepTest::SetNumberOfSteps(uint8_t steps)
{
    if (steps == 0)
        return;

    numberOfSteps = (steps > PIDCONTROL_STEPTEST_MAX_STEPS)
        ? PIDCONTROL_STEPTEST_MAX_STEPS
        : steps;
}

void StepTest::SetFirstStepUp(bool up)
{
    firstStepUp = up;
}

void StepTest::SetProcessGain(ProcessGain gain)
{
    processGain = gain;
}

void StepTest::SetStabilityCriteria(double band, double timeSeconds)
{
    if (band > 0.0)
        stabilityBand = band;

    if (timeSeconds > 0.0)
    {
        const double ms = timeSeconds * 1000.0;
        stabilityTimeMs = (ms >= (double)ULONG_MAX) ? ULONG_MAX : (unsigned long)ms;
    }
}

void StepTest::SetStabilityCriteria(double band, double timeSeconds, double maxSlopePerMinute)
{
    SetStabilityCriteria(band, timeSeconds);
    SetMaxStabilitySlope(maxSlopePerMinute);
}

void StepTest::SetMaxStabilitySlope(double maxSlopePerMinute)
{
    if (maxSlopePerMinute > 0.0)
        maxStabilitySlopePerMinute = maxSlopePerMinute;
}

void StepTest::SetResponseThreshold(double value)
{
    if (value >= 0.0)
        responseThreshold = value;
}

void StepTest::SetMaxTestTime(double minutes)
{
    if (minutes > 0.0)
        maxTestTimeMs = MinutesToMillis(minutes);
}

double StepTest::GetMaxTestTime() const
{
    return maxTestTimeMs / 60000.0;
}

ProcessGain StepTest::GetProcessGain() const
{
    return processGain;
}

double StepTest::GetStabilityBand() const
{
    return stabilityBand;
}

double StepTest::GetStabilityTime() const
{
    return stabilityTimeMs / 1000.0;
}

double StepTest::GetMaxStabilitySlope() const
{
    return maxStabilitySlopePerMinute;
}

double StepTest::GetResponseThreshold() const
{
    return responseThreshold;
}

double StepTest::GetCurrentStabilitySlope() const
{
    return CalculateStabilitySlope();
}

double StepTest::GetBiasOutput() const
{
    return biasOutput;
}

double StepTest::GetCurrentTargetOutput() const
{
    return currentTargetOP;
}

uint8_t StepTest::GetCurrentStepIndex() const
{
    return currentStep;
}

bool StepTest::Start()
{
    if (IsRunning())
        return false;

    error = StepTestError::NONE;
    state = StepTestState::IDLE;

    if (minTestOutput < pid.GetOutputMinimum())
        minTestOutput = pid.GetOutputMinimum();
    if (maxTestOutput > pid.GetOutputMaximum())
        maxTestOutput = pid.GetOutputMaximum();

    if (Input == 0 || Output == 0 || numberOfSteps == 0 || stepSize <= 0.0 ||
        stabilityBand <= 0.0 || stabilityTimeMs == 0 || maxStabilitySlopePerMinute <= 0.0 ||
        minTestOutput >= maxTestOutput || minPV >= maxPV)
    {
        error = StepTestError::INVALID_CONFIG;
        state = StepTestState::ABORTED;
        return false;
    }

    biasOutput = *Output;

    if (*Input < minPV)
    {
        error = StepTestError::PV_LOW;
        state = StepTestState::ABORTED;
        return false;
    }

    if (*Input > maxPV)
    {
        error = StepTestError::PV_HIGH;
        state = StepTestState::ABORTED;
        return false;
    }

    if (biasOutput < minTestOutput || biasOutput > maxTestOutput)
    {
        error = StepTestError::OP_LIMIT;
        state = StepTestState::ABORTED;
        return false;
    }

    pid.SetMode(PIDMode::MAN);
    pid.SetManualOutput(biasOutput);

    currentStep = 0;
    validSteps = 0;
    sumKp = 0.0;
    sumT0 = 0.0;
    sumTp = 0.0;

    for (uint8_t i = 0; i < PIDCONTROL_STEPTEST_MAX_STEPS; ++i)
        stepResults[i].valid = false;

    const unsigned long now = millis();
    testStartMs = now;
    ResetStability(now);
    state = StepTestState::WAITING_STABLE;
    return true;
}

void StepTest::Update()
{
    if (!IsRunning())
        return;

    const unsigned long now = millis();

    if ((unsigned long)(now - testStartMs) >= maxTestTimeMs)
    {
        const StepTestError timeoutError =
            (state == StepTestState::RUNNING_STEP && !responseDetected)
                ? StepTestError::NO_RESPONSE
                : StepTestError::TIMEOUT;
        AbortWithError(timeoutError);
        return;
    }

    if (!CheckSafety())
        return;

    switch (state)
    {
        case StepTestState::WAITING_STABLE:
            pid.SetManualOutput(biasOutput);
            if (UpdateStability(now))
            {
                if (!ApplyCurrentStep(now))
                    return;
            }
            break;

        case StepTestState::RUNNING_STEP:
        {
            pid.SetManualOutput(currentTargetOP);
            RecordSample(now);

            const double threshold = (responseThreshold > 0.0)
                ? responseThreshold
                : (2.0 * stabilityBand);

            const double deltaPV = *Input - stepInitialPV;
            const int expectedDirection = ExpectedPVDirection();

            // The response is only accepted in the physically expected direction.
            // This prevents residual thermal inertia after a downward OP step from
            // being mistaken for the response to that downward step.
            if ((double)expectedDirection * deltaPV >= threshold)
            {
                responseDetected = true;
                ResetStability(now);
                state = StepTestState::WAITING_FINAL_STABLE;
            }
            break;
        }

        case StepTestState::WAITING_FINAL_STABLE:
            pid.SetManualOutput(currentTargetOP);
            RecordSample(now);

            if (UpdateStability(now))
                CompleteCurrentStep(now);
            break;

        default:
            break;
    }
}

void StepTest::Abort()
{
    if (IsRunning())
        AbortWithError(StepTestError::USER_ABORT);
}

bool StepTest::IsRunning() const
{
    return state == StepTestState::WAITING_STABLE ||
           state == StepTestState::RUNNING_STEP ||
           state == StepTestState::WAITING_FINAL_STABLE;
}

bool StepTest::IsFinished() const
{
    return state == StepTestState::FINISHED;
}

bool StepTest::IsAborted() const
{
    return state == StepTestState::ABORTED;
}

StepTestState StepTest::GetState() const
{
    return state;
}

StepTestError StepTest::GetError() const
{
    return error;
}

StepTestResult StepTest::GetResult() const
{
    StepTestResult result;
    result.completedSteps = currentStep;
    result.validSteps = validSteps;
    result.valid = (state == StepTestState::FINISHED && validSteps > 0);
    result.KpNormalizedValid = false;
    result.KpNormalized = 0.0;

    if (validSteps > 0)
    {
        result.Kp = sumKp / validSteps;
        result.T0 = sumT0 / validSteps;
        result.Tp = sumTp / validSteps;

        if (HasEngineeringRanges())
        {
            const double pvSpan = pvRangeMax - pvRangeMin;
            const double opSpan = outputRangeMax - outputRangeMin;
            result.KpNormalized = result.Kp * opSpan / pvSpan;
            result.KpNormalizedValid = true;
        }
    }
    else
    {
        result.Kp = 0.0;
        result.T0 = 0.0;
        result.Tp = 0.0;
    }

    return result;
}

FOPDTModel StepTest::GetModel() const
{
    const StepTestResult result = GetResult();
    FOPDTModel model;
    model.valid = result.valid;
    model.Kp = result.Kp;
    model.KpNormalizedValid = result.KpNormalizedValid;
    model.KpNormalized = result.KpNormalized;
    model.T0 = result.T0;
    model.Tp = result.Tp;
    model.pvRangeMin = pvRangeMin;
    model.pvRangeMax = pvRangeMax;
    model.outputRangeMin = outputRangeMin;
    model.outputRangeMax = outputRangeMax;
    return model;
}

bool StepTest::GetStepResult(uint8_t index, StepTestStepResult &result) const
{
    if (index >= currentStep || index >= PIDCONTROL_STEPTEST_MAX_STEPS)
        return false;

    result = stepResults[index];
    return true;
}

const char *StepTest::StateName(StepTestState value)
{
    switch (value)
    {
        case StepTestState::IDLE:                 return "IDLE";
        case StepTestState::WAITING_STABLE:       return "WAITING_STABLE";
        case StepTestState::RUNNING_STEP:         return "RUNNING_STEP";
        case StepTestState::WAITING_FINAL_STABLE: return "WAITING_FINAL_STABLE";
        case StepTestState::FINISHED:             return "FINISHED";
        case StepTestState::ABORTED:              return "ABORTED";
        default:                                   return "UNKNOWN";
    }
}

const char *StepTest::ErrorName(StepTestError value)
{
    switch (value)
    {
        case StepTestError::NONE:           return "NONE";
        case StepTestError::INVALID_CONFIG: return "INVALID_CONFIG";
        case StepTestError::PV_LOW:         return "PV_LOW";
        case StepTestError::PV_HIGH:        return "PV_HIGH";
        case StepTestError::OP_LIMIT:        return "OP_LIMIT";
        case StepTestError::TIMEOUT:         return "TIMEOUT";
        case StepTestError::NO_RESPONSE:     return "NO_RESPONSE";
        case StepTestError::INVALID_MODEL:   return "INVALID_MODEL";
        case StepTestError::USER_ABORT:      return "USER_ABORT";
        default:                             return "UNKNOWN";
    }
}

const char *StepTest::ProcessGainName(ProcessGain value)
{
    return value == ProcessGain::NEGATIVE ? "NEGATIVE" : "POSITIVE";
}

unsigned long StepTest::MinutesToMillis(double minutes)
{
    const double ms = minutes * 60000.0;
    if (ms >= (double)ULONG_MAX)
        return ULONG_MAX;

    if (ms <= 1.0)
        return 1UL;

    return (unsigned long)ms;
}

bool StepTest::CheckSafety()
{
    if (*Input < minPV)
    {
        AbortWithError(StepTestError::PV_LOW);
        return false;
    }

    if (*Input > maxPV)
    {
        AbortWithError(StepTestError::PV_HIGH);
        return false;
    }

    if (*Output < minTestOutput || *Output > maxTestOutput)
    {
        AbortWithError(StepTestError::OP_LIMIT);
        return false;
    }

    return true;
}

void StepTest::ResetStabilityAccumulators()
{
    stabilityMinPV = 0.0;
    stabilityMaxPV = 0.0;
    stabilitySumPV = 0.0;
    stabilitySumT = 0.0;
    stabilitySumTT = 0.0;
    stabilitySumTPV = 0.0;
    stabilityCount = 0;
}

void StepTest::ResetStability(unsigned long now)
{
    stabilityStartMs = now;
    ResetStabilityAccumulators();
    AddStabilitySample(now);
}

void StepTest::AddStabilitySample(unsigned long now)
{
    const double y = *Input;
    const double tMinutes = (now - stabilityStartMs) / 60000.0;

    if (stabilityCount == 0)
    {
        stabilityMinPV = y;
        stabilityMaxPV = y;
    }
    else
    {
        if (y < stabilityMinPV) stabilityMinPV = y;
        if (y > stabilityMaxPV) stabilityMaxPV = y;
    }

    stabilitySumPV += y;
    stabilitySumT += tMinutes;
    stabilitySumTT += tMinutes * tMinutes;
    stabilitySumTPV += tMinutes * y;
    ++stabilityCount;
}

bool StepTest::UpdateStability(unsigned long now)
{
    AddStabilitySample(now);

    // Range criterion: a stable window must remain inside a total width of
    // 2*band. Unlike the old anchor-only criterion, this tracks the entire
    // window and rejects wide excursions.
    if ((stabilityMaxPV - stabilityMinPV) > (2.0 * stabilityBand))
    {
        ResetStability(now);
        return false;
    }

    if ((unsigned long)(now - stabilityStartMs) < stabilityTimeMs)
        return false;

    // Trend criterion: even a smooth slow ramp can remain inside a narrow
    // band for some time. Linear-regression slope rejects that false steady
    // state. Slope unit is PV engineering-units per minute.
    const double slope = CalculateStabilitySlope();
    if (fabs(slope) > maxStabilitySlopePerMinute)
    {
        ResetStability(now);
        return false;
    }

    return true;
}

double StepTest::CalculateStabilitySlope() const
{
    if (stabilityCount < 2)
        return 0.0;

    const double n = (double)stabilityCount;
    const double denominator = n * stabilitySumTT - stabilitySumT * stabilitySumT;

    if (fabs(denominator) < 1.0e-18)
        return 0.0;

    return (n * stabilitySumTPV - stabilitySumT * stabilitySumPV) / denominator;
}

double StepTest::GetStableMean() const
{
    return (stabilityCount > 0) ? stabilitySumPV / (double)stabilityCount : *Input;
}

double StepTest::TargetForStep(uint8_t index) const
{
    if ((index & 1U) != 0U)
        return biasOutput;

    const uint8_t excursion = index / 2U;
    const bool up = ((excursion & 1U) == 0U) ? firstStepUp : !firstStepUp;
    return biasOutput + (up ? stepSize : -stepSize);
}

int StepTest::ExpectedPVDirection() const
{
    const double deltaOP = currentTargetOP - stepInitialOP;
    const int opDirection = (deltaOP >= 0.0) ? 1 : -1;
    const int gainDirection = (processGain == ProcessGain::POSITIVE) ? 1 : -1;
    return opDirection * gainDirection;
}

bool StepTest::ApplyCurrentStep(unsigned long now)
{
    if (currentStep >= numberOfSteps)
    {
        FinishTest();
        return false;
    }

    stepInitialOP = *Output;
    stepInitialPV = GetStableMean();
    currentTargetOP = TargetForStep(currentStep);

    if (currentTargetOP < minTestOutput || currentTargetOP > maxTestOutput)
    {
        AbortWithError(StepTestError::OP_LIMIT);
        return false;
    }

    if (fabs(currentTargetOP - stepInitialOP) < 1.0e-12)
    {
        AbortWithError(StepTestError::INVALID_CONFIG);
        return false;
    }

    responseDetected = false;
    stepStartMs = now;
    ResetSamples();
    RecordSample(now, true);
    pid.SetManualOutput(currentTargetOP);
    state = StepTestState::RUNNING_STEP;
    return true;
}

void StepTest::CompleteCurrentStep(unsigned long now)
{
    RecordSample(now, true);
    const double finalPV = GetStableMean();
    const StepTestStepResult result = CalculateStepResult(finalPV);

    stepResults[currentStep] = result;

    if (result.valid)
    {
        sumKp += result.Kp;
        sumT0 += result.T0;
        sumTp += result.Tp;
        ++validSteps;
    }

    ++currentStep;

    if (currentStep >= numberOfSteps)
    {
        FinishTest();
        return;
    }

    // The final stable PV of the previous step is also the initial stable PV
    // of the next step. Do NOT reset the stability accumulator before
    // ApplyCurrentStep(): that function reads GetStableMean() to preserve the
    // previous stationary mean as stepInitialPV. The accumulator will be reset
    // later when the directional response of the new step is detected.
    if (!ApplyCurrentStep(now))
        return;
}

void StepTest::FinishTest()
{
    pid.SetMode(PIDMode::MAN);
    pid.SetManualOutput(biasOutput);

    if (validSteps == 0)
    {
        state = StepTestState::ABORTED;
        error = responseDetected ? StepTestError::INVALID_MODEL : StepTestError::NO_RESPONSE;
        return;
    }

    state = StepTestState::FINISHED;
    error = StepTestError::NONE;
}

void StepTest::AbortWithError(StepTestError reason)
{
    pid.SetMode(PIDMode::MAN);
    pid.SetManualOutput(biasOutput);
    error = reason;
    state = StepTestState::ABORTED;
}

void StepTest::ResetSamples()
{
    sampleCount = 0;
    sampleIntervalMs = pid.GetSampleTime();
    if (sampleIntervalMs < 50UL)
        sampleIntervalMs = 50UL;
    lastSampleElapsedMs = 0;
}

void StepTest::RecordSample(unsigned long now, bool force)
{
    const unsigned long elapsed = now - stepStartMs;

    if (!force && sampleCount > 0 &&
        (unsigned long)(elapsed - lastSampleElapsedMs) < sampleIntervalMs)
        return;

    if (sampleCount >= PIDCONTROL_STEPTEST_SAMPLES)
        CompressSamples();

    if (sampleCount < PIDCONTROL_STEPTEST_SAMPLES)
    {
        samplePV[sampleCount] = (float)(*Input);
        sampleTimeMs[sampleCount] = elapsed;
        ++sampleCount;
        lastSampleElapsedMs = elapsed;
    }
}

void StepTest::CompressSamples()
{
    if (sampleCount < 3)
        return;

    uint8_t write = 0;
    for (uint8_t read = 0; read < sampleCount; read += 2)
    {
        samplePV[write] = samplePV[read];
        sampleTimeMs[write] = sampleTimeMs[read];
        ++write;
    }

    sampleCount = write;
    if (sampleIntervalMs <= ULONG_MAX / 2UL)
        sampleIntervalMs *= 2UL;

    lastSampleElapsedMs = sampleTimeMs[sampleCount - 1];
}

bool StepTest::FindCrossingTime(double targetPV, bool increasing, double &timeSeconds) const
{
    if (sampleCount < 2)
        return false;

    for (uint8_t i = 1; i < sampleCount; ++i)
    {
        const double y0 = samplePV[i - 1];
        const double y1 = samplePV[i];
        const bool crossed = increasing
            ? (y0 <= targetPV && y1 >= targetPV)
            : (y0 >= targetPV && y1 <= targetPV);

        if (!crossed)
            continue;

        const double dy = y1 - y0;
        double fraction = 0.0;
        if (fabs(dy) > 1.0e-12)
            fraction = (targetPV - y0) / dy;

        const double t0 = sampleTimeMs[i - 1];
        const double t1 = sampleTimeMs[i];
        timeSeconds = (t0 + fraction * (t1 - t0)) / 1000.0;
        return true;
    }

    return false;
}

StepTestStepResult StepTest::CalculateStepResult(double finalPV) const
{
    StepTestStepResult result;
    result.valid = false;
    result.initialOP = stepInitialOP;
    result.finalOP = currentTargetOP;
    result.initialPV = stepInitialPV;
    result.finalPV = finalPV;
    result.Kp = 0.0;
    result.KpNormalizedValid = false;
    result.KpNormalized = 0.0;
    result.T0 = 0.0;
    result.Tp = 0.0;

    const double deltaOP = result.finalOP - result.initialOP;
    const double deltaPV = result.finalPV - result.initialPV;

    if (fabs(deltaOP) < 1.0e-12 || fabs(deltaPV) < 1.0e-12)
        return result;

    // Reject a final response whose sign contradicts the configured process gain.
    const int expectedDirection = ExpectedPVDirection();
    if ((double)expectedDirection * deltaPV <= 0.0)
        return result;

    result.Kp = deltaPV / deltaOP;

    if (HasEngineeringRanges())
    {
        const double pvSpan = pvRangeMax - pvRangeMin;
        const double opSpan = outputRangeMax - outputRangeMin;
        result.KpNormalized = result.Kp * opSpan / pvSpan;
        result.KpNormalizedValid = true;
    }

    const bool increasing = deltaPV > 0.0;
    const double pv283 = result.initialPV + 0.283 * deltaPV;
    const double pv632 = result.initialPV + 0.632 * deltaPV;

    double t283 = 0.0;
    double t632 = 0.0;

    if (!FindCrossingTime(pv283, increasing, t283) ||
        !FindCrossingTime(pv632, increasing, t632) ||
        t632 <= t283)
        return result;

    result.Tp = 1.5 * (t632 - t283);
    result.T0 = 1.5 * t283 - 0.5 * t632;

    if (result.T0 < 0.0)
        result.T0 = 0.0;

    result.valid = (result.Tp > 0.0);
    return result;
}
