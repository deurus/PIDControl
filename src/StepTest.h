#ifndef STEPTEST_H
#define STEPTEST_H

#include <Arduino.h>
#include "PIDControl.h"

#ifndef PIDCONTROL_STEPTEST_MAX_STEPS
#define PIDCONTROL_STEPTEST_MAX_STEPS 10
#endif

#ifndef PIDCONTROL_STEPTEST_SAMPLES
#define PIDCONTROL_STEPTEST_SAMPLES 64
#endif

enum class StepTestState
{
    IDLE,
    WAITING_STABLE,
    RUNNING_STEP,
    WAITING_FINAL_STABLE,
    FINISHED,
    ABORTED
};

enum class StepTestError
{
    NONE,
    INVALID_CONFIG,
    PV_LOW,
    PV_HIGH,
    OP_LIMIT,
    TIMEOUT,
    NO_RESPONSE,
    INVALID_MODEL,
    USER_ABORT
};

enum class ProcessGain
{
    POSITIVE,
    NEGATIVE
};

struct StepTestStepResult
{
    bool valid;
    double initialOP;
    double finalOP;
    double initialPV;
    double finalPV;
    double Kp;
    bool KpNormalizedValid;
    double KpNormalized;
    double T0;
    double Tp;
};

struct FOPDTModel
{
    bool valid;
    double Kp;                 // Engineering-unit gain: DeltaPV / DeltaOP
    bool KpNormalizedValid;
    double KpNormalized;       // Normalized gain: %PV / %OP
    double T0;                 // Dead time [s]
    double Tp;                 // Process time constant [s]
    double pvRangeMin;
    double pvRangeMax;
    double outputRangeMin;
    double outputRangeMax;
};

struct StepTestResult
{
    bool valid;
    double Kp;
    bool KpNormalizedValid;
    double KpNormalized;
    double T0;
    double Tp;
    uint8_t completedSteps;
    uint8_t validSteps;
};

class StepTest
{
public:
    StepTest(PIDControl &controller, double *input, double *output);

    // Safety limits used only while the test is running.
    void SetOutputLimits(double minimum, double maximum);
    void SetPVLimits(double minimum, double maximum);

    // Full engineering ranges used for normalized process gain (%PV/%OP).
    // They are independent from the safety limits above.
    void SetOutputRange(double minimum, double maximum);
    void SetPVRange(double minimum, double maximum);
    bool HasEngineeringRanges() const;
    double GetOutputRangeMinimum() const;
    double GetOutputRangeMaximum() const;
    double GetPVRangeMinimum() const;
    double GetPVRangeMaximum() const;
    void SetStepSize(double value);
    void SetNumberOfSteps(uint8_t steps);
    void SetFirstStepUp(bool up);
    void SetProcessGain(ProcessGain gain);

    // Backward-compatible overload: preserves the current slope criterion.
    void SetStabilityCriteria(double band, double timeSeconds);
    void SetStabilityCriteria(double band, double timeSeconds, double maxSlopePerMinute);
    void SetMaxStabilitySlope(double maxSlopePerMinute);
    void SetResponseThreshold(double value);

    // User-facing unit: minutes. Internally stored as milliseconds.
    void SetMaxTestTime(double minutes);
    double GetMaxTestTime() const;

    ProcessGain GetProcessGain() const;
    double GetStabilityBand() const;
    double GetStabilityTime() const;
    double GetMaxStabilitySlope() const;
    double GetResponseThreshold() const;
    double GetCurrentStabilitySlope() const;
    double GetBiasOutput() const;
    double GetCurrentTargetOutput() const;
    uint8_t GetCurrentStepIndex() const;

    bool Start();
    void Update();
    void Abort();

    // Runtime reset. Configuration values are preserved.
    void Reset()
    {
        if (IsRunning())
        {
            pid.SetMode(PIDMode::MAN);
            pid.SetManualOutput(biasOutput);
        }

        state = StepTestState::IDLE;
        error = StepTestError::NONE;
        testStartMs = 0;
        stepStartMs = 0;
        stabilityStartMs = 0;
        responseDetected = false;
        currentStep = 0;
        validSteps = 0;
        sumKp = 0.0;
        sumT0 = 0.0;
        sumTp = 0.0;
        sampleCount = 0;
        lastSampleElapsedMs = 0;

        ResetStabilityAccumulators();

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

    bool IsRunning() const;
    bool IsFinished() const;
    bool IsAborted() const;

    StepTestState GetState() const;
    StepTestError GetError() const;
    StepTestResult GetResult() const;
    FOPDTModel GetModel() const;
    bool GetStepResult(uint8_t index, StepTestStepResult &result) const;

    static const char *StateName(StepTestState value);
    static const char *ErrorName(StepTestError value);
    static const char *ProcessGainName(ProcessGain value);

private:
    PIDControl &pid;
    double *Input;
    double *Output;

    double minTestOutput;
    double maxTestOutput;
    double minPV;
    double maxPV;

    double outputRangeMin;
    double outputRangeMax;
    double pvRangeMin;
    double pvRangeMax;
    bool outputRangeConfigured;
    bool pvRangeConfigured;
    double stepSize;
    uint8_t numberOfSteps;
    bool firstStepUp;
    ProcessGain processGain;

    double stabilityBand;
    unsigned long stabilityTimeMs;
    double maxStabilitySlopePerMinute;
    double responseThreshold;
    unsigned long maxTestTimeMs;

    StepTestState state;
    StepTestError error;

    unsigned long testStartMs;
    unsigned long stepStartMs;
    unsigned long stabilityStartMs;

    // Stability statistics for linear regression y = a + b*t.
    // t is expressed in minutes from stabilityStartMs, therefore b is C/min
    // (or PV engineering-units/minute).
    double stabilityMinPV;
    double stabilityMaxPV;
    double stabilitySumPV;
    double stabilitySumT;
    double stabilitySumTT;
    double stabilitySumTPV;
    uint32_t stabilityCount;

    double biasOutput;
    double stepInitialOP;
    double stepInitialPV;
    double currentTargetOP;
    bool responseDetected;

    uint8_t currentStep;
    StepTestStepResult stepResults[PIDCONTROL_STEPTEST_MAX_STEPS];
    double sumKp;
    double sumT0;
    double sumTp;
    uint8_t validSteps;

    float samplePV[PIDCONTROL_STEPTEST_SAMPLES];
    unsigned long sampleTimeMs[PIDCONTROL_STEPTEST_SAMPLES];
    uint8_t sampleCount;
    unsigned long sampleIntervalMs;
    unsigned long lastSampleElapsedMs;

    static unsigned long MinutesToMillis(double minutes);
    bool CheckSafety();
    void ResetStability(unsigned long now);
    void ResetStabilityAccumulators();
    void AddStabilitySample(unsigned long now);
    bool UpdateStability(unsigned long now);
    double CalculateStabilitySlope() const;
    double GetStableMean() const;

    double TargetForStep(uint8_t index) const;
    int ExpectedPVDirection() const;
    bool ApplyCurrentStep(unsigned long now);
    void CompleteCurrentStep(unsigned long now);
    void FinishTest();
    void AbortWithError(StepTestError reason);

    void ResetSamples();
    void RecordSample(unsigned long now, bool force = false);
    void CompressSamples();
    bool FindCrossingTime(double targetPV, bool increasing, double &timeSeconds) const;
    StepTestStepResult CalculateStepResult(double finalPV) const;
};

#endif
