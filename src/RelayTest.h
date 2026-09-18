#ifndef RELAYTEST_H
#define RELAYTEST_H

#include <Arduino.h>
#include "PIDControl.h"

enum class RelayTestState
{
    IDLE,
    RUNNING,
    FINISHED,
    ABORTED
};

enum class RelayTestError
{
    NONE,
    INVALID_CONFIG,
    PV_LOW,
    PV_HIGH,
    OP_LIMIT,
    TIMEOUT,
    NO_OSCILLATION,
    USER_ABORT
};


struct RelayTestConfig
{
    double amplitude;
    double hysteresis;
    uint8_t cycles;
    bool useCurrentReference;
    double referencePV;
    double maxTestTimeMinutes;
    double outputMin;
    double outputMax;
    double pvMin;
    double pvMax;

    RelayTestConfig()
        : amplitude(5.0), hysteresis(0.5), cycles(5),
          useCurrentReference(true), referencePV(0.0),
          maxTestTimeMinutes(10.0), outputMin(0.0), outputMax(100.0),
          pvMin(-1.0e30), pvMax(1.0e30)
    {
    }
};

struct RelayTestResult
{
    bool valid;
    double Ku;
    double Tu;
    double amplitudePV;
    double relayAmplitude;
    uint8_t completedCycles;
};

class RelayTest
{
public:
    RelayTest(PIDControl &controller, double *input, double *output);

    void SetOutputLimits(double minimum, double maximum);
    void SetPVLimits(double minimum, double maximum);
    void SetRelayAmplitude(double value);
    void SetHysteresis(double value);
    void SetCycles(uint8_t cycles);
    void SetReference(double value);
    void UseCurrentPVAsReference();

    // User-facing unit: minutes. Internally stored as milliseconds.
    void SetMaxTestTime(double minutes);
    double GetMaxTestTime() const;

    bool Start();
    bool Start(const RelayTestConfig &config);
    void Update();
    void Abort();

    // Runtime reset is inline so sketches never depend on a separately linked
    // Reset() symbol. Configuration values are preserved.
    void Reset()
    {
        if (IsRunning())
        {
            pid.SetMode(PIDMode::MAN);
            pid.SetManualOutput(biasOutput);
        }

        state = RelayTestState::IDLE;
        error = RelayTestError::NONE;
        testStartMs = 0;
        previousHighSwitchMs = 0;
        haveCycleStart = false;
        warmupDiscarded = false;
        cycleMinPV = 0.0;
        cycleMaxPV = 0.0;
        sumKu = 0.0;
        sumTu = 0.0;
        sumAmplitudePV = 0.0;
        completedCycles = 0;
    }

    bool IsRunning() const;
    bool IsFinished() const;
    bool IsAborted() const;

    RelayTestState GetState() const;
    RelayTestError GetError() const;
    RelayTestResult GetResult() const;

private:
    PIDControl &pid;
    double *Input;
    double *Output;

    double minTestOutput;
    double maxTestOutput;
    double minPV;
    double maxPV;
    double relayAmplitude;
    double hysteresis;
    uint8_t requiredCycles;
    unsigned long maxTestTimeMs;

    bool useCurrentReference;
    double referencePV;
    double biasOutput;
    double highOutput;
    double lowOutput;
    bool relayHigh;
    int processSign;

    RelayTestState state;
    RelayTestError error;
    unsigned long testStartMs;
    unsigned long previousHighSwitchMs;
    bool haveCycleStart;
    bool warmupDiscarded;

    double cycleMinPV;
    double cycleMaxPV;
    double sumKu;
    double sumTu;
    double sumAmplitudePV;
    uint8_t completedCycles;

    static unsigned long MinutesToMillis(double minutes);
    bool CheckSafety();
    void SwitchHigh(unsigned long now);
    void SwitchLow();
    void CompleteCycle(unsigned long now);
    void FinishTest();
    void AbortWithError(RelayTestError reason);
};

#endif
