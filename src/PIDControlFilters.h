#ifndef PIDCONTROLFILTERS_H
#define PIDCONTROLFILTERS_H

#include <Arduino.h>
#include <math.h>

namespace PIDControlFilters
{

template <size_t N>
class MovingAverageFilter
{
    static_assert(N > 0, "MovingAverageFilter window must be greater than zero");

public:
    MovingAverageFilter() : index(0), count(0), sum(0.0), value(0.0) {}

    void Reset()
    {
        index = 0;
        count = 0;
        sum = 0.0;
        value = 0.0;
    }

    void Reset(double initialValue)
    {
        for (size_t i = 0; i < N; ++i)
            samples[i] = initialValue;

        index = 0;
        count = N;
        sum = initialValue * (double)N;
        value = initialValue;
    }

    double Update(double input)
    {
        if (count < N)
        {
            samples[index] = input;
            sum += input;
            ++count;
        }
        else
        {
            sum -= samples[index];
            samples[index] = input;
            sum += input;
        }

        index = (index + 1U) % N;
        value = sum / (double)count;
        return value;
    }

    double Value() const { return value; }
    bool IsInitialized() const { return count > 0; }
    size_t Count() const { return count; }
    static constexpr size_t WindowSize() { return N; }

private:
    double samples[N];
    size_t index;
    size_t count;
    double sum;
    double value;
};

template <size_t N>
class MedianFilter
{
    static_assert(N > 0, "MedianFilter window must be greater than zero");

public:
    MedianFilter() : index(0), count(0), value(0.0) {}

    void Reset()
    {
        index = 0;
        count = 0;
        value = 0.0;
    }

    void Reset(double initialValue)
    {
        for (size_t i = 0; i < N; ++i)
            samples[i] = initialValue;

        index = 0;
        count = N;
        value = initialValue;
    }

    double Update(double input)
    {
        samples[index] = input;
        index = (index + 1U) % N;
        if (count < N)
            ++count;

        double sorted[N];
        for (size_t i = 0; i < count; ++i)
            sorted[i] = samples[i];

        for (size_t i = 1; i < count; ++i)
        {
            const double key = sorted[i];
            size_t j = i;
            while (j > 0 && sorted[j - 1] > key)
            {
                sorted[j] = sorted[j - 1];
                --j;
            }
            sorted[j] = key;
        }

        if ((count & 1U) != 0U)
            value = sorted[count / 2U];
        else
            value = 0.5 * (sorted[count / 2U - 1U] + sorted[count / 2U]);

        return value;
    }

    double Value() const { return value; }
    bool IsInitialized() const { return count > 0; }
    size_t Count() const { return count; }
    static constexpr size_t WindowSize() { return N; }

private:
    double samples[N];
    size_t index;
    size_t count;
    double value;
};

class LowPassFilter
{
public:
    explicit LowPassFilter(double tauSeconds = 1.0)
        : tau(tauSeconds > 0.0 ? tauSeconds : 1.0), value(0.0), initialized(false) {}

    void SetTimeConstant(double tauSeconds)
    {
        if (tauSeconds > 0.0)
            tau = tauSeconds;
    }

    double GetTimeConstant() const { return tau; }

    void Reset()
    {
        value = 0.0;
        initialized = false;
    }

    void Reset(double initialValue)
    {
        value = initialValue;
        initialized = true;
    }

    double Update(double input, double dtSeconds)
    {
        if (!initialized)
        {
            Reset(input);
            return value;
        }

        if (dtSeconds <= 0.0)
            return value;

        const double alpha = 1.0 - exp(-dtSeconds / tau);
        value += alpha * (input - value);
        return value;
    }

    double Value() const { return value; }
    bool IsInitialized() const { return initialized; }

private:
    double tau;
    double value;
    bool initialized;
};

class ComplementaryFilter
{
public:
    explicit ComplementaryFilter(double alphaValue = 0.98)
        : alpha(ClampAlpha(alphaValue)), value(0.0), initialized(false) {}

    void SetAlpha(double alphaValue) { alpha = ClampAlpha(alphaValue); }
    double GetAlpha() const { return alpha; }

    void Reset()
    {
        value = 0.0;
        initialized = false;
    }

    void Reset(double initialValue)
    {
        value = initialValue;
        initialized = true;
    }

    // Generic complementary fusion. fastEstimate should contain the
    // high-frequency/dynamic estimate and slowReference the low-frequency
    // absolute reference of the same physical variable.
    double Update(double fastEstimate, double slowReference)
    {
        value = alpha * fastEstimate + (1.0 - alpha) * slowReference;
        initialized = true;
        return value;
    }

    // Classical form for an integrated rate plus an absolute reference,
    // e.g. gyroscope rate + accelerometer angle.
    double UpdateIntegrated(double rate, double absoluteReference, double dtSeconds)
    {
        if (!initialized)
        {
            Reset(absoluteReference);
            return value;
        }

        if (dtSeconds <= 0.0)
            return value;

        const double integratedEstimate = value + rate * dtSeconds;
        value = alpha * integratedEstimate + (1.0 - alpha) * absoluteReference;
        return value;
    }

    double Value() const { return value; }
    bool IsInitialized() const { return initialized; }

private:
    static double ClampAlpha(double a)
    {
        if (a < 0.0) return 0.0;
        if (a > 1.0) return 1.0;
        return a;
    }

    double alpha;
    double value;
    bool initialized;
};

} // namespace PIDControlFilters

#endif
