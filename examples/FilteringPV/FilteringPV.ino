#include <PIDControlFilters.h>

using namespace PIDControlFilters;

MovingAverageFilter<10> movingAverage;
MedianFilter<5> medianFilter;
LowPassFilter lowPass(1.0);          // tau = 1 s
ComplementaryFilter complementary(0.98);

unsigned long previousMs = 0;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    const unsigned long now = millis();
    if (now - previousMs < 100)
        return;

    const double dt = (previousMs == 0) ? 0.1 : (now - previousMs) / 1000.0;
    previousMs = now;

    const double raw = analogRead(A0);

    const double ma = movingAverage.Update(raw);
    const double med = medianFilter.Update(raw);
    const double lp = lowPass.Update(med, dt);

    // Example only: complementary filters need two estimates of the same
    // physical variable. Here ma is used as the slow reference and lp as
    // the faster estimate solely to demonstrate the API.
    const double comp = complementary.Update(lp, ma);

    Serial.print(raw); Serial.print(' ');
    Serial.print(ma); Serial.print(' ');
    Serial.print(med); Serial.print(' ');
    Serial.print(lp); Serial.print(' ');
    Serial.println(comp);
}
