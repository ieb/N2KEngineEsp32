#pragma once
#include <Arduino.h>
#include "config.h"
#include "RpmSensorBase.h"

// Bench test for the flywheel RPM measurement.
// Generates a square wave with the LEDC peripheral on PIN_BENCH_SIGNAL and
// reports how the active RPM sensor implementation tracks it.
//
// Wiring: GPIO2 -> 1K -> GPIO4 (flywheel sense), and short the flywheel input
// (J15) so the LM393 open-collector output stays released.
//
// CLI 'T' cycles the test frequency: off -> 100 Hz -> 400 Hz -> 1500 Hz -> off
// (200, 800 and 3000 RPM with 30 teeth). Every 2 s the min/mean/max/std-dev of
// the reported RPM is printed against the expected value.
class BenchSignal {
public:
    BenchSignal();
    void cycle();
    void update(const RpmSensorBase &rpm);
    bool isActive() const { return freqHz != 0; }

private:
    void start(uint32_t hz);
    void stop();
    void resetStats();

    uint32_t freqHz;
    uint8_t stepIndex;
    unsigned long windowStart;
    unsigned long lastSample;
    uint32_t lastGlitches;

    // Statistics over the current report window
    uint32_t n;
    double sum;
    double sumSq;
    double minRpm;
    double maxRpm;
};
