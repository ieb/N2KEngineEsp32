#pragma once
#include <Arduino.h>
#include "config.h"

// Common interface and running/stopping state machine for flywheel RPM sensors.
// Two implementations exist so they can be compared on the bench:
//   RpmSensor      - counts pulses in a fixed ~100 ms window (Arduino ISR)
//   EdgeRpmSensor  - times edges and averages over whole revolutions (IRAM ISR)
// Select one with RPM_SENSOR_EDGE_TIMING in config.h.
class RpmSensorBase {
public:
    virtual ~RpmSensorBase() {}
    virtual void begin(uint8_t pin = PIN_FLYWHEEL) = 0;
    virtual void update() = 0;
    virtual const char *name() const = 0;
    // Pulses rejected as glitches (implementations that filter; 0 otherwise)
    virtual uint32_t getGlitchCount() const { return 0; }

    double getRpm() const { return currentRpm; }
    bool isEngineRunning() const { return engineRunning; }
    bool isEngineStopping() const { return engineStopping; }
    void toggleFakeRpm(double fakeValue = 1000.0);
    bool isFakeActive() const { return fakeActive; }

protected:
    // Applies a new measurement (or the fake value) and runs the state machine.
    void setMeasuredRpm(double rpm, unsigned long nowMs);

    double currentRpm = 0.0;
    bool engineRunning = false;
    bool engineStopping = false;
    bool fakeActive = false;
    double fakeRpm = 1000.0;
    unsigned long engineStartedTime = 0;
};
