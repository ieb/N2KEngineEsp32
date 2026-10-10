#pragma once
#include <Arduino.h>
#include "config.h"

class RpmSensor {
public:
    RpmSensor();
    void begin(uint8_t pin = PIN_FLYWHEEL);
    void update();

    double getRpm() const;
    bool isEngineRunning() const;
    bool isEngineStopping() const;
    void toggleFakeRpm(double fakeValue = 1000.0);
    bool isFakeActive() const { return fakeActive; }

    static void IRAM_ATTR isrHandler();

private:
    uint8_t pin;
    double currentRpm;
    bool engineRunning;
    bool engineStopping;
    bool fakeActive;
    double fakeRpm;

    unsigned long lastCalcTime;
    uint32_t lastPulseCount;
    unsigned long engineStartedTime;

    static volatile uint32_t isrPulseCount;
    static volatile unsigned long isrLastPulseMicros;
};
