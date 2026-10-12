#pragma once
#include <Arduino.h>
#include "config.h"
#include "RpmSensorBase.h"

// Window-counting implementation: the ISR counts rising edges and update()
// divides the count by the elapsed millis() over a ~100 ms window.
// Resolution is +/-1 pulse per window (20 RPM with 30 teeth) plus millis() jitter.
class RpmSensor : public RpmSensorBase {
public:
    RpmSensor();
    void begin(uint8_t pin = PIN_FLYWHEEL) override;
    void update() override;
    const char *name() const override { return "window-count"; }

    static void IRAM_ATTR isrHandler();

private:
    uint8_t pin;
    unsigned long lastCalcTime;
    uint32_t lastPulseCount;

    static volatile uint32_t isrPulseCount;
    static volatile unsigned long isrLastPulseMicros;
};
