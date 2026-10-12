#pragma once
#include <Arduino.h>
#include "config.h"
#include "RpmSensorBase.h"

// Edge-timing implementation (reciprocal counting).
//
// The ISR timestamps every rising edge with a 1 us clock and records the
// timestamp of every FLYWHEEL_TEETH-th edge (one flywheel revolution). update()
// computes
//     RPM = revolutions / time between revolution edges
// so there is no +/-1 pulse quantisation and no dependence on loop timing.
// Averaging over whole revolutions cancels tooth-spacing errors and the speed
// ripple from individual cylinder firings. A new value is available once per
// revolution (20 ms at 3000 RPM, 75 ms at 800 RPM).
//
// On the ESP32 the ISR is registered directly with the IDF GPIO ISR service using
// ESP_INTR_FLAG_IRAM, so it keeps running while flash is being written (NVS
// engine-hours saves, SPIFFS event log). The Arduino attachInterrupt() path is not
// IRAM-safe in this build (CONFIG_ARDUINO_ISR_IRAM is not set), so pulses that
// arrive during a flash write are merged into a single interrupt and lost.
// NOTE: because the GPIO ISR service is installed with the IRAM flag, any other
// code that later uses attachInterrupt() would have its (non-IRAM) Arduino
// dispatcher called with the IRAM flag; nothing else in this firmware does.
class EdgeRpmSensor : public RpmSensorBase {
public:
    EdgeRpmSensor();
    void begin(uint8_t pin = PIN_FLYWHEEL) override;
    void update() override;
    const char *name() const override { return "edge-timing"; }
    uint32_t getGlitchCount() const override;

    // ISR entry point (void* signature for gpio_isr_handler_add)
    static void IRAM_ATTR isrHandler(void *arg);
    // Records one edge at time tUs (called by the ISR; also used by native tests)
    static void IRAM_ATTR recordEdge(uint32_t tUs);
    // Clears all ISR state (tests)
    static void resetIsrState();

private:
    uint8_t pin;
    bool haveAnchor;
    uint32_t anchorCount;
    uint32_t anchorUs;
    double measuredRpm;

    // ISR shared state (protected by a critical section when read)
    static volatile bool s_haveEdge;
    static volatile uint32_t s_edgeCount;
    static volatile uint32_t s_lastEdgeUs;
    static volatile uint32_t s_revCount;
    static volatile uint32_t s_revEdgeUs;
    static volatile uint32_t s_glitchCount;
};
