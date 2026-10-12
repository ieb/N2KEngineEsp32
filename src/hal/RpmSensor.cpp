#include "RpmSensor.h"

volatile uint32_t RpmSensor::isrPulseCount = 0;
volatile unsigned long RpmSensor::isrLastPulseMicros = 0;

void IRAM_ATTR RpmSensor::isrHandler() {
    isrPulseCount++;
    isrLastPulseMicros = micros();
}

RpmSensor::RpmSensor()
    : pin(PIN_FLYWHEEL),
      lastCalcTime(0),
      lastPulseCount(0) {
}

void RpmSensor::begin(uint8_t pin) {
    this->pin = pin;
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), isrHandler, RISING);
    lastCalcTime = millis();
    lastPulseCount = isrPulseCount;
}

void RpmSensor::update() {
    unsigned long now = millis();
    unsigned long dt = now - lastCalcTime;
    if (dt < FLYWHEEL_READ_PERIOD_MS) {
        return;
    }

    // Always sample the ISR counters so switching fake mode off does not produce
    // a burst of accumulated pulses.
    noInterrupts();
    uint32_t pulses = isrPulseCount;
    unsigned long lastPulseUs = isrLastPulseMicros;
    interrupts();

    uint32_t deltaPulses = pulses - lastPulseCount;
    lastPulseCount = pulses;
    lastCalcTime = now;

    double rpm;
    if ((micros() - lastPulseUs) > 500000UL) {
        // If no pulse received for > 500ms, RPM is 0
        rpm = 0.0;
    } else {
        double freqHz = (double)deltaPulses * 1000.0 / (double)dt;
        rpm = freqHz * RPM_FACTOR;
    }
    setMeasuredRpm(rpm, now);
}
