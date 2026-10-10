#include "RpmSensor.h"

volatile uint32_t RpmSensor::isrPulseCount = 0;
volatile unsigned long RpmSensor::isrLastPulseMicros = 0;

void IRAM_ATTR RpmSensor::isrHandler() {
    isrPulseCount++;
    isrLastPulseMicros = micros();
}

RpmSensor::RpmSensor()
    : pin(PIN_FLYWHEEL),
      currentRpm(0.0),
      engineRunning(false),
      engineStopping(false),
      fakeActive(false),
      fakeRpm(1000.0),
      lastCalcTime(0),
      lastPulseCount(0),
      engineStartedTime(0) {
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

    if (fakeActive) {
        currentRpm = fakeRpm;
    } else {
        // Atomic read of ISR counters
        noInterrupts();
        uint32_t pulses = isrPulseCount;
        unsigned long lastPulseUs = isrLastPulseMicros;
        interrupts();

        uint32_t deltaPulses = pulses - lastPulseCount;
        lastPulseCount = pulses;
        lastCalcTime = now;

        // If no pulse received for > 500ms, RPM is 0
        if ((micros() - lastPulseUs) > 500000UL) {
            currentRpm = 0.0;
        } else if (dt > 0) {
            double freqHz = (double)deltaPulses * 1000.0 / (double)dt;
            currentRpm = freqHz * RPM_FACTOR;
        }
    }

    // Evaluate running vs stopping states
    if (currentRpm >= MIN_ENGINE_RUNNING_RPM) {
        if (!engineRunning) {
            engineRunning = true;
            engineStopping = false;
            engineStartedTime = now;
            Serial.printf("[RPM] Engine RUNNING (%.1f RPM)\n", currentRpm);
        }
    } else if (currentRpm < ENGINE_SHUTDOWN_RPM) {
        if (engineRunning) {
            engineRunning = false;
            engineStopping = true;
            Serial.printf("[RPM] Engine STOPPING / STOPPED (%.1f RPM)\n", currentRpm);
        }
    }
}

double RpmSensor::getRpm() const {
    return currentRpm;
}

bool RpmSensor::isEngineRunning() const {
    return engineRunning;
}

bool RpmSensor::isEngineStopping() const {
    return engineStopping;
}

void RpmSensor::toggleFakeRpm(double fakeValue) {
    fakeActive = !fakeActive;
    fakeRpm = fakeValue;
    if (fakeActive) {
        Serial.printf("[RPM] Fake RPM ENABLED: %.0f RPM\n", fakeRpm);
    } else {
        Serial.println(F("[RPM] Fake RPM DISABLED"));
    }
}
