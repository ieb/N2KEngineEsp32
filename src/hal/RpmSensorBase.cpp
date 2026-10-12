#include "RpmSensorBase.h"

void RpmSensorBase::toggleFakeRpm(double fakeValue) {
    fakeActive = !fakeActive;
    fakeRpm = fakeValue;
    if (fakeActive) {
        Serial.printf("[RPM] Fake RPM ENABLED: %.0f RPM\n", fakeRpm);
    } else {
        Serial.println(F("[RPM] Fake RPM DISABLED"));
    }
}

void RpmSensorBase::setMeasuredRpm(double rpm, unsigned long nowMs) {
    currentRpm = fakeActive ? fakeRpm : rpm;

    // Evaluate running vs stopping states
    if (currentRpm >= MIN_ENGINE_RUNNING_RPM) {
        if (!engineRunning) {
            engineRunning = true;
            engineStopping = false;
            engineStartedTime = nowMs;
            Serial.printf("[RPM] Engine RUNNING (%.1f RPM)\n", currentRpm);
        }
    } else if (currentRpm < ENGINE_SHUTDOWN_RPM) {
        if (engineRunning) {
            engineRunning = false;
            engineStopping = true;
            Serial.printf("[RPM] Engine STOPPING (%.1f RPM)\n", currentRpm);
        }
        // Stopping lasts until the flywheel has come to rest
        if (engineStopping && currentRpm <= 0.0) {
            engineStopping = false;
            Serial.println(F("[RPM] Engine STOPPED"));
        }
    }
}
