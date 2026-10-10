#include "DigitalInputs.h"

DigitalInputs::DigitalInputs() : lastReadTime(0) {}

void DigitalInputs::begin() {
    pinMode(PIN_STOP_COIL, INPUT);
    pinMode(PIN_START_COIL, INPUT);
    pinMode(PIN_GLOW_COIL, INPUT);
}

void DigitalInputs::update() {
    unsigned long now = millis();
    if (now - lastReadTime < 50) { // 20 Hz polling
        return;
    }
    lastReadTime = now;

    states.stopActive  = (digitalRead(PIN_STOP_COIL) == HIGH);
    states.startActive = (digitalRead(PIN_START_COIL) == HIGH);
    states.glowActive  = (digitalRead(PIN_GLOW_COIL) == HIGH);
}
