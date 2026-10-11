#include "OneWireTemp.h"
#include <N2kMsg.h>

OneWireTemp::OneWireTemp()
    : oneWire(PIN_ONE_WIRE),
      sensors(&oneWire),
      activeCount(0),
      lastRequestTime(0),
      requestPending(false) {
    for (int i = 0; i < MAX_ONE_WIRE_SENSORS; i++) {
        temperaturesK[i] = N2kDoubleNA;
    }
}

void OneWireTemp::begin(uint8_t pin) {
    oneWire.begin(pin);
    sensors.setOneWire(&oneWire);
    sensors.begin();
    sensors.setWaitForConversion(false);

    activeCount = 0;
    for (int i = 0; i < MAX_ONE_WIRE_SENSORS; i++) {
        if (sensors.getAddress(deviceAddresses[i], i)) {
            Serial.printf("[1-Wire] Sensor %d found: ", i);
            for (uint8_t b = 0; b < 8; b++) {
                Serial.printf("%02X", deviceAddresses[i][b]);
            }
            Serial.println();
            activeCount++;
        }
    }

    if (activeCount > 0) {
        sensors.requestTemperatures();
        requestPending = true;
        lastRequestTime = millis();
    }
}

void OneWireTemp::update() {
    if (activeCount == 0) return;

    unsigned long now = millis();
    if (requestPending) {
        // Wait at least 750ms for DS18B20 12-bit conversion
        if (now - lastRequestTime >= 1000) {
            for (uint8_t i = 0; i < activeCount; i++) {
                float tempC = sensors.getTempC(deviceAddresses[i]);
                // 85.0C is the DS18B20 power-on reset value: only accept it when the
                // previous reading was already close (i.e. it is a genuine temperature).
                bool powerOnValue = (tempC == 85.0f) &&
                    (temperaturesK[i] == N2kDoubleNA ||
                     fabs((temperaturesK[i] - 273.15) - 85.0) > 10.0);
                if (tempC > -55.0f && tempC < 125.0f && !powerOnValue) {
                    temperaturesK[i] = (double)tempC + 273.15;
                } else {
                    temperaturesK[i] = N2kDoubleNA;
                }
            }
            requestPending = false;
            lastRequestTime = now;
        }
    } else {
        // Request temperature conversions every 3 seconds
        if (now - lastRequestTime >= 3000) {
            sensors.requestTemperatures();
            requestPending = true;
            lastRequestTime = now;
        }
    }
}

double OneWireTemp::getTemperatureK(uint8_t index) const {
    if (index >= activeCount) return N2kDoubleNA;
    return temperaturesK[index];
}
