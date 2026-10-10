#pragma once
#include "OneWire.h"

typedef uint8_t DeviceAddress[8];

class DallasTemperature {
public:
    DallasTemperature(OneWire *wire = nullptr) {}
    void setOneWire(OneWire *wire) {}
    void begin() {}
    void setWaitForConversion(bool wait) {}
    bool getAddress(DeviceAddress deviceAddress, uint8_t index) { return false; }
    void requestTemperatures() {}
    float getTempC(const uint8_t *deviceAddress) { return -127.0f; }
};
