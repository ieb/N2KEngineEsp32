#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "config.h"

#define MAX_ONE_WIRE_SENSORS 4

class OneWireTemp {
public:
    OneWireTemp();
    void begin(uint8_t pin = PIN_ONE_WIRE);
    void update();

    uint8_t getActiveCount() const { return activeCount; }
    double getTemperatureK(uint8_t index) const;

private:
    OneWire oneWire;
    DallasTemperature sensors;
    DeviceAddress deviceAddresses[MAX_ONE_WIRE_SENSORS];
    double temperaturesK[MAX_ONE_WIRE_SENSORS];
    uint8_t activeCount;

    unsigned long lastRequestTime;
    bool requestPending;
};
