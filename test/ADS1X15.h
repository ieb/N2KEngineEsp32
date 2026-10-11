#pragma once
#include <cstdint>
#include "Wire.h"

#define ADS1X15_OK 0

class ADS1115 {
public:
    ADS1115(uint8_t addr = 0x48, TwoWire *wire = nullptr) {}
    bool begin() { return true; }
    void setGain(uint8_t gain) {}
    void setDataRate(uint8_t rate) {}
    void setMode(uint8_t mode) {}
    bool requestADC(uint8_t channel) { return true; }
    bool isReady() { return true; }
    int16_t getValue() { return 0; }
    int8_t getError() { return ADS1X15_OK; }
};
