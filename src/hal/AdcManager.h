#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <ADS1X15.h>
#include "config.h"

struct AdcReadings {
    // ADC 1 (0x48)
    float ntc1ExhaustWaterTrap_V = 0.0f;
    float ntc2Alternator_V       = 0.0f;
    float ntc3EngineRoom_V       = 0.0f;
    float alternatorVoltage_V    = 0.0f;

    // ADC 2 (0x49)
    float oilPressure_V          = 0.0f;
    float coolant_V              = 0.0f;
    float engineBlock_V          = 0.0f;
    float engineBattery_V        = 0.0f;

    // ADC 3 (0x4B)
    float ntc4Gearbox_V          = 0.0f;
    float ntc5ExhaustElbow_V     = 0.0f;
    float fuel_V                 = 0.0f;
    float spareVoltage_V         = 0.0f;

    // Raw ADC counts (normalized to 0..4096 scale for lookup tables if needed)
    int16_t raw_ntc1             = 0;
    int16_t raw_ntc2             = 0;
    int16_t raw_ntc3             = 0;
    int16_t raw_ntc4             = 0;
    int16_t raw_ntc5             = 0;
    int16_t raw_coolant          = 0;
    int16_t raw_oil              = 0;
    int16_t raw_fuel             = 0;
    int16_t raw_engineBlock      = 0;

    bool adc1Ok = false;
    bool adc2Ok = false;
    bool adc3Ok = false;
};

class AdcManager {
public:
    AdcManager();
    bool begin(TwoWire &wire = Wire);
    void update();
    const AdcReadings &getReadings() const { return readings; }
    void printDebug();

private:
    TwoWire *wirePtr;
    ADS1115 ads1;
    ADS1115 ads2;
    ADS1115 ads3;

    AdcReadings readings;
    uint8_t currentChannel;
    bool conversionPending;
    unsigned long conversionStartTime;

    // Exponential moving average filter (alpha = 0.25)
    float filter(float oldVal, float newVal, float alpha = 0.25f);
    void storeReading(uint8_t chip, uint8_t channel, int16_t rawCounts, float voltage);
};
