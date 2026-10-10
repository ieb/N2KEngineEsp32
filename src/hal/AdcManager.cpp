#include "AdcManager.h"

AdcManager::AdcManager()
    : wirePtr(&Wire),
      ads1(ADS1115_ADDR_1),
      ads2(ADS1115_ADDR_2),
      ads3(ADS1115_ADDR_3),
      currentChannel(0),
      conversionPending(false),
      conversionStartTime(0) {
}

bool AdcManager::begin(TwoWire &wire) {
    wirePtr = &wire;
    wirePtr->begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY);

    readings.adc1Ok = ads1.begin();
    if (readings.adc1Ok) {
        ads1.setGain(0); // +/- 6.144V (0.1875 mV/LSB)
        ads1.setDataRate(3); // 64 SPS
        ads1.setMode(1); // Single-shot
    } else {
        Serial.println(F("[ADC] ADS1 (0x48) not found on I2C"));
    }

    readings.adc2Ok = ads2.begin();
    if (readings.adc2Ok) {
        ads2.setGain(0);
        ads2.setDataRate(3);
        ads2.setMode(1);
    } else {
        Serial.println(F("[ADC] ADS2 (0x49) not found on I2C"));
    }

    readings.adc3Ok = ads3.begin();
    if (readings.adc3Ok) {
        ads3.setGain(0);
        ads3.setDataRate(3);
        ads3.setMode(1);
    } else {
        Serial.println(F("[ADC] ADS3 (0x4B) not found on I2C"));
    }

    // Trigger initial conversions on channel 0
    if (readings.adc1Ok) ads1.requestADC(0);
    if (readings.adc2Ok) ads2.requestADC(0);
    if (readings.adc3Ok) ads3.requestADC(0);
    currentChannel = 0;
    conversionPending = true;
    conversionStartTime = millis();

    return (readings.adc1Ok || readings.adc2Ok || readings.adc3Ok);
}

float AdcManager::filter(float oldVal, float newVal, float alpha) {
    if (oldVal == 0.0f) return newVal;
    return oldVal + alpha * (newVal - oldVal);
}

void AdcManager::storeReading(uint8_t chip, uint8_t channel, int16_t rawCounts, float voltage) {
    if (chip == 1) {
        switch (channel) {
            case 0:
                readings.raw_ntc1 = rawCounts;
                readings.ntc1ExhaustWaterTrap_V = filter(readings.ntc1ExhaustWaterTrap_V, voltage);
                break;
            case 1:
                readings.raw_ntc2 = rawCounts;
                readings.ntc2Alternator_V = filter(readings.ntc2Alternator_V, voltage);
                break;
            case 2:
                readings.raw_ntc3 = rawCounts;
                readings.ntc3EngineRoom_V = filter(readings.ntc3EngineRoom_V, voltage);
                break;
            case 3:
                readings.alternatorVoltage_V = filter(readings.alternatorVoltage_V, voltage);
                break;
        }
    } else if (chip == 2) {
        switch (channel) {
            case 0:
                readings.raw_oil = rawCounts;
                readings.oilPressure_V = filter(readings.oilPressure_V, voltage);
                break;
            case 1:
                readings.raw_coolant = rawCounts;
                readings.coolant_V = filter(readings.coolant_V, voltage);
                break;
            case 2:
                readings.raw_engineBlock = rawCounts;
                readings.engineBlock_V = filter(readings.engineBlock_V, voltage);
                break;
            case 3:
                readings.engineBattery_V = filter(readings.engineBattery_V, voltage);
                break;
        }
    } else if (chip == 3) {
        switch (channel) {
            case 0:
                readings.raw_ntc4 = rawCounts;
                readings.ntc4Gearbox_V = filter(readings.ntc4Gearbox_V, voltage);
                break;
            case 1:
                readings.raw_ntc5 = rawCounts;
                readings.ntc5ExhaustElbow_V = filter(readings.ntc5ExhaustElbow_V, voltage);
                break;
            case 2:
                readings.raw_fuel = rawCounts;
                readings.fuel_V = filter(readings.fuel_V, voltage);
                break;
            case 3:
                readings.spareVoltage_V = filter(readings.spareVoltage_V, voltage);
                break;
        }
    }
}

void AdcManager::update() {
    if (!conversionPending) {
        // Start conversion on currentChannel
        if (readings.adc1Ok) ads1.requestADC(currentChannel);
        if (readings.adc2Ok) ads2.requestADC(currentChannel);
        if (readings.adc3Ok) ads3.requestADC(currentChannel);
        conversionPending = true;
        conversionStartTime = millis();
        return;
    }

    // Check if conversion is ready on all active chips (or timeout after 25ms, since 64SPS is ~15.6ms)
    bool ready1 = (!readings.adc1Ok) || ads1.isReady();
    bool ready2 = (!readings.adc2Ok) || ads2.isReady();
    bool ready3 = (!readings.adc3Ok) || ads3.isReady();
    bool timeout = (millis() - conversionStartTime) > 25;

    if ((ready1 && ready2 && ready3) || timeout) {
        // Read conversion results
        const float lsbVolts = 6.144f / 32768.0f; // 0.1875 mV per bit

        if (readings.adc1Ok) {
            int16_t val1 = ads1.getValue();
            float v1 = val1 * lsbVolts;
            if (v1 < 0.0f) v1 = 0.0f;
            storeReading(1, currentChannel, val1, v1);
        }

        if (readings.adc2Ok) {
            int16_t val2 = ads2.getValue();
            float v2 = val2 * lsbVolts;
            if (v2 < 0.0f) v2 = 0.0f;
            storeReading(2, currentChannel, val2, v2);
        }

        if (readings.adc3Ok) {
            int16_t val3 = ads3.getValue();
            float v3 = val3 * lsbVolts;
            if (v3 < 0.0f) v3 = 0.0f;
            storeReading(3, currentChannel, val3, v3);
        }

        // Advance to next channel (0..3)
        currentChannel = (currentChannel + 1) & 0x03;
        conversionPending = false;
    }
}

void AdcManager::printDebug() {
    Serial.printf("[ADC1] ExhTrap=%.3fV, Alt=%.3fV, Room=%.3fV, AltDiv=%.3fV\n",
                  readings.ntc1ExhaustWaterTrap_V, readings.ntc2Alternator_V,
                  readings.ntc3EngineRoom_V, readings.alternatorVoltage_V);
    Serial.printf("[ADC2] Oil=%.3fV, Cool=%.3fV, Block=%.3fV, BattDiv=%.3fV\n",
                  readings.oilPressure_V, readings.coolant_V,
                  readings.engineBlock_V, readings.engineBattery_V);
    Serial.printf("[ADC3] Gearbox=%.3fV, Elbow=%.3fV, Fuel=%.3fV, Spare=%.3fV\n",
                  readings.ntc4Gearbox_V, readings.ntc5ExhaustElbow_V,
                  readings.fuel_V, readings.spareVoltage_V);
}
