#include "SensorProcessing.h"

// Volvo Penta Standard Coolant Sensor Lookup Table (from enginesensors.cpp)
// 10.0 C (100 deci-C) to 120.0 C (1200 deci-C) in 10.0 C (100 deci-C) steps.
static const int16_t coolantTable[] = {
    5095, 3969, 2999, 2216, 1618, 1162, 869, 643, 477, 360, 277, 212
};
#define COOLANT_MIN     100
#define COOLANT_MAX     1200
#define COOLANT_STEP    100
#define COOLANT_LEN     12

int16_t SensorProcessing::interpolateCurve(int16_t reading,
                                          int16_t minVal,
                                          int16_t maxVal,
                                          int16_t step,
                                          const int16_t *curve,
                                          int curveLength) {
    int16_t cvp = curve[0];
    if (reading > cvp) {
        return minVal;
    }
    for (int i = 1; i < curveLength; i++) {
        int16_t cv = curve[i];
        if (reading > cv) {
            return minVal + ((i - 1) * step) + ((cvp - reading) * step) / (cvp - cv);
        }
        cvp = cv;
    }
    return maxVal;
}

SensorProcessing::SensorProcessing() {
}


double SensorProcessing::convertNtc(float voltage, int16_t &outDeciC) {
    // Check for short-circuit (<0.05V) or open-circuit / disconnected (>4.95V)
    if (voltage <= 0.05f || voltage >= 4.95f) {
        outDeciC = -9999;
        return N2kDoubleNA;
    }

    // Circuit: 5V -> 4K7 -> ADC -> NTC -> GND
    // R_ntc = R_top * V / (5.0 - V)
    float rNtc = NTC_R_TOP_OHM * voltage / (5.0f - voltage);

    // Beta parameter model: 1/T = 1/T0 + (1/Beta) * ln(R / R0)
    // T in Kelvin, T0 = 298.15K (25C), R0 = 10000 ohm, Beta = 3950K
    float lnRatio = logf(rNtc / NTC_R0_OHM);
    float kelvin = 1.0f / ((1.0f / NTC_T0_KELVIN) + (lnRatio / NTC_BETA_KELVIN));

    // Sanity range check (-40C to 160C)
    if (kelvin < 233.15f || kelvin > 433.15f) {
        outDeciC = -9999;
        return N2kDoubleNA;
    }

    outDeciC = (int16_t)roundf((kelvin - 273.15f) * 10.0f);
    return (double)kelvin;
}


double SensorProcessing::convertCoolant(float coolantV, float blockV, float batV, int16_t &outDeciC) {
    outDeciC = -9999;

    // Sense node at the BAT46 clamp: sender open-circuit or engine colder than the
    // ADC can measure. The true value is unknown, so report N/A rather than a guess.
    if (coolantV >= COOLANT_CLAMP_V) {
        return N2kDoubleNA;
    }

    // True voltages referenced to engine block
    float vCoolantBlock = coolantV - blockV;
    float vSupplyBlock  = batV - blockV;

    if (vCoolantBlock < 0.0f) vCoolantBlock = 0.0f;
    if (vSupplyBlock < 5.0f) {
        // Battery supply too low to read coolant sensor
        return N2kDoubleNA;
    }

    // Convert to 12V normalized ADC count (4096 counts @ 5V basis)
    // coolantReading_12V = (4096 * (vCoolant / 5.0)) * (12.0 / vSupply)
    float normalizedReading = 4096.0f * (vCoolantBlock / 5.0f) * (12.0f / vSupplyBlock);
    if (normalizedReading > 32000.0f) normalizedReading = 32000.0f;
    int16_t reading = (int16_t)(normalizedReading + 0.5f);

    // Shorted sender (or sense wire shorted to block) would otherwise read as 120C
    if (reading < COOLANT_SHORT_COUNTS) {
        return N2kDoubleNA;
    }

    outDeciC = interpolateCurve(reading, COOLANT_MIN, COOLANT_MAX, COOLANT_STEP, coolantTable, COOLANT_LEN);
    return (0.1 * outDeciC) + 273.15;
}

double SensorProcessing::convertOilPressure(float voltage) {
    if (voltage < OIL_PRESS_DISCONNECTED_V) {
        return N2kDoubleNA;
    }
    double pa = (voltage - OIL_PRESS_V0) * OIL_PRESS_SCALE_PA;
    if (pa < 0.0) pa = 0.0;
    return pa;
}

double SensorProcessing::convertFuelLevel(float voltage) {
    if (voltage < 0.01f) {
        return 0.0;
    }
    // European fuel sender 0-190 ohm with 1K pullup to 5V:
    // V = 5 * R / (1000 + R) => R = 1000 * V / (5 - V)
    if (voltage >= 4.5f) {
        return N2kDoubleNA; // Disconnected or open circuit
    }
    float rSender = FUEL_RESISTOR_TOP * voltage / (5.0f - voltage);
    double percent = (rSender / FUEL_SENSOR_MAX_OHM) * 100.0;
    if (percent < 0.0) percent = 0.0;
    if (percent > 100.0) percent = 100.0;
    return percent;
}

void SensorProcessing::update(EngineState &state,
                             const AdcReadings &adc,
                             const RpmSensor &rpm,
                             const RelayCoilStates &relays,
                             const OneWireTemp &oneWire) {
    // 1. Speeds & Relays
    state.engineRpm = rpm.getRpm();
    state.isEngineRunning = rpm.isEngineRunning();
    state.isEngineStopping = rpm.isEngineStopping();
    state.stopRelayActive = relays.stopActive;
    state.startRelayActive = relays.startActive;
    state.glowRelayActive = relays.glowActive;

    // 2. Engine Block Negative Offset
    // Circuit: 5V --- 10K --- ADC (ADC2 AIN2) --- 10K --- Engine Block (-)
    // V_adc = (5.0 + V_block) / 2 => V_block = 2 * V_adc - 5.0
    // A disconnected block wire reads ~+5V; an implausible offset is not used for compensation.
    float blockComp = 0.0f;
    if (adc.adc2Ok) {
        float block = 2.0f * adc.engineBlock_V - 5.0f;
        state.engineBlockOffsetV = block;
        if (block >= ENGINE_BLOCK_OFFSET_MIN_V && block <= ENGINE_BLOCK_OFFSET_MAX_V) {
            blockComp = block;
        }
    } else {
        state.engineBlockOffsetV = 0.0;
    }

    // 3. Electrical Voltages (N/A when the ADC chip is missing or failing)
    state.alternatorVoltage = adc.adc1Ok ? adc.alternatorVoltage_V * VOLTAGE_SCALE_ALTERNATOR : N2kDoubleNA;
    state.engineBatteryVoltage = adc.adc2Ok ? adc.engineBattery_V * VOLTAGE_SCALE : N2kDoubleNA;
    if (state.engineBatteryVoltage != N2kDoubleNA && state.engineBatteryVoltage < ENGINE_BATTERY_OFF_V) {
        state.engineBatteryVoltage = N2kDoubleNA; // switched feed off (ignition off)
    }
    state.spareVoltage = adc.adc3Ok ? adc.spareVoltage_V * VOLTAGE_SCALE : N2kDoubleNA;

    // 4. Temperatures
    // NTC1: Exhaust Water Trap (Reported as Exhaust Gas)
    state.exhaustWaterTrapK = convertNtc(adc.adc1Ok ? adc.ntc1ExhaustWaterTrap_V : 0.0f, state.exhaustWaterTrap_dC);

    // NTC2: Alternator
    state.alternatorTempK = convertNtc(adc.adc1Ok ? adc.ntc2Alternator_V : 0.0f, state.alternatorTemp_dC);

    // NTC3: Engine Room
    state.engineRoomTempK = convertNtc(adc.adc1Ok ? adc.ntc3EngineRoom_V : 0.0f, state.engineRoomTemp_dC);

    // NTC4: Gearbox
    state.gearboxTempK = convertNtc(adc.adc3Ok ? adc.ntc4Gearbox_V : 0.0f, state.gearboxTemp_dC);

    // NTC5: Exhaust Elbow
    state.exhaustElbowTempK = convertNtc(adc.adc3Ok ? adc.ntc5ExhaustElbow_V : 0.0f, state.exhaustElbowTemp_dC);

    // Coolant (adjusted for block drop and battery voltage)
    if (adc.adc2Ok) {
        state.coolantTempK = convertCoolant(adc.coolant_V,
                                            blockComp,
                                            (float)state.engineBatteryVoltage,
                                            state.coolantTemp_dC);
    } else {
        state.coolantTempK = N2kDoubleNA;
        state.coolantTemp_dC = -9999;
    }

    // 1-Wire Dallas Temperatures
    state.oneWireCount = oneWire.getActiveCount();
    for (uint8_t i = 0; i < state.oneWireCount; i++) {
        state.oneWireTempK[i] = oneWire.getTemperatureK(i);
    }

    // 5. Pressures & Fluids
    state.oilPressurePa = adc.adc2Ok ? convertOilPressure(adc.oilPressure_V) : N2kDoubleNA;
    state.fuelLevelPercent = adc.adc3Ok ? convertFuelLevel(adc.fuel_V) : N2kDoubleNA;
}
