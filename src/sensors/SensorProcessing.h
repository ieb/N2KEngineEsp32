#pragma once
#include <Arduino.h>
#include "config.h"
#include "EngineState.h"
#include "hal/AdcManager.h"
#include "hal/RpmSensor.h"
#include "hal/DigitalInputs.h"
#include "hal/OneWireTemp.h"

class SensorProcessing {
public:
    SensorProcessing();
    void update(EngineState &state,
                const AdcReadings &adc,
                const RpmSensor &rpm,
                const RelayCoilStates &relays,
                const OneWireTemp &oneWire);

    static int16_t interpolateCurve(int16_t reading,
                                    int16_t minVal,
                                    int16_t maxVal,
                                    int16_t step,
                                    const int16_t *curve,
                                    int curveLength);

    double convertNtc(float voltage, int16_t &outDeciC);
    double convertCoolant(float coolantV, float blockV, float batV, int16_t &outDeciC);
    double convertOilPressure(float voltage);
    double convertFuelLevel(float voltage);
};

