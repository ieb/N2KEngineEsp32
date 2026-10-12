#include <Arduino.h>
#include "version.h"
#include "config.h"
#include "hal/AdcManager.h"
#include "hal/RpmSensor.h"
#include "hal/EdgeRpmSensor.h"
#include "hal/BenchSignal.h"
#include "hal/DigitalInputs.h"
#include "hal/OneWireTemp.h"
#include "sensors/EngineState.h"
#include "sensors/SensorProcessing.h"
#include "alarms/AlarmManager.h"
#include "storage/StorageManager.h"
#include "n2k/N2kManager.h"

// System Components
AdcManager       adcManager;
#if RPM_SENSOR_EDGE_TIMING
EdgeRpmSensor    rpmSensor;
#else
RpmSensor        rpmSensor;
#endif
BenchSignal      benchSignal;
DigitalInputs    digitalInputs;
OneWireTemp      oneWireTemp;
StorageManager   storageManager;
SensorProcessing sensorProcessing;
AlarmManager     alarmManager;
N2kManager       n2kManager;
EngineState      engineState;

// Debug and CLI flags
bool adcDebugOutput = false;
unsigned long lastHoursTick = 0;
unsigned long lastStatusPrint = 0;
bool wasEngineRunning = false;

// Non-blocking line entry for the 'E' (set engine hours) command
bool hoursEntryActive = false;
unsigned long hoursEntryStart = 0;
char hoursEntryBuf[16];
size_t hoursEntryLen = 0;
#define HOURS_ENTRY_TIMEOUT_MS 30000UL

static double naOr(double v, double fallback) {
    return v != N2kDoubleNA ? v : fallback;
}

void printStatus() {
    Serial.println(F("==================== ENGINE STATUS ===================="));
    Serial.printf("Engine State:  %s (RPM: %.0f)\n",
                  engineState.isEngineRunning ? "RUNNING" : (engineState.isEngineStopping ? "STOPPING" : "STOPPED"),
                  engineState.engineRpm);
    Serial.printf("Engine Hours:  %.2f hrs (%u sec)\n",
                  (double)engineState.engineHoursSeconds / 3600.0, engineState.engineHoursSeconds);
    if (engineState.systemTimeUtc > 0) {
        Serial.printf("GPS Time UTC:  %u\n", engineState.systemTimeUtc);
    } else {
        Serial.println(F("GPS Time UTC:  Not received yet"));
    }

    Serial.println(F("--- Temperatures ---"));
    Serial.printf("Exhaust Water Trap (NTC1): %.1f C\n",
                  engineState.exhaustWaterTrapK != N2kDoubleNA ? (engineState.exhaustWaterTrapK - 273.15) : -999.0);
    Serial.printf("Alternator         (NTC2): %.1f C\n",
                  engineState.alternatorTempK != N2kDoubleNA ? (engineState.alternatorTempK - 273.15) : -999.0);
    Serial.printf("Engine Room        (NTC3): %.1f C\n",
                  engineState.engineRoomTempK != N2kDoubleNA ? (engineState.engineRoomTempK - 273.15) : -999.0);
    Serial.printf("Gearbox            (NTC4): %.1f C\n",
                  engineState.gearboxTempK != N2kDoubleNA ? (engineState.gearboxTempK - 273.15) : -999.0);
    Serial.printf("Exhaust Elbow      (NTC5): %.1f C\n",
                  engineState.exhaustElbowTempK != N2kDoubleNA ? (engineState.exhaustElbowTempK - 273.15) : -999.0);
    Serial.printf("Engine Coolant           : %.1f C\n",
                  engineState.coolantTempK != N2kDoubleNA ? (engineState.coolantTempK - 273.15) : -999.0);

    for (uint8_t i = 0; i < engineState.oneWireCount; i++) {
        Serial.printf("1-Wire Aux Temp [%d]     : %.1f C\n",
                      i, engineState.oneWireTempK[i] != N2kDoubleNA ? (engineState.oneWireTempK[i] - 273.15) : -999.0);
    }

    Serial.println(F("--- Pressures & Levels ---"));
    Serial.printf("Oil Pressure:  %.1f kPa (%.1f PSI)\n",
                  engineState.oilPressurePa != N2kDoubleNA ? (engineState.oilPressurePa / 1000.0) : 0.0,
                  engineState.oilPressurePa != N2kDoubleNA ? (engineState.oilPressurePa / 6894.76) : 0.0);
    Serial.printf("Fuel Level:    %.1f %% (Capacity: %.1f L)\n",
                  engineState.fuelLevelPercent != N2kDoubleNA ? engineState.fuelLevelPercent : 0.0,
                  engineState.fuelCapacityLitres);

    Serial.println(F("--- Electrical ---"));
    Serial.printf("Engine Battery:      %.2f V\n", naOr(engineState.engineBatteryVoltage, -999.0));
    Serial.printf("Alternator Output:   %.2f V\n", naOr(engineState.alternatorVoltage, -999.0));
    Serial.printf("Engine Block Offset: %+.3f V (relative to 0V)\n", engineState.engineBlockOffsetV);

    Serial.println(F("--- Relays ---"));
    Serial.printf("Stop: %s | Start: %s | Glow: %s\n",
                  engineState.stopRelayActive ? "ON" : "OFF",
                  engineState.startRelayActive ? "ON" : "OFF",
                  engineState.glowRelayActive ? "ON" : "OFF");

    Serial.println(F("--- Discrete Alarm Status ---"));
    Serial.printf("Status 1: 0x%04X | Status 2: 0x%04X\n",
                  engineState.discreteStatus1, engineState.discreteStatus2);
    if (engineState.discreteStatus1 & STATUS1_CHECK_ENGINE)       Serial.print(F(" [CHECK_ENGINE]"));
    if (engineState.discreteStatus1 & STATUS1_OVERTEMP)           Serial.print(F(" [OVERTEMP]"));
    if (engineState.discreteStatus1 & STATUS1_LOW_OIL_PRES)       Serial.print(F(" [LOW_OIL_PRES]"));
    if (engineState.discreteStatus1 & STATUS1_LOW_SYSTEM_VOLTAGE) Serial.print(F(" [LOW_VOLTAGE]"));
    if (engineState.discreteStatus1 & STATUS1_WATER_FLOW)         Serial.print(F(" [WATER_FLOW_COLLAPSE]"));
    if (engineState.discreteStatus1 & STATUS1_CHARGE_INDICATOR)   Serial.print(F(" [CHARGE_INDICATOR]"));
    if (engineState.discreteStatus1 & STATUS1_PREHEAT_INDICATOR)  Serial.print(F(" [PREHEAT]"));
    if (engineState.discreteStatus1 & STATUS1_EMERGENCY_STOP)     Serial.print(F(" [EMERGENCY_STOP]"));
    if (engineState.discreteStatus2 & STATUS2_WARN_1)             Serial.print(F(" [WARN_1_OVERTEMP]"));
    if (engineState.discreteStatus2 & STATUS2_ENGINE_SHUTTING_DOWN) Serial.print(F(" [SHUTTING_DOWN]"));
    Serial.println();
    Serial.println(F("======================================================="));
}

void showHelp() {
    Serial.println(F("=== N2K Engine Monitor CLI Commands ==="));
    Serial.print(F("Git Version: ")); Serial.println(F(GIT_SHA1_VERSION));
    Serial.println(F("  'h' - Show this help menu"));
    Serial.println(F("  's' - Print formatted status dashboard"));
    Serial.println(F("  'F' - Toggle Fake 1000 RPM test"));
    Serial.println(F("  'E' - Set engine hours"));
    Serial.println(F("  'l' - Dump SPIFFS event history"));
    Serial.println(F("  'C' - Clear SPIFFS event history"));
    Serial.println(F("  'd' - Toggle raw ADC readings output"));
    Serial.printf("  'T' - Cycle bench RPM test signal on GPIO%d (off/100/400/1500 Hz), sensor: %s\n",
                  PIN_BENCH_SIGNAL, rpmSensor.name());
    Serial.println(F("  'R' - Restart ESP32"));
}

void handleHoursEntry() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            hoursEntryActive = false;
            if (hoursEntryLen == 0) {
                Serial.println(F("Canceled"));
                return;
            }
            hoursEntryBuf[hoursEntryLen] = '\0';
            double hrs = atof(hoursEntryBuf);
            if (hrs < 0.0 || hrs > 1000000.0) {
                Serial.println(F("Invalid value"));
                return;
            }
            uint32_t secs = (uint32_t)(hrs * 3600.0);
            storageManager.setEngineHoursSeconds(secs);
            engineState.engineHoursSeconds = secs;
            return;
        }
        if (hoursEntryLen < sizeof(hoursEntryBuf) - 1) {
            hoursEntryBuf[hoursEntryLen++] = c;
        }
    }
    if (millis() - hoursEntryStart > HOURS_ENTRY_TIMEOUT_MS) {
        hoursEntryActive = false;
        Serial.println(F("Canceled (timeout)"));
    }
}

void handleCli() {
    if (hoursEntryActive) {
        handleHoursEntry();
        return;
    }
    if (!Serial.available()) return;
    char ch = Serial.read();
    switch (ch) {
        case 'h':
            showHelp();
            break;
        case 's':
            printStatus();
            break;
        case 'F':
            rpmSensor.toggleFakeRpm(1000.0);
            break;
        case 'E':
            // Non-blocking: the main loop keeps running while the value is typed
            Serial.print(F("Enter new Engine Hours ?> "));
            hoursEntryActive = true;
            hoursEntryStart = millis();
            hoursEntryLen = 0;
            break;
        case 'l':
            storageManager.dumpEvents(Serial);
            break;
        case 'C':
            storageManager.clearEvents();
            break;
        case 'd':
            adcDebugOutput = !adcDebugOutput;
            Serial.printf("[DEBUG] ADC Debug output: %s\n", adcDebugOutput ? "ENABLED" : "DISABLED");
            break;
        case 'T':
            benchSignal.cycle();
            break;
        case 'R':
            Serial.println(F("Restarting..."));
            storageManager.saveEngineHoursIfNeeded(true);
            delay(100);
            ESP.restart();
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("=================================================="));
    Serial.println(F("  NMEA 2000 Engine Interface with ESP32-C3 & ADS1115"));
    Serial.printf("  Firmware Version: %s\n", GIT_SHA1_VERSION);
    Serial.println(F("=================================================="));

    // 1. SPIFFS & Non-Volatile Storage
    storageManager.begin();
    engineState.engineHoursSeconds = storageManager.getEngineHoursSeconds();

    // 2. Hardware Peripherals
    adcManager.begin();
    rpmSensor.begin(PIN_FLYWHEEL);
    digitalInputs.begin();
    oneWireTemp.begin(PIN_ONE_WIRE);

    // 3. NMEA 2000 Stack
    n2kManager.begin(&storageManager);

    Serial.println(F("[SYSTEM] Initialization complete. Type 'h' for help."));
}

void loop() {
    // 1. Hardware updates
    adcManager.update();
    rpmSensor.update();
    digitalInputs.update();
    oneWireTemp.update();

    // 2. Sensor signal processing & physical conversions
    sensorProcessing.update(engineState,
                           adcManager.getReadings(),
                           rpmSensor,
                           digitalInputs.getStates(),
                           oneWireTemp);

    // 3. Engine hours accumulation (every second while engine is running)
    // Advance the tick by exactly 1000ms so loop latency does not cause drift.
    unsigned long now = millis();
    if (now - lastHoursTick >= 1000) {
        if (now - lastHoursTick > 5000) {
            lastHoursTick = now; // resync after a long stall rather than burst-counting
        } else {
            lastHoursTick += 1000;
        }
        // Bench test signal looks like a running engine: do not accumulate hours
        if (engineState.isEngineRunning && !benchSignal.isActive()) {
            storageManager.tickEngineHours(1);
            engineState.engineHoursSeconds = storageManager.getEngineHoursSeconds();
        }
    }
    // Persist hours as soon as the engine stops (power is often cut right after)
    if (wasEngineRunning && !engineState.isEngineRunning) {
        storageManager.saveEngineHoursIfNeeded(true);
    }
    wasEngineRunning = engineState.isEngineRunning;

    // 4. Alarm evaluation & persistent event logging
    alarmManager.update(engineState, storageManager);

    // 5. NMEA 2000 message dispatching & network reception
    n2kManager.update(engineState);

    // 6. Interactive CLI & periodic diagnostics
    handleCli();

    benchSignal.update(rpmSensor);

    if (adcDebugOutput && (now - lastStatusPrint >= 2000)) {
        lastStatusPrint = now;
        adcManager.printDebug();
    }
}
