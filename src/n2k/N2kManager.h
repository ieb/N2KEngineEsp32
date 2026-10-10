#pragma once
#include <Arduino.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include "config.h"
#include "sensors/EngineState.h"
#include "storage/StorageManager.h"

class N2kManager {
public:
    N2kManager();
    bool begin(StorageManager *storage);
    void update(EngineState &state);

    static void handleIncomingMsg(const tN2kMsg &msg);

private:
    void sendRapidEngineData(const EngineState &state);
    void sendEngineDynamic(const EngineState &state);
    void sendBatteryStatus(const EngineState &state);
    void sendFluidLevel(const EngineState &state);
    void sendTransmissionDynamic(const EngineState &state);
    void sendLegacyTemperatures(const EngineState &state);
    void sendExtendedTemperatures(const EngineState &state);

    tNMEA2000_esp32 n2k;
    StorageManager *storagePtr;

    // Transmission scheduling timers
    unsigned long lastRapidTime;
    unsigned long lastDynamicTime;
    unsigned long lastBatteryTime;
    unsigned long lastFuelTime;
    unsigned long lastTransmissionTime;
    unsigned long lastLegacyTempTime;
    unsigned long lastExtTempTime;

    uint8_t batterySid;
    uint8_t tempSid;

    static StorageManager *s_storage;
    static EngineState *s_state;
};
