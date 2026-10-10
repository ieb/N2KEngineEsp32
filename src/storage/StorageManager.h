#pragma once
#include <Arduino.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include "config.h"

class StorageManager {
public:
    StorageManager();
    bool begin();

    // Engine Hours Accumulation
    uint32_t getEngineHoursSeconds() const { return engineHoursSeconds; }
    void setEngineHoursSeconds(uint32_t seconds);
    void tickEngineHours(uint32_t secondsToAdd = 1);
    void saveEngineHoursIfNeeded(bool force = false);

    // Event History on SPIFFS
    void logEvent(uint8_t eventId, double engineHours, uint32_t utcTime);
    void clearEvents();
    void dumpEvents(Print &out);
    uint32_t getEventCount();

private:
    Preferences prefs;
    uint32_t engineHoursSeconds;
    unsigned long lastSaveTime;
    bool dirtyHours;
    uint32_t eventCount;

    const char *getEventName(uint8_t eventId);
};
