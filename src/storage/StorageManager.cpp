#include "StorageManager.h"
#include <FS.h>
#include <time.h>
#include <algorithm>

#define EVENTS_FILE_PATH "/events.csv"

StorageManager::StorageManager()
    : engineHoursSeconds(0),
      lastSaveTime(0),
      dirtyHours(false),
      eventCount(0) {
}

bool StorageManager::begin() {
    bool spiffsOk = SPIFFS.begin(true);
    if (!spiffsOk) {
        Serial.println(F("[STORAGE] Failed to mount SPIFFS filesystem!"));
    } else {
        Serial.printf("[STORAGE] SPIFFS mounted. Total: %u KB, Used: %u KB\n",
                      SPIFFS.totalBytes() / 1024, SPIFFS.usedBytes() / 1024);
    }

    // Load Engine Hours from Preferences
    prefs.begin("n2k_engine", false);
    engineHoursSeconds = prefs.getUInt("hours_sec", 0);
    eventCount = prefs.getUInt("evt_count", 0);
    Serial.printf("[STORAGE] Initial Engine Hours: %.2f h (%u sec)\n",
                  (double)engineHoursSeconds / 3600.0, engineHoursSeconds);

    return spiffsOk;
}

void StorageManager::setEngineHoursSeconds(uint32_t seconds) {
    engineHoursSeconds = seconds;
    prefs.putUInt("hours_sec", engineHoursSeconds);
    dirtyHours = false;
    Serial.printf("[STORAGE] Engine Hours set to: %.2f h\n", (double)engineHoursSeconds / 3600.0);
}

void StorageManager::tickEngineHours(uint32_t secondsToAdd) {
    engineHoursSeconds += secondsToAdd;
    dirtyHours = true;
    saveEngineHoursIfNeeded(false);
}

void StorageManager::saveEngineHoursIfNeeded(bool force) {
    unsigned long now = millis();
    // Save to NVS every 60 seconds while running or when forced
    if (dirtyHours && (force || (now - lastSaveTime >= 60000UL))) {
        prefs.putUInt("hours_sec", engineHoursSeconds);
        dirtyHours = false;
        lastSaveTime = now;
    }
}

const char *StorageManager::getEventName(uint8_t eventId) {
    switch (eventId) {
        case EVENT_ENGINE_STOP:      return "ENGINE_STOP";
        case EVENT_LOW_OIL_PRES:     return "LOW_OIL_PRESSURE";
        case EVENT_HIGH_COOLANT:     return "HIGH_COOLANT_TEMP";
        case EVENT_EXHAUST_TEMP:     return "EXHAUST_OVERTEMP_WATER_FLOW";
        case EVENT_ALTERNATOR_TEMP:  return "ALTERNATOR_OVERTEMP";
        case EVENT_ENGINE_ROOM_TEMP: return "ENGINE_ROOM_OVERTEMP";
        default:                     return "UNKNOWN_EVENT";
    }
}

void StorageManager::logEvent(uint8_t eventId, double engineHours, uint32_t utcTime) {
    if (eventId == EVENT_NONE) return;

    eventCount++;
    prefs.putUInt("evt_count", eventCount);

    File file = SPIFFS.open(EVENTS_FILE_PATH, FILE_APPEND);
    if (!file) {
        Serial.println(F("[STORAGE] Failed to open events file for writing!"));
        return;
    }

    // Format: id, event_id, event_name, engine_hours, utc_epoch, utc_iso
    char isoBuffer[32] = "N/A";
    if (utcTime > 0) {
        time_t raw = (time_t)utcTime;
        struct tm *ti = gmtime(&raw);
        if (ti) {
            strftime(isoBuffer, sizeof(isoBuffer), "%Y-%m-%d %H:%M:%SZ", ti);
        }
    }

    file.printf("%u,%u,%s,%.3f,%u,%s\n",
                eventCount,
                eventId,
                getEventName(eventId),
                engineHours,
                utcTime,
                isoBuffer);
    file.close();

    Serial.printf("[STORAGE] Logged event #%u: %s at %.3f hrs (UTC: %s)\n",
                  eventCount, getEventName(eventId), engineHours, isoBuffer);
}

void StorageManager::clearEvents() {
    if (SPIFFS.exists(EVENTS_FILE_PATH)) {
        SPIFFS.remove(EVENTS_FILE_PATH);
    }
    eventCount = 0;
    prefs.putUInt("evt_count", 0);
    Serial.println(F("[STORAGE] All stored events cleared."));
}

void StorageManager::dumpEvents(Print &out) {
    out.println(F("--- Stored Event Log ---"));
    out.printf("Total logged events: %u\n", eventCount);
    if (!SPIFFS.exists(EVENTS_FILE_PATH)) {
        out.println(F("No events file found on SPIFFS."));
        return;
    }
    File file = SPIFFS.open(EVENTS_FILE_PATH, FILE_READ);
    if (!file) {
        out.println(F("Error opening events file."));
        return;
    }
    while (file.available()) {
        out.write(file.read());
    }
    file.close();
    out.println(F("--- End of Log ---"));
}

uint32_t StorageManager::getEventCount() {
    return eventCount;
}

uint8_t StorageManager::getRecentEvents(EventRecord *out, uint8_t maxEvents) {
    if (maxEvents == 0 || !SPIFFS.exists(EVENTS_FILE_PATH)) return 0;
    File file = SPIFFS.open(EVENTS_FILE_PATH, FILE_READ);
    if (!file) return 0;

    // Keep the last maxEvents records in a ring buffer while streaming the file.
    uint32_t total = 0;
    char line[96];
    size_t len = 0;
    bool more = true;
    while (more) {
        int c = file.available() ? file.read() : -1;
        if (c < 0) more = false;
        if (c == '\n' || c < 0) {
            line[len] = '\0';
            unsigned int seq = 0, id = 0;
            char name[48];
            double hours = 0.0;
            // Format: seq,event_id,event_name,engine_hours,...
            if (len > 0 && sscanf(line, "%u,%u,%47[^,],%lf", &seq, &id, name, &hours) == 4) {
                EventRecord &rec = out[total % maxEvents];
                rec.eventId = (uint8_t)id;
                rec.engineHours = hours;
                total++;
            }
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)c;
        }
    }
    file.close();

    if (total <= maxEvents) return (uint8_t)total;
    // Rotate so the oldest retained record is first
    uint8_t start = total % maxEvents;
    std::rotate(out, out + start, out + maxEvents);
    return maxEvents;
}
