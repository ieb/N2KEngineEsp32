#include "StorageManager.h"
#include <FS.h>
#include <time.h>
#include <algorithm>
#include <cstddef>
#include <cstring>

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#endif
#ifndef RTC_NOINIT_ATTR
#define RTC_NOINIT_ATTR
#endif

#define EVENTS_FILE_PATH "/events.csv"
#define RTC_MAGIC        0x454E4731UL // "ENG1"

// ----------------------------------------------------------------------------
// RTC no-init mirror. Survives software resets, panics and watchdog resets on
// the ESP32-C3; contents are random after power-on, hence magic + checksum.
// ----------------------------------------------------------------------------
struct RtcEvent {
    uint32_t seq;
    uint32_t hoursSeconds;
    uint32_t utc;
    uint8_t id;
    uint8_t pad[3];
};

struct RtcBlock {
    uint32_t magic;
    uint32_t hoursSeconds;
    uint32_t eventCount;
    uint8_t pendingCount;
    uint8_t pad[3];
    RtcEvent pending[MAX_PENDING_EVENTS];
    uint32_t checksum;
};

RTC_NOINIT_ATTR static RtcBlock s_rtc;

static uint32_t rtcChecksum() {
    // FNV-1a over everything except the checksum field
    const uint8_t *p = (const uint8_t *)&s_rtc;
    size_t len = offsetof(RtcBlock, checksum);
    uint32_t h = 2166136261UL;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619UL;
    }
    return h;
}

static bool rtcValid() {
    return s_rtc.magic == RTC_MAGIC &&
           s_rtc.pendingCount <= MAX_PENDING_EVENTS &&
           s_rtc.checksum == rtcChecksum();
}

void StorageManager::resetRtcState() {
    memset(&s_rtc, 0, sizeof(s_rtc));
}

void StorageManager::syncRtc() {
    s_rtc.magic = RTC_MAGIC;
    s_rtc.hoursSeconds = engineHoursSeconds;
    s_rtc.eventCount = eventCount;
    s_rtc.checksum = rtcChecksum();
}

StorageManager::StorageManager()
    : engineHoursSeconds(0),
      lastSaveTime(0),
      dirtyHours(false),
      eventCount(0),
      dirtyEventCount(false),
      deferWrites(false) {
}

bool StorageManager::begin() {
    bool spiffsOk = SPIFFS.begin(true);
    if (!spiffsOk) {
        Serial.println(F("[STORAGE] Failed to mount SPIFFS filesystem!"));
    } else {
        Serial.printf("[STORAGE] SPIFFS mounted. Total: %u KB, Used: %u KB\n",
                      (unsigned)(SPIFFS.totalBytes() / 1024), (unsigned)(SPIFFS.usedBytes() / 1024));
    }

    // Load Engine Hours from Preferences
    prefs.begin("n2k_engine", false);
    engineHoursSeconds = prefs.getUInt("hours_sec", 0);
    eventCount = prefs.getUInt("evt_count", 0);
    dirtyHours = false;
    dirtyEventCount = false;
    deferWrites = false;

    // Recover anything newer from the RTC mirror (crash / watchdog / restart)
    if (rtcValid()) {
        if (s_rtc.hoursSeconds > engineHoursSeconds) {
            Serial.printf("[STORAGE] Recovered %u s of engine hours from RTC memory\n",
                          (unsigned)(s_rtc.hoursSeconds - engineHoursSeconds));
            engineHoursSeconds = s_rtc.hoursSeconds;
            dirtyHours = true;
        }
        if (s_rtc.eventCount > eventCount) {
            eventCount = s_rtc.eventCount;
            dirtyEventCount = true;
        }
        if (s_rtc.pendingCount > 0) {
            Serial.printf("[STORAGE] Recovered %u pending event(s) from RTC memory\n",
                          (unsigned)s_rtc.pendingCount);
        }
    } else {
        memset(&s_rtc, 0, sizeof(s_rtc));
    }
    syncRtc();
    flush();
    lastSaveTime = millis();

    Serial.printf("[STORAGE] Initial Engine Hours: %.2f h (%u sec)\n",
                  (double)engineHoursSeconds / 3600.0, (unsigned)engineHoursSeconds);
    return spiffsOk;
}

void StorageManager::setEngineHoursSeconds(uint32_t seconds) {
    engineHoursSeconds = seconds;
    prefs.putUInt("hours_sec", engineHoursSeconds);
    dirtyHours = false;
    lastSaveTime = millis();
    syncRtc();
    Serial.printf("[STORAGE] Engine Hours set to: %.2f h\n", (double)engineHoursSeconds / 3600.0);
}

void StorageManager::tickEngineHours(uint32_t secondsToAdd) {
    engineHoursSeconds += secondsToAdd;
    dirtyHours = true;
    syncRtc();
    // Backstop only: protects against an abnormal power cut while running.
#if ENGINE_HOURS_BACKSTOP_SAVE_S > 0
    if (millis() - lastSaveTime >= (unsigned long)ENGINE_HOURS_BACKSTOP_SAVE_S * 1000UL) {
        saveEngineHours();
    }
#endif
}

void StorageManager::saveEngineHours() {
    if (dirtyHours) {
        prefs.putUInt("hours_sec", engineHoursSeconds);
        dirtyHours = false;
    }
    lastSaveTime = millis();
}

void StorageManager::setDeferWrites(bool defer) {
    if (deferWrites && !defer) {
        deferWrites = false;
        flush();
    } else {
        deferWrites = defer;
    }
}

void StorageManager::flush() {
    saveEngineHours();
    flushPendingEvents();
    if (dirtyEventCount) {
        prefs.putUInt("evt_count", eventCount);
        dirtyEventCount = false;
    }
}

uint8_t StorageManager::getPendingEventCount() const {
    return s_rtc.pendingCount;
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

static void formatEventLine(char *buf, size_t size, uint32_t seq, uint8_t eventId,
                            const char *name, uint32_t hoursSeconds, uint32_t utcTime) {
    // Format: id, event_id, event_name, engine_hours, utc_epoch, utc_iso
    char isoBuffer[32] = "N/A";
    if (utcTime > 0) {
        time_t raw = (time_t)utcTime;
        struct tm *ti = gmtime(&raw);
        if (ti) {
            strftime(isoBuffer, sizeof(isoBuffer), "%Y-%m-%d %H:%M:%SZ", ti);
        }
    }
    snprintf(buf, size, "%u,%u,%s,%.3f,%u,%s",
             (unsigned)seq, (unsigned)eventId, name,
             (double)hoursSeconds / 3600.0, (unsigned)utcTime, isoBuffer);
}

void StorageManager::writeEventLine(uint32_t seq, uint8_t eventId, uint32_t hoursSeconds, uint32_t utcTime) {
    File file = SPIFFS.open(EVENTS_FILE_PATH, FILE_APPEND);
    if (!file) {
        Serial.println(F("[STORAGE] Failed to open events file for writing!"));
        return;
    }
    char line[96];
    formatEventLine(line, sizeof(line), seq, eventId, getEventName(eventId), hoursSeconds, utcTime);
    file.printf("%s\n", line);
    file.close();
}

void StorageManager::flushPendingEvents() {
    uint8_t n = s_rtc.pendingCount;
    if (n == 0) return;
    File file = SPIFFS.open(EVENTS_FILE_PATH, FILE_APPEND);
    if (!file) {
        Serial.println(F("[STORAGE] Failed to open events file; pending events kept"));
        return;
    }
    char line[96];
    for (uint8_t i = 0; i < n; i++) {
        const RtcEvent &e = s_rtc.pending[i];
        formatEventLine(line, sizeof(line), e.seq, e.id, getEventName(e.id), e.hoursSeconds, e.utc);
        file.printf("%s\n", line);
    }
    file.close();
    s_rtc.pendingCount = 0;
    syncRtc();
    Serial.printf("[STORAGE] Wrote %u pending event(s) to flash\n", (unsigned)n);
}

void StorageManager::logEvent(uint8_t eventId, double engineHours, uint32_t utcTime) {
    if (eventId == EVENT_NONE) return;

    eventCount++;
    uint32_t hoursSeconds = (uint32_t)(engineHours * 3600.0 + 0.5);

    if (deferWrites) {
        if (s_rtc.pendingCount >= MAX_PENDING_EVENTS) {
            // Should not happen (alarms latch once per run); keep the data anyway.
            Serial.println(F("[STORAGE] Pending event buffer full, writing to flash now"));
            flushPendingEvents();
        }
        RtcEvent &e = s_rtc.pending[s_rtc.pendingCount];
        e.seq = eventCount;
        e.hoursSeconds = hoursSeconds;
        e.utc = utcTime;
        e.id = eventId;
        s_rtc.pendingCount++;
        dirtyEventCount = true;
        syncRtc();
        Serial.printf("[STORAGE] Event #%u: %s at %.3f hrs (pending until engine stops)\n",
                      (unsigned)eventCount, getEventName(eventId), engineHours);
        return;
    }

    writeEventLine(eventCount, eventId, hoursSeconds, utcTime);
    prefs.putUInt("evt_count", eventCount);
    dirtyEventCount = false;
    syncRtc();
    Serial.printf("[STORAGE] Logged event #%u: %s at %.3f hrs\n",
                  (unsigned)eventCount, getEventName(eventId), engineHours);
}

void StorageManager::clearEvents() {
    if (SPIFFS.exists(EVENTS_FILE_PATH)) {
        SPIFFS.remove(EVENTS_FILE_PATH);
    }
    eventCount = 0;
    s_rtc.pendingCount = 0;
    prefs.putUInt("evt_count", 0);
    dirtyEventCount = false;
    syncRtc();
    Serial.println(F("[STORAGE] All stored events cleared."));
}

void StorageManager::dumpEvents(Print &out) {
    out.println(F("--- Stored Event Log ---"));
    out.printf("Total logged events: %u\n", (unsigned)eventCount);
    if (!SPIFFS.exists(EVENTS_FILE_PATH)) {
        out.println(F("No events file found on SPIFFS."));
    } else {
        File file = SPIFFS.open(EVENTS_FILE_PATH, FILE_READ);
        if (!file) {
            out.println(F("Error opening events file."));
        } else {
            while (file.available()) {
                out.write(file.read());
            }
            file.close();
        }
    }
    char line[96];
    for (uint8_t i = 0; i < s_rtc.pendingCount; i++) {
        const RtcEvent &e = s_rtc.pending[i];
        formatEventLine(line, sizeof(line), e.seq, e.id, getEventName(e.id), e.hoursSeconds, e.utc);
        out.printf("%s (pending)\n", line);
    }
    out.println(F("--- End of Log ---"));
}

uint32_t StorageManager::getEventCount() {
    return eventCount;
}

uint8_t StorageManager::getRecentEvents(EventRecord *out, uint8_t maxEvents) {
    if (maxEvents == 0) return 0;
    File file;
    if (SPIFFS.exists(EVENTS_FILE_PATH)) {
        file = SPIFFS.open(EVENTS_FILE_PATH, FILE_READ);
    }

    // Keep the last maxEvents records in a ring buffer while streaming the file.
    uint32_t total = 0;
    char line[96];
    size_t len = 0;
    bool more = true;
    while (more) {
        int c = (file && file.available()) ? file.read() : -1;
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
    if (file) file.close();

    // Events not yet written to flash are the newest
    for (uint8_t i = 0; i < s_rtc.pendingCount; i++) {
        EventRecord &rec = out[total % maxEvents];
        rec.eventId = s_rtc.pending[i].id;
        rec.engineHours = (double)s_rtc.pending[i].hoursSeconds / 3600.0;
        total++;
    }

    if (total <= maxEvents) return (uint8_t)total;
    // Rotate so the oldest retained record is first
    uint8_t start = total % maxEvents;
    std::rotate(out, out + start, out + maxEvents);
    return maxEvents;
}
