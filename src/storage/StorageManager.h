#pragma once
#include <Arduino.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include "config.h"

// Persistent storage for engine hours and the alarm event log.
//
// Flash writes stall every interrupt that is not in IRAM (TWAI/CAN, and BLE in
// future) for up to tens of milliseconds during a sector erase. To keep flash
// quiet while the engine is running:
//   * Engine hours and pending events are mirrored in RTC no-init memory, which
//     survives crashes, watchdog resets and software restarts (not power loss).
//   * While writes are deferred (engine running) nothing is written to NVS or
//     SPIFFS, except an optional long backstop save of engine hours
//     (ENGINE_HOURS_BACKSTOP_SAVE_S) against an abnormal power cut.
//   * flush() writes everything; it is called when the stop relay is energised,
//     when the engine stops (setDeferWrites(false)) and before a restart.
//   * At boot, a valid RTC copy that is ahead of NVS (i.e. after a crash) is
//     adopted and written to flash.
// User-initiated actions (set engine hours, clear events) write immediately.
class StorageManager {
public:
    StorageManager();
    bool begin();

    // Engine Hours Accumulation
    uint32_t getEngineHoursSeconds() const { return engineHoursSeconds; }
    void setEngineHoursSeconds(uint32_t seconds);
    void tickEngineHours(uint32_t secondsToAdd = 1);

    // Write deferral: true while the engine is running. Going false flushes.
    void setDeferWrites(bool defer);
    bool isDeferringWrites() const { return deferWrites; }
    // Writes dirty engine hours and pending events to flash now.
    void flush();

    // Event History on SPIFFS
    void logEvent(uint8_t eventId, double engineHours, uint32_t utcTime);
    void clearEvents();
    void dumpEvents(Print &out);
    uint32_t getEventCount();
    uint8_t getPendingEventCount() const;

    struct EventRecord {
        uint8_t eventId;
        double engineHours;
    };
    // Reads the most recent events (oldest first), including events not yet
    // written to flash, into out[]. Returns the number of records written.
    uint8_t getRecentEvents(EventRecord *out, uint8_t maxEvents);

    // Test support: invalidates the RTC copy (as after a power-on reset).
    static void resetRtcState();

private:
    Preferences prefs;
    uint32_t engineHoursSeconds;
    unsigned long lastSaveTime;
    bool dirtyHours;
    uint32_t eventCount;
    bool dirtyEventCount;
    bool deferWrites;

    const char *getEventName(uint8_t eventId);
    void saveEngineHours();
    void writeEventLine(uint32_t seq, uint8_t eventId, uint32_t hoursSeconds, uint32_t utcTime);
    void flushPendingEvents();
    void syncRtc();
};
