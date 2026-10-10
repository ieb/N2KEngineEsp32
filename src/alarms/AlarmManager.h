#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors/EngineState.h"
#include "storage/StorageManager.h"

class AlarmManager {
public:
    AlarmManager();
    void update(EngineState &state, StorageManager &storage);
    void reset();

private:
    bool delayedTrigger(unsigned long &start, unsigned long windowMs);

    unsigned long engineStartedTime;
    bool canEmitAlarms;
    bool wasRunning;

    // Timers for sustained alarm conditions
    unsigned long coolantOverTempStart;
    unsigned long lowOilPressureStart;
    unsigned long lowBatteryVStart;
    unsigned long lowAlternatorVStart;
    unsigned long highExhaustStart;

    // Rate-of-rise anchor for exhaust elbow
    unsigned long exhaustRiseAnchorTime;
    int16_t exhaustRiseAnchor;
    bool exhaustFlowTripped;

    // Latch flags for event logging (log once per alarm activation)
    bool loggedStop;
    bool loggedLowOil;
    bool loggedHighCoolant;
    bool loggedExhaustTemp;
    bool loggedAltTemp;
    bool loggedRoomTemp;
};
