#include "AlarmManager.h"

AlarmManager::AlarmManager()
    : engineStartedTime(0),
      canEmitAlarms(false),
      wasRunning(false),
      coolantOverTempStart(0),
      lowOilPressureStart(0),
      lowBatteryVStart(0),
      lowAlternatorVStart(0),
      highExhaustStart(0),
      exhaustRiseAnchorTime(0),
      exhaustRiseAnchor(0),
      exhaustFlowTripped(false),
      loggedStop(false),
      loggedLowOil(false),
      loggedHighCoolant(false),
      loggedExhaustTemp(false),
      loggedAltTemp(false),
      loggedRoomTemp(false) {
}

bool AlarmManager::delayedTrigger(unsigned long &start, unsigned long windowMs) {
    if (start == 0) {
        start = millis();
        if (start == 0) start = 1; // 0 is the "not started" sentinel
        return false;
    }
    return ((millis() - start) >= windowMs);
}

void AlarmManager::reset() {
    engineStartedTime = 0;
    canEmitAlarms = false;
    wasRunning = false;
    coolantOverTempStart = 0;
    lowOilPressureStart = 0;
    lowBatteryVStart = 0;
    lowAlternatorVStart = 0;
    highExhaustStart = 0;
    exhaustRiseAnchorTime = 0;
    exhaustRiseAnchor = 0;
    exhaustFlowTripped = false;
    // Per-run event latches: an alarm active when the engine was stopped must be
    // logged again if it recurs on the next run.
    loggedStop = false;
    loggedLowOil = false;
    loggedHighCoolant = false;
    loggedExhaustTemp = false;
}

void AlarmManager::update(EngineState &state, StorageManager &storage) {
    unsigned long now = millis();
    double engineHours = (double)state.engineHoursSeconds / 3600.0;
    uint32_t utcTime = state.systemTimeUtc;

    // Track running transition
    bool justStopped = false;
    if (state.isEngineRunning && !wasRunning) {
        reset();
        wasRunning = true;
        engineStartedTime = now;
    } else if (!state.isEngineRunning && wasRunning) {
        wasRunning = false;
        canEmitAlarms = false;
        justStopped = true;
    }

    if (state.isEngineRunning && !canEmitAlarms) {
        if (now - engineStartedTime >= ENGINE_START_GRACE_PERIOD) {
            canEmitAlarms = true;
            Serial.println(F("[ALARMS] Grace period complete - active monitoring enabled"));
        }
    }

    uint16_t s1 = 0;
    uint16_t s2 = 0;

    // 1. Preheat Indicator (Active whenever glow plug relay coil is energized)
    if (state.glowRelayActive) {
        s1 |= STATUS1_PREHEAT_INDICATOR;
    }

    // 2. Engine Shutting Down (stop relay energised, or RPM run-down to rest)
    bool shuttingDown = state.stopRelayActive || state.isEngineStopping;
    if (shuttingDown) {
        s2 |= STATUS2_ENGINE_SHUTTING_DOWN;
    }
    // Log one ENGINE_STOP event per run (latch cleared by reset() at next start),
    // or once per stop-relay activation while the engine is not running.
    if ((shuttingDown || justStopped) && !loggedStop) {
        storage.logEvent(EVENT_ENGINE_STOP, engineHours, utcTime);
        loggedStop = true;
    } else if (!shuttingDown && !state.isEngineRunning && !justStopped) {
        loggedStop = false;
    }

    // Alarms requiring running engine and elapsed grace period
    if (state.isEngineRunning && canEmitAlarms && !state.isEngineStopping) {
        // 3. Oil Pressure
        if (state.oilPressurePa != N2kDoubleNA && state.oilPressurePa < MIN_OIL_PRESSURE_PA) {
            if (delayedTrigger(lowOilPressureStart, OIL_PRESS_ALARM_WINDOW_MS)) {
                s1 |= (STATUS1_LOW_OIL_PRES | STATUS1_CHECK_ENGINE);
                s2 |= STATUS2_MAINTENANCE_NEEDED;
                if (!loggedLowOil) {
                    storage.logEvent(EVENT_LOW_OIL_PRES, engineHours, utcTime);
                    loggedLowOil = true;
                }
            }
        } else {
            lowOilPressureStart = 0;
            loggedLowOil = false;
        }

        // 4. Coolant Temperature (> 97.0 C sustained > 15s)
        if (state.coolantTemp_dC > MAX_COOLANT_TEMP_DECI_C) {
            if (delayedTrigger(coolantOverTempStart, COOLANT_ALARM_WINDOW_MS)) {
                s1 |= (STATUS1_OVERTEMP | STATUS1_CHECK_ENGINE);
                s2 |= STATUS2_MAINTENANCE_NEEDED;
                if (!loggedHighCoolant) {
                    storage.logEvent(EVENT_HIGH_COOLANT, engineHours, utcTime);
                    loggedHighCoolant = true;
                }
            }
        } else {
            coolantOverTempStart = 0;
            loggedHighCoolant = false;
        }

        // 5. Battery Voltage (< 11.8V sustained > 5s)
        if (state.engineBatteryVoltage != N2kDoubleNA && state.engineBatteryVoltage < LOW_BATTERY_VOLTAGE) {
            if (delayedTrigger(lowBatteryVStart, VOLTAGE_ALARM_WINDOW_MS)) {
                s1 |= (STATUS1_LOW_SYSTEM_VOLTAGE | STATUS1_CHECK_ENGINE);
            }
        } else {
            lowBatteryVStart = 0;
        }

        // 6. Alternator Voltage (< 12.2V sustained > 5s)
        if (state.alternatorVoltage != N2kDoubleNA && state.alternatorVoltage < LOW_ALTERNATOR_VOLTAGE) {
            if (delayedTrigger(lowAlternatorVStart, VOLTAGE_ALARM_WINDOW_MS)) {
                s1 |= STATUS1_CHARGE_INDICATOR;
            }
        } else {
            lowAlternatorVStart = 0;
        }

        // 7. Raw Water Flow Collapse via Exhaust Elbow (NTC5) per enginesensors.cpp:720-800
        int16_t elbow = state.exhaustElbowTemp_dC;
        bool flowTrippedThisCycle = false;

        if (elbow != -9999) {
            // Absolute high temp trip (> 45.0 C)
            if (elbow > MAX_EXHAUST_TEMP_DECI_C) {
                s1 |= (STATUS1_WATER_FLOW | STATUS1_CHECK_ENGINE);
                s2 |= STATUS2_MAINTENANCE_NEEDED;
                if (delayedTrigger(highExhaustStart, HIGH_EXHAUST_WINDOW_MS)) {
                    s1 |= STATUS1_EMERGENCY_STOP;
                }
                flowTrippedThisCycle = true;
                if (!loggedExhaustTemp) {
                    storage.logEvent(EVENT_EXHAUST_TEMP, engineHours, utcTime);
                    loggedExhaustTemp = true;
                }
            } else {
                highExhaustStart = 0;
            }

            // Rate-of-rise check (> 8.0 C in 30s past 35.0 C baseline)
            if (elbow >= EXHAUST_BASELINE_TEMP_DECI_C) {
                if (exhaustRiseAnchorTime == 0 || elbow < exhaustRiseAnchor) {
                    exhaustRiseAnchor = elbow;
                    exhaustRiseAnchorTime = now;
                } else if ((now - exhaustRiseAnchorTime) <= EXHAUST_RISE_WINDOW_MS) {
                    if ((elbow - exhaustRiseAnchor) >= EXHAUST_RISE_DELTA_DECI_C) {
                        s1 |= (STATUS1_WATER_FLOW | STATUS1_CHECK_ENGINE);
                        s2 |= STATUS2_MAINTENANCE_NEEDED;
                        flowTrippedThisCycle = true;
                        if (!loggedExhaustTemp) {
                            storage.logEvent(EVENT_EXHAUST_TEMP, engineHours, utcTime);
                            loggedExhaustTemp = true;
                        }
                    }
                } else {
                    // Window expired without trip - re-anchor
                    exhaustRiseAnchor = elbow;
                    exhaustRiseAnchorTime = now;
                }
            } else {
                exhaustRiseAnchorTime = 0;
            }

            // Clear trip when safely below 38.0 C
            if (flowTrippedThisCycle) {
                exhaustFlowTripped = true;
            } else if (elbow < CLEAR_EXHAUST_TEMP_DECI_C) {
                exhaustFlowTripped = false;
                loggedExhaustTemp = false;
            }

            if (exhaustFlowTripped) {
                s1 |= (STATUS1_WATER_FLOW | STATUS1_CHECK_ENGINE);
                s2 |= STATUS2_MAINTENANCE_NEEDED;
            }
        }
    } else {
        // Engine not running / in grace
        highExhaustStart = 0;
        exhaustRiseAnchorTime = 0;
        lowOilPressureStart = 0;
        coolantOverTempStart = 0;
        lowBatteryVStart = 0;
        lowAlternatorVStart = 0;
    }

    // 8. Warning Level 1 (Alternator > 110 C or Engine Room > 70 C)
    if (state.alternatorTemp_dC > MAX_ALTERNATOR_TEMP_DECI_C) {
        s2 |= STATUS2_WARN_1;
        s1 |= STATUS1_CHECK_ENGINE;
        if (!loggedAltTemp) {
            storage.logEvent(EVENT_ALTERNATOR_TEMP, engineHours, utcTime);
            loggedAltTemp = true;
        }
    } else if (state.alternatorTemp_dC < CLEAR_ALTERNATOR_TEMP_DECI_C) {
        loggedAltTemp = false;
    }

    if (state.engineRoomTemp_dC > MAX_ENGINEROOM_TEMP_DECI_C) {
        s2 |= STATUS2_WARN_1;
        s1 |= STATUS1_CHECK_ENGINE;
        if (!loggedRoomTemp) {
            storage.logEvent(EVENT_ENGINE_ROOM_TEMP, engineHours, utcTime);
            loggedRoomTemp = true;
        }
    } else if (state.engineRoomTemp_dC < CLEAR_ENGINEROOM_TEMP_DECI_C) {
        loggedRoomTemp = false;
    }

    state.discreteStatus1 = s1;
    state.discreteStatus2 = s2;
}
