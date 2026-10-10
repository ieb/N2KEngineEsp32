#include "N2kManager.h"

StorageManager *N2kManager::s_storage = nullptr;
EngineState *N2kManager::s_state = nullptr;

static const unsigned long TransmitMessages[] PROGMEM = {
    127488L, // Engine Rapid
    127489L, // Engine Dynamic
    127493L, // Transmission Dynamic (Remapped to Exhaust)
    127508L, // Battery Status (Engine Batt + Alternator)
    127505L, // Fluid Level
    130312L, // Temperature (Legacy)
    130316L, // Extended Temperature
    0
};

static const unsigned long ReceiveMessages[] PROGMEM = {
    126992L, // System Time (GPS)
    PGN_PROPRIETARY_CMD, // 65305L
    0
};

N2kManager::N2kManager()
    : n2k((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX),
      storagePtr(nullptr),
      lastRapidTime(0),
      lastDynamicTime(0),
      lastBatteryTime(0),
      lastFuelTime(0),
      lastTransmissionTime(0),
      lastLegacyTempTime(0),
      lastExtTempTime(0),
      batterySid(0),
      tempSid(0) {
}

bool N2kManager::begin(StorageManager *storage) {
    storagePtr = storage;
    s_storage = storage;

    // Product information (PGN 126996)
    n2k.SetProductInformation("00000002",
                             44,
                             "EMS",
                             "1.2.3.4 (2026-10-10)",
                             "5.6.7.8 (2026-10-10)",
                             0,
                             1);

    // Device information (PGN 60928)
    n2k.SetDeviceInformation(N2K_DEVICE_ID,
                            N2K_DEVICE_FUNCTION, // 140 = Engine
                            N2K_DEVICE_CLASS,    // 50 = Propulsion
                            N2K_MANUFACTURER_CODE,
                            N2K_INDUSTRY_GROUP);

    // Configuration information (PGN 126998)
    n2k.SetConfigurationInformation("Luna Technical Area",
                                   "Engine Monitor",
                                   "https://github.com/ieb/N2KEngineEsp32");

    n2k.SetCANBufferSize(100, 40);
    n2k.SetN2kCANReceiveFrameBufSize(150);
    n2k.SetN2kCANMsgBufSize(8);

    n2k.ExtendTransmitMessages(TransmitMessages);
    n2k.ExtendReceiveMessages(ReceiveMessages);

    n2k.SetMsgHandler(N2kManager::handleIncomingMsg);
    n2k.SetMode(tNMEA2000::N2km_NodeOnly, N2K_DEFAULT_ADDRESS);

    bool opened = n2k.Open();
    if (opened) {
        Serial.println(F("[N2K] NMEA2000 stack opened on TWAI driver"));
    } else {
        Serial.println(F("[N2K] Failed to open NMEA2000 stack!"));
    }
    return opened;
}

void N2kManager::handleIncomingMsg(const tN2kMsg &msg) {
    // 1. GPS System Time (PGN 126992)
    if (msg.PGN == 126992L) {
        unsigned char sid;
        uint16_t systemDate;
        double systemTime;
        tN2kTimeSource source;
        if (ParseN2kSystemTime(msg, sid, systemDate, systemTime, source)) {
            if (s_state && systemDate > 0) {
                s_state->systemTimeUtc = (uint32_t)(systemDate * 86400UL + (uint32_t)systemTime);
            }
        }
    }
    // 2. Proprietary Command (PGN 65305)
    else if (msg.PGN == PGN_PROPRIETARY_CMD && msg.DataLen >= 3) {
        uint16_t mfgCode = (((uint16_t)msg.Data[1]) << 8) | msg.Data[0];
        if (mfgCode == PROPRIETARY_CODE) {
            uint8_t fn = msg.Data[2];
            if (fn == FN_CLEAR_EVENTS && s_storage) {
                s_storage->clearEvents();
                // Send clear response confirmation
                tN2kMsg resp;
                resp.SetPGN(PGN_PROPRIETARY_CMD);
                resp.Priority = 6;
                resp.DataLen = 3;
                resp.Data[0] = PROPRIETARY_CODE & 0xFF;
                resp.Data[1] = (PROPRIETARY_CODE >> 8) & 0xFF;
                resp.Data[2] = FN_CLEAR_EVENTS_RESP;
                // Broadcast response
                // (response can be sent if needed)
            }
        }
    }
}

void N2kManager::sendRapidEngineData(const EngineState &state) {
    tN2kMsg msg;
    SetN2kEngineParamRapid(msg,
                           ENGINE_INSTANCE,
                           state.engineRpm,
                           N2kDoubleNA,
                           N2kInt8NA);
    n2k.SendMsg(msg);
}

void N2kManager::sendEngineDynamic(const EngineState &state) {
    tN2kMsg msg;
    // Remap: Alternator temperature sent as EngineOilTemp so MFD displays alternator load
    SetN2kEngineDynamicParam(msg,
                             ENGINE_INSTANCE,
                             state.oilPressurePa,
                             state.alternatorTempK,      // [REMAPPED] Alternator Temp
                             state.coolantTempK,
                             state.alternatorVoltage,
                             N2kDoubleNA,                // Fuel Rate
                             state.engineHoursSeconds,   // Engine Hours in seconds
                             N2kDoubleNA,                // Coolant Pressure
                             N2kDoubleNA,                // Fuel Pressure
                             N2kInt8NA,                  // Engine Load
                             N2kInt8NA,                  // Engine Torque
                             (tN2kEngineDiscreteStatus1)state.discreteStatus1,
                             (tN2kEngineDiscreteStatus2)state.discreteStatus2);
    n2k.SendMsg(msg);
}

void N2kManager::sendBatteryStatus(const EngineState &state) {
    tN2kMsg msg0;
    // Instance 0: Engine Battery
    SetN2kDCBatStatus(msg0,
                      ENGINE_BATTERY_INSTANCE,
                      state.engineBatteryVoltage,
                      N2kDoubleNA,
                      N2kDoubleNA,
                      batterySid);
    n2k.SendMsg(msg0);

    // Instance 2: Alternator Voltage & Alternator Temp [REMAPPED]
    tN2kMsg msg2;
    SetN2kDCBatStatus(msg2,
                      ALTERNATOR_BATTERY_INSTANCE,
                      state.alternatorVoltage,
                      N2kDoubleNA,
                      state.alternatorTempK,
                      batterySid);
    n2k.SendMsg(msg2);

    batterySid++;
}

void N2kManager::sendFluidLevel(const EngineState &state) {
    tN2kMsg msg;
    SetN2kFluidLevel(msg,
                     FUEL_TANK_INSTANCE,
                     N2kft_Fuel,
                     state.fuelLevelPercent,
                     state.fuelCapacityLitres);
    n2k.SendMsg(msg);
}

void N2kManager::sendTransmissionDynamic(const EngineState &state) {
    tN2kMsg msg;
    // Remapped: Transmission oil temp carried from NTC1 (Exhaust Water Trap)
    SetN2kTransmissionParameters(msg,
                                 ENGINE_INSTANCE,
                                 N2kTG_Unknown,
                                 N2kDoubleNA,
                                 state.exhaustWaterTrapK, // [REMAPPED] Exhaust Temp
                                 0);
    n2k.SendMsg(msg);
}

void N2kManager::sendLegacyTemperatures(const EngineState &state) {
    tN2kMsg msg;

    // 1. Exhaust Gas (Source 14): NTC1 Exhaust Water Trap
    if (state.exhaustWaterTrapK != N2kDoubleNA) {
        SetN2kTemperature(msg, tempSid, 0, N2kts_ExhaustGasTemperature, state.exhaustWaterTrapK);
        n2k.SendMsg(msg);
    }

    // 2. Engine Room (Source 3): NTC3
    if (state.engineRoomTempK != N2kDoubleNA) {
        SetN2kTemperature(msg, tempSid, 0, N2kts_EngineRoomTemperature, state.engineRoomTempK);
        n2k.SendMsg(msg);
    }

    // 3. Alternator (Source 30): NTC2
    if (state.alternatorTempK != N2kDoubleNA) {
        SetN2kTemperature(msg, tempSid, 0, (tN2kTempSource)30, state.alternatorTempK);
        n2k.SendMsg(msg);
    }

    // 4. Dallas 1-Wire (Source 31+i)
    for (uint8_t i = 0; i < state.oneWireCount; i++) {
        if (state.oneWireTempK[i] != N2kDoubleNA) {
            SetN2kTemperature(msg, tempSid, 0, (tN2kTempSource)(31 + i), state.oneWireTempK[i]);
            n2k.SendMsg(msg);
        }
    }
}

void N2kManager::sendExtendedTemperatures(const EngineState &state) {
    tN2kMsg msg;

    // 1. Exhaust Gas (Source 14)
    if (state.exhaustWaterTrapK != N2kDoubleNA) {
        SetN2kTemperatureExt(msg, tempSid, 0, N2kts_ExhaustGasTemperature, state.exhaustWaterTrapK);
        n2k.SendMsg(msg);
    }

    // 2. Engine Room (Source 3)
    if (state.engineRoomTempK != N2kDoubleNA) {
        SetN2kTemperatureExt(msg, tempSid, 0, N2kts_EngineRoomTemperature, state.engineRoomTempK);
        n2k.SendMsg(msg);
    }

    // 3. Alternator (Source 30)
    if (state.alternatorTempK != N2kDoubleNA) {
        SetN2kTemperatureExt(msg, tempSid, 0, (tN2kTempSource)30, state.alternatorTempK);
        n2k.SendMsg(msg);
    }

    tempSid++;
}

void N2kManager::update(EngineState &state) {
    s_state = &state;
    n2k.ParseMessages();

    unsigned long now = millis();

    // 1. Rapid Engine Updates (PGN 127488): Every 500ms when running (RPM >= 800)
    if (state.isEngineRunning && (now - lastRapidTime >= PERIOD_RAPID_ENGINE_MS)) {
        lastRapidTime = now;
        sendRapidEngineData(state);
    }

    // 2. Dynamic Engine Updates (PGN 127489): Every 1000ms when running (RPM >= 800)
    if (state.isEngineRunning && (now - lastDynamicTime >= PERIOD_ENGINE_DYNAMIC_MS)) {
        lastDynamicTime = now;
        sendEngineDynamic(state);
    }

    // 3. Battery Status (PGN 127508): Every 1000ms continuous
    if (now - lastBatteryTime >= PERIOD_BATTERY_STATUS_MS) {
        lastBatteryTime = now;
        sendBatteryStatus(state);
    }

    // 4. Fluid Level (PGN 127505): Every 2500ms continuous
    if (now - lastFuelTime >= PERIOD_FLUID_LEVEL_MS) {
        lastFuelTime = now;
        sendFluidLevel(state);
    }

    // 5. Extended Temperature (PGN 130316): Every 2500ms continuous
    if (now - lastExtTempTime >= PERIOD_EXT_TEMPERATURE_MS) {
        lastExtTempTime = now;
        sendExtendedTemperatures(state);
    }

    // 6. Transmission Dynamic (PGN 127493): Every 4850ms continuous (Remapped to Exhaust)
    if (now - lastTransmissionTime >= PERIOD_TRANSMISSION_MS) {
        lastTransmissionTime = now;
        sendTransmissionDynamic(state);
    }

    // 7. Legacy Temperature (PGN 130312): Every 4850ms continuous
    if (now - lastLegacyTempTime >= PERIOD_LEGACY_TEMP_MS) {
        lastLegacyTempTime = now;
        sendLegacyTemperatures(state);
    }
}
