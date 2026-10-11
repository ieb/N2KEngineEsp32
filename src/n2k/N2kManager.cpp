#include "N2kManager.h"
#include "version.h"

StorageManager *N2kManager::s_storage = nullptr;
EngineState *N2kManager::s_state = nullptr;
N2kManager *N2kManager::s_instance = nullptr;

static const unsigned long TransmitMessages[] PROGMEM = {
    127488L, // Engine Rapid
    127489L, // Engine Dynamic
    127493L, // Transmission Dynamic (Remapped to Exhaust)
    127508L, // Battery Status (Engine Batt + Alternator)
    127505L, // Fluid Level
    130312L, // Temperature (Legacy)
    130316L, // Extended Temperature
    PGN_PROPRIETARY_CMD,  // 65305 (clear events response)
    PGN_PROPRIETARY_FAST, // 130817 (event dump response)
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
    s_instance = this;

    // Unique number (21 bits) derived from the ESP32 MAC so two boards never clash
    uint64_t mac = ESP.getEfuseMac();
    uint32_t uniqueId = (uint32_t)((mac >> 24) ^ mac) & 0x1FFFFF;
    if (uniqueId == 0) uniqueId = N2K_DEVICE_ID;

    char serialCode[16];
    snprintf(serialCode, sizeof(serialCode), "%07lu", (unsigned long)uniqueId);
    char swCode[32];
    snprintf(swCode, sizeof(swCode), "%.12s%s", GIT_SHA1_VERSION, GIT_LOCAL_CHANGES > 0 ? "+" : "");

    // Product information (PGN 126996)
    n2k.SetProductInformation(serialCode,
                             44,       // Product code
                             "EMS",    // Model ID
                             swCode,   // Software version (git sha)
                             "1.0",    // Model version
                             1,        // Load equivalency
                             2101,     // N2K version
                             0);       // Certification level

    // Device information (PGN 60928)
    n2k.SetDeviceInformation(uniqueId,
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
            if (s_state && systemDate > 0 && systemDate != N2kUInt16NA && systemTime != N2kDoubleNA) {
                s_state->systemTimeUtc = (uint32_t)(systemDate * 86400UL + (uint32_t)systemTime);
            }
        }
    }
    // 2. Proprietary Command (PGN 65305)
    else if (msg.PGN == PGN_PROPRIETARY_CMD && msg.DataLen >= 3 && s_instance) {
        uint16_t mfgCode = (((uint16_t)msg.Data[1]) << 8) | msg.Data[0];
        if (mfgCode == PROPRIETARY_CODE) {
            uint8_t fn = msg.Data[2];
            if (fn == FN_CLEAR_EVENTS && s_storage) {
                s_storage->clearEvents();
                s_instance->sendClearEventsResponse();
            } else if (fn == FN_DUMP_EVENTS && s_storage) {
                s_instance->sendEventDump();
            }
        }
    }
}

void N2kManager::sendClearEventsResponse() {
    tN2kMsg resp;
    resp.SetPGN(PGN_PROPRIETARY_CMD);
    resp.Priority = 6;
    resp.Add2ByteUInt(PROPRIETARY_CODE);
    resp.AddByte(FN_CLEAR_EVENTS_RESP);
    n2k.SendMsg(resp);
}

static void addUInt24(tN2kMsg &msg, double hours) {
    // 0.001 h per bit, clamped to the 24-bit range
    double v = hours * 1000.0;
    if (v < 0.0) v = 0.0;
    if (v > 16777215.0) v = 16777215.0;
    uint32_t u = (uint32_t)v;
    msg.AddByte(u & 0xFF);
    msg.AddByte((u >> 8) & 0xFF);
    msg.AddByte((u >> 16) & 0xFF);
}

void N2kManager::sendEventDump() {
    StorageManager::EventRecord events[MAX_DUMP_EVENTS];
    uint8_t count = s_storage->getRecentEvents(events, MAX_DUMP_EVENTS);

    tN2kMsg msg;
    msg.SetPGN(PGN_PROPRIETARY_FAST);
    msg.Priority = 6;
    msg.Add2ByteUInt(PROPRIETARY_CODE);
    msg.AddByte(FN_DUMP_EVENTS_RESP);
    msg.AddByte(count + 1);  // Number of events + 1 (per spec)
    msg.AddByte(0);          // Reserved
    addUInt24(msg, (double)s_storage->getEngineHoursSeconds() / 3600.0);
    for (uint8_t i = 0; i < count; i++) {
        msg.AddByte(events[i].eventId);
        addUInt24(msg, events[i].engineHours);
    }
    n2k.SendMsg(msg);
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
    // Remapped: Transmission oil temp carries exhaust temperature (NTC1 water trap,
    // falling back to NTC5 exhaust elbow when NTC1 is unavailable)
    double exhaustK = state.exhaustWaterTrapK != N2kDoubleNA ? state.exhaustWaterTrapK
                                                              : state.exhaustElbowTempK;
    SetN2kTransmissionParameters(msg,
                                 ENGINE_INSTANCE,
                                 N2kTG_Unknown,
                                 N2kDoubleNA,
                                 exhaustK, // [REMAPPED] Exhaust Temp
                                 0);
    n2k.SendMsg(msg);
}

// Builds the list of temperature channels shared by PGN 130312 and 130316
uint8_t N2kManager::collectTemperatures(const EngineState &state, TempChannel *out) {
    uint8_t n = 0;
    auto add = [&](uint8_t instance, uint8_t source, double k) {
        if (k != N2kDoubleNA) {
            out[n].instance = instance;
            out[n].source = source;
            out[n].kelvin = k;
            n++;
        }
    };
    add(TEMP_INSTANCE_EXHAUST_TRAP,  N2kts_ExhaustGasTemperature, state.exhaustWaterTrapK); // NTC1
    add(TEMP_INSTANCE_EXHAUST_ELBOW, N2kts_ExhaustGasTemperature, state.exhaustElbowTempK); // NTC5
    add(0, N2kts_EngineRoomTemperature, state.engineRoomTempK);                              // NTC3
    add(0, TEMP_SOURCE_ALTERNATOR, state.alternatorTempK);                                   // NTC2
    add(0, TEMP_SOURCE_GEARBOX, state.gearboxTempK);                                         // NTC4
    for (uint8_t i = 0; i < state.oneWireCount && i < 4; i++) {
        add(0, TEMP_SOURCE_ONEWIRE_BASE + i, state.oneWireTempK[i]);                         // 1-Wire
    }
    return n;
}

void N2kManager::sendLegacyTemperatures(const EngineState &state) {
    TempChannel channels[MAX_TEMP_CHANNELS];
    uint8_t n = collectTemperatures(state, channels);
    for (uint8_t i = 0; i < n; i++) {
        tN2kMsg msg;
        SetN2kTemperature(msg, tempSid, channels[i].instance,
                          (tN2kTempSource)channels[i].source, channels[i].kelvin);
        n2k.SendMsg(msg);
    }
}

void N2kManager::sendExtendedTemperatures(const EngineState &state) {
    TempChannel channels[MAX_TEMP_CHANNELS];
    uint8_t n = collectTemperatures(state, channels);
    for (uint8_t i = 0; i < n; i++) {
        tN2kMsg msg;
        SetN2kTemperatureExt(msg, tempSid, channels[i].instance,
                             (tN2kTempSource)channels[i].source, channels[i].kelvin);
        n2k.SendMsg(msg);
    }
    tempSid++;
}

void N2kManager::update(EngineState &state) {
    s_state = &state;
    n2k.ParseMessages();

    unsigned long now = millis();

    // Engine PGNs are sent while running, and also while shutting down (RPM run-down
    // or stop relay energised) so the Shutting Down flag and falling RPM reach displays.
    bool engineActive = state.isEngineRunning || state.isEngineStopping || state.stopRelayActive;

    // 1. Rapid Engine Updates (PGN 127488): Every 500ms when running / shutting down
    if (engineActive && (now - lastRapidTime >= PERIOD_RAPID_ENGINE_MS)) {
        lastRapidTime = now;
        sendRapidEngineData(state);
    }

    // 2. Dynamic Engine Updates (PGN 127489): Every 1000ms when running / shutting down
    if (engineActive && (now - lastDynamicTime >= PERIOD_ENGINE_DYNAMIC_MS)) {
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
