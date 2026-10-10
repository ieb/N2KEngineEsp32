#pragma once
#include <Arduino.h>
#include <N2kMsg.h>
#include "config.h"

struct EngineState {
    // Engine Speed & State
    double engineRpm = 0.0;
    bool isEngineRunning = false;
    bool isEngineStopping = false;

    // Digital Relays
    bool stopRelayActive  = false;
    bool startRelayActive = false;
    bool glowRelayActive  = false;

    // Electrical Voltages (V)
    double alternatorVoltage   = N2kDoubleNA;
    double engineBatteryVoltage = N2kDoubleNA;
    double spareVoltage        = N2kDoubleNA;
    double engineBlockOffsetV  = 0.0; // Voltage of engine block relative to board GND

    // Temperatures in Kelvin (N2kDoubleNA if invalid / disconnected)
    double exhaustWaterTrapK   = N2kDoubleNA; // NTC1 (Reported as Exhaust Gas)
    double alternatorTempK     = N2kDoubleNA; // NTC2
    double engineRoomTempK     = N2kDoubleNA; // NTC3
    double gearboxTempK        = N2kDoubleNA; // NTC4
    double exhaustElbowTempK   = N2kDoubleNA; // NTC5 (Monitored for rapid rise / water flow)
    double coolantTempK        = N2kDoubleNA; // Coolant sensor adjusted for engine block drop

    // Temperatures in deci-Celsius (0.1 C) for alarms
    int16_t exhaustWaterTrap_dC = -9999;
    int16_t alternatorTemp_dC   = -9999;
    int16_t engineRoomTemp_dC   = -9999;
    int16_t gearboxTemp_dC      = -9999;
    int16_t exhaustElbowTemp_dC = -9999;
    int16_t coolantTemp_dC      = -9999;

    // 1-Wire Auxiliary Temperatures (Kelvin)
    uint8_t oneWireCount = 0;
    double oneWireTempK[4] = {N2kDoubleNA, N2kDoubleNA, N2kDoubleNA, N2kDoubleNA};

    // Pressures
    double oilPressurePa = N2kDoubleNA;

    // Tank Levels
    double fuelLevelPercent = N2kDoubleNA; // 0.0 - 100.0%
    double fuelCapacityLitres = FUEL_CAPACITY_LITRES;

    // Alarms / Status Bitfields
    uint16_t discreteStatus1 = 0;
    uint16_t discreteStatus2 = 0;

    // Engine Hours Accumulated (Seconds)
    uint32_t engineHoursSeconds = 0;

    // GPS / System Time (UTC Unix epoch seconds, 0 if not received)
    uint32_t systemTimeUtc = 0;
};
