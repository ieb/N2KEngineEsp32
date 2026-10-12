#pragma once
#include <Arduino.h>

// ============================================================================
// GPIO Pin Definitions (ESP32-C3)
// ============================================================================
#define PIN_CAN_RX          1
#define PIN_CAN_TX          3
#define PIN_FLYWHEEL        4
#define PIN_STOP_COIL       5
#define PIN_GLOW_COIL       6
#define PIN_START_COIL      7
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9
#define PIN_ONE_WIRE        0

// ============================================================================
// I2C & ADS1115 Configuration
// ============================================================================
#define I2C_FREQUENCY       400000UL

// ADS1115 I2C Addresses (7-bit)
#define ADS1115_ADDR_1      0x48   // Addr pin -> GND: NTC1, NTC2, NTC3, Alternator Voltage
#define ADS1115_ADDR_2      0x49   // Addr pin -> VDD: Oil Press, Coolant, Engine Block, Engine Battery
#define ADS1115_ADDR_3      0x4B   // Addr pin -> SCL: NTC4, NTC5, Fuel, Spare Voltage

// ADS1115 SPS (64 SPS gives DC Nyquist 32Hz, matches sigma-delta filtering)
#define ADS1115_SPS         64

// ADC Voltage full-scale reference for ADS1115 (running at 5.0V with level shifter)
// Using PGA gain = GAIN_TWOTHIRDS (+/- 6.144V) so 0-5V analog inputs are within range
#define ADS_VOLTS_PER_BIT   (0.1875 / 1000.0) // 0.1875 mV per bit

// ============================================================================
// Flywheel / Engine Speed
// ============================================================================
#define FLYWHEEL_TEETH              30
#define RPM_FACTOR                  (60.0 / FLYWHEEL_TEETH) // 2.0
#define MIN_ENGINE_RUNNING_RPM      800.0
#define ENGINE_SHUTDOWN_RPM         500.0
#define FLYWHEEL_READ_PERIOD_MS     100
#define ENGINE_START_GRACE_PERIOD   15000UL // 15s grace before alarming

// RPM sensor implementation: 0 = window counting (RpmSensor),
// 1 = edge timing averaged over whole revolutions (EdgeRpmSensor)
#ifndef RPM_SENSOR_EDGE_TIMING
#define RPM_SENSOR_EDGE_TIMING      1
#endif
// Edge timing: reject edges closer than this (glitches; 150us == 13,333 RPM)
#define RPM_EDGE_MIN_PERIOD_US      150UL
// Edge timing: no edge for this long means the flywheel has stopped
#define RPM_EDGE_ZERO_TIMEOUT_US    500000UL
// Edge timing: if a full revolution has not completed within this time (very low
// RPM, start-up, run-down), compute from the edges seen so far instead of waiting.
#define RPM_EDGE_PARTIAL_TIMEOUT_US 250000UL

// Bench test signal (LEDC square wave). Wire GPIO2 -> 1K -> GPIO4 and short the
// flywheel input (J15) so the LM393 output is released. GPIO2 is a strapping pin;
// the 10K pull-up on GPIO4 keeps it high at boot as required.
#define PIN_BENCH_SIGNAL            2
#define BENCH_LEDC_CHANNEL          0

// ============================================================================
// Resistor Dividers & Calibrations
// ============================================================================
// Voltage Dividers (12V side: 10K / 4K7)
// V_adc = V_in * 4.7 / (10 + 4.7) => V_in = V_adc * (14.7 / 4.7) = 3.12765957
#define VOLTAGE_SCALE               (14.7 / 4.7)
// Alternator V_adc = V_in * 3.3 / (10 + 3.3) => V_in = V_adc * (13.3 / 3.3) = 4.0303030303, max arround 18V
#define VOLTAGE_SCALE_ALTERNATOR    (13.3 / 3.3)

// NTC Thermistors (10K @ 25C, 4K7 pullup to 5V)
#define NTC_R_TOP_OHM               4700.0f
#define NTC_R0_OHM                  10000.0f
#define NTC_T0_KELVIN               298.15f
#define NTC_BETA_KELVIN             3950.0f

// Oil Pressure (0-5V sensor)
// 0.5V == 0 psi, 4.5V == 100 psi, 100 / 4 = 25 psi/V, 1 psi = 6894.76 Pa
// 25 * 6894.76 = 172369 Pa/V
#define OIL_PRESS_V0                0.5
#define OIL_PRESS_V100              4.5
#define OIL_PRESS_DISCONNECTED_V    0.2
#define OIL_PRESS_SCALE_PA          172369.0
#define MIN_OIL_PRESSURE_PA         68940.0 // 10.0 PSI

// Fuel Sender (European 0-190 ohm, 1K pullup to 5V)
// Full (190 ohm) = 5V * 190 / (190 + 1000) = 0.7983V
// Empty (0 ohm) = 0V
#define FUEL_RESISTOR_TOP           1000.0
#define FUEL_SENSOR_MAX_OHM         190.0
#define FUEL_CAPACITY_LITRES        60.0

// Alarm Thresholds & Windows
// Coolant sender (1K pull-up to engine battery, BAT46 clamp to 5V on the sense node)
// Sense node at/above the clamp means sender open-circuit or engine colder than measurable.
#define COOLANT_CLAMP_V             4.9f
// Normalised reading below this (~130C+) means a shorted sender / wire to block.
#define COOLANT_SHORT_COUNTS        100
// Plausible engine block offset window (V, relative to board 0V). Outside it the
// block sense wire is treated as disconnected and no compensation is applied.
#define ENGINE_BLOCK_OFFSET_MIN_V   (-2.0f)
#define ENGINE_BLOCK_OFFSET_MAX_V   (0.5f)

#define MAX_COOLANT_TEMP_DECI_C     970    // 97.0 C
#define COOLANT_ALARM_WINDOW_MS     15000UL // 15s sustained

#define LOW_BATTERY_VOLTAGE         11.8   // V
// Engine battery is measured on the ignition-switched feed (J14.4). Below this the
// feed is considered off and the voltage is reported as N/A rather than ~0V.
#define ENGINE_BATTERY_OFF_V        3.0    // V
#define LOW_ALTERNATOR_VOLTAGE      12.2   // V
#define VOLTAGE_ALARM_WINDOW_MS     5000UL // 5s sustained
#define OIL_PRESS_ALARM_WINDOW_MS   5000UL // 5s sustained

// Exhaust Alarms (in 0.1 C)
#define MAX_EXHAUST_TEMP_DECI_C     450    // 45.0 C absolute trip
#define CLEAR_EXHAUST_TEMP_DECI_C   380    // 38.0 C clear threshold
#define EXHAUST_BASELINE_TEMP_DECI_C 350   // 35.0 C warmup threshold for rate-of-rise
#define EXHAUST_RISE_DELTA_DECI_C   80     // +8.0 C rise
#define EXHAUST_RISE_WINDOW_MS      30000UL // within 30s
#define HIGH_EXHAUST_WINDOW_MS      5000UL // 5s sustained above 45C for EMERGENCY_STOP

// Alternator & Engine Room Alarms (in 0.1 C)
#define MAX_ALTERNATOR_TEMP_DECI_C  1100   // 110.0 C
#define CLEAR_ALTERNATOR_TEMP_DECI_C 500   // 50.0 C
#define MAX_ENGINEROOM_TEMP_DECI_C  700    // 70.0 C
#define CLEAR_ENGINEROOM_TEMP_DECI_C 400   // 40.0 C

// ============================================================================
// NMEA 2000 Configuration
// ============================================================================
#define N2K_DEFAULT_ADDRESS         24
#define N2K_DEVICE_ID               223    // Fallback only; unique number is derived from the ESP32 MAC
#define N2K_DEVICE_FUNCTION         140    // Engine
#define N2K_DEVICE_CLASS            50     // Propulsion
#define N2K_MANUFACTURER_CODE       2046   // 11-bit field (max 2047); must match PROPRIETARY_CODE
#define N2K_INDUSTRY_GROUP          4      // Marine

// PGN Periods (in milliseconds)
#define PERIOD_RAPID_ENGINE_MS      500    // PGN 127488 (when running)
#define PERIOD_ENGINE_DYNAMIC_MS    1000   // PGN 127489 (when running)
#define PERIOD_BATTERY_STATUS_MS    1000   // PGN 127508 (continuous)
#define PERIOD_FLUID_LEVEL_MS       2500   // PGN 127505 (continuous)
#define PERIOD_EXT_TEMPERATURE_MS   2500   // PGN 130316 (continuous)
#define PERIOD_TRANSMISSION_MS      4850   // PGN 127493 (continuous)
#define PERIOD_LEGACY_TEMP_MS       4850   // PGN 130312 (continuous)

// Instances
#define ENGINE_INSTANCE             0
#define ENGINE_BATTERY_INSTANCE     0
#define ALTERNATOR_BATTERY_INSTANCE 2
#define FUEL_TANK_INSTANCE          0

// Temperature sources / instances (PGN 130312 / 130316)
#define TEMP_INSTANCE_EXHAUST_TRAP  0      // NTC1, Source 14 (Exhaust Gas)
#define TEMP_INSTANCE_EXHAUST_ELBOW 1      // NTC5, Source 14 (Exhaust Gas)
#define TEMP_SOURCE_ALTERNATOR      30     // NTC2
#define TEMP_SOURCE_ONEWIRE_BASE    31     // 1-Wire sensor i -> 31 + i (i = 0..3)
#define TEMP_SOURCE_GEARBOX         35     // NTC4

// Proprietary PGNs
#define PROPRIETARY_CODE            0x9FFE // Manufacturer 2046 + Marine bit
#define PGN_PROPRIETARY_CMD         65305L
#define PGN_PROPRIETARY_FAST        130817L
#define FN_DUMP_EVENTS              11
#define FN_DUMP_EVENTS_RESP         12
#define MAX_DUMP_EVENTS             53     // (223 - 8 header bytes) / 4 bytes per record
#define FN_CLEAR_EVENTS             13
#define FN_CLEAR_EVENTS_RESP        14

// Event IDs
#define EVENT_NONE                  0
#define EVENT_ENGINE_STOP           1
#define EVENT_LOW_OIL_PRES          2
#define EVENT_HIGH_COOLANT          3
#define EVENT_EXHAUST_TEMP          4
#define EVENT_ALTERNATOR_TEMP       5
#define EVENT_ENGINE_ROOM_TEMP      6

// Discrete Status 1 Flags (PGN 127489 Bytes 20-21)
#define STATUS1_CHECK_ENGINE        0x0001
#define STATUS1_OVERTEMP            0x0002
#define STATUS1_LOW_OIL_PRES        0x0004
#define STATUS1_LOW_OIL_LEVEL       0x0008
#define STATUS1_LOW_FUEL_PRESS      0x0010
#define STATUS1_LOW_SYSTEM_VOLTAGE  0x0020
#define STATUS1_LOW_COOLANT_LEVEL   0x0040
#define STATUS1_WATER_FLOW          0x0080
#define STATUS1_WATER_IN_FUEL       0x0100
#define STATUS1_CHARGE_INDICATOR    0x0200
#define STATUS1_PREHEAT_INDICATOR   0x0400
#define STATUS1_HIGH_BOOST_PRESSURE 0x0800
#define STATUS1_REV_LIMIT_EXCEEDED  0x1000
#define STATUS1_EGR_SYSTEM          0x2000
#define STATUS1_THROTTLE_POS_SENSOR 0x4000
#define STATUS1_EMERGENCY_STOP      0x8000

// Discrete Status 2 Flags (PGN 127489 Bytes 22-23)
#define STATUS2_WARN_1              0x0001
#define STATUS2_WARN_2              0x0002
#define STATUS2_POWER_REDUCTION     0x0004
#define STATUS2_MAINTENANCE_NEEDED  0x0008
#define STATUS2_COMM_ERROR          0x0010
#define STATUS2_SUB_THROTTLE        0x0020
#define STATUS2_NEUTRAL_START       0x0040
#define STATUS2_ENGINE_SHUTTING_DOWN 0x0080
