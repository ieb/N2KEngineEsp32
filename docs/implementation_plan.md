# Implementation Plan: NMEA2000 Engine Interface on ESP32-C3

Implementation of the marine diesel engine monitoring system based on the ESP32-C3 microcontroller, 3x ADS1115 16-bit ADCs, flywheel pulse RPM counter, relay coil status detectors, 1-Wire temperature sensors, and NMEA 2000 CAN bus interface adhering to [docs/spec.md](spec.md) and reference curves in `/Users/ieb/timefields/PlatformIO/Projects/N2KEngine/lib/enginesensors/enginesensors.cpp`.

## User Review & Decisions Incorporated

1. **Sensor Curves & Conversion Models**:
   - **NTC Thermistors**: Analytical Beta parameter model ($\frac{1}{T} = \frac{1}{T_0} + \frac{1}{\beta} \ln\left(\frac{R}{R_0}\right)$ with $R_0 = 10000\,\Omega$, $T_0 = 298.15\text{ K}$, $\beta = 3950\text{ K}$) eliminating lookup table quantization kinks.
   - **Oil Pressure**: 0–5V sensor (0.5V = 0 psi, 4.5V = 100 psi, linear at 172,369 Pa/V).
   - **Coolant Temperature**: Volvo Penta Standard Coolant Sensor with `coolantTable` lookup (10°C to 120°C). 12V 1k pullup.
   - **Fuel Level**: European 0–190 $\Omega$ resistive sender (0 $\Omega$ = Empty, 190 $\Omega$ = Full), 1k pullup.
2. **Engine Block Negative Offset Compensation**:
   - Engine Block is -ve relative to board GND due to alternator high charging current.
   - Circuit: `5V - 10K - ADC (ADC2 AIN2) - 10K - Engine Block -`.
   - Engine block voltage: $V_{block} = 2 \cdot V_{adc2\_ain2} - 5.0\text{ V}$.
   - Coolant sensor and supply voltage are referenced to Engine Block: $V_{coolant} = V_{coolant\_gnd} - V_{block}$ and $V_{supply} = V_{battery\_gnd} - V_{block}$.
3. **Exhaust Temperature Mapping**:
   - NTC1 (Exhaust Water Trap) is reported as **Exhaust Gas** on PGN 127493 (Transmission Oil Temp remapped) and PGN 130312 / 130316 Source 14.
   - NTC5 (Exhaust Elbow) is monitored for the water flow collapse rate-of-rise and absolute temperature alarms using lines 720–800 logic of `enginesensors.cpp`.
4. **Storage Architecture**:
   - Partition table configured for **2MB Application / 2MB SPIFFS** (`no_ota.csv`).
   - Historical alarm events stored in SPIFFS (`/events.csv`) for retrieval over BLE / serial.
   - Dropped PGN 130817 (Dump Events fast packet).
   - Listen for NMEA 2000 GPS System Time (PGN 126992 / PGN 129033) to timestamp logged events with real UTC time.

---

## Proposed Changes

### Configuration & Build Environment

#### [MODIFY] [platformio.ini](../platformio.ini)
- Add required libraries:
  - `RobTillaart/ADS1x15`
  - `paulstoffregen/OneWire`
  - `DallasTemperature`
- Ensure build flags and partition table match ESP32-C3 4MB flash layout.

#### [NEW] [no_ota.csv](../src/no_ota.csv)
- Partition table: 2MB code (`app0`), 2MB `spiffs`, plus `nvs` partition.

#### [NEW] [config.h](../src/config.h)
- Centralized hardware definitions:
  - GPIO pin numbers (CAN TX=3, RX=1, Flywheel=4, Stop=5, Glow=6, Start=7, SDA=8, SCL=9, 1-Wire=0)
  - ADS1115 I2C addresses (0x48, 0x49, 0x4B)
  - Calibration constants, alarm thresholds, N2K device parameters.

---

### Hardware Abstraction Layer (HAL)

#### [NEW] [AdcManager.h](../src/hal/AdcManager.h) & [AdcManager.cpp](../src/hal/AdcManager.cpp)
- Manages 3x ADS1115 ADCs on I2C bus (GPIO8/GPIO9) at 64 SPS with PGA `GAIN_TWOTHIRDS` (+/- 6.144V).
- Samples all 12 channels asynchronously/cooperatively with rolling averaging.

#### [NEW] [RpmSensor.h](../src/hal/RpmSensor.h) & [RpmSensor.cpp](../src/hal/RpmSensor.cpp)
- Captures 30 pulses/rev on GPIO4 using hardware interrupt / timer.
- Computes engine speed: $\text{RPM} = \text{frequency (Hz)} \times 2.0$.
- Tracks engine running ($\text{RPM} \ge 800$) and stopping ($\text{RPM} < 500$) states.

#### [NEW] [DigitalInputs.h](../src/hal/DigitalInputs.h) & [DigitalInputs.cpp](../src/hal/DigitalInputs.cpp)
- Reads Stop Relay Coil (GPIO5), Start Relay Coil (GPIO7), and Glow Relay Coil (GPIO6).

#### [NEW] [OneWireTemp.h](../src/hal/OneWireTemp.h) & [OneWireTemp.cpp](../src/hal/OneWireTemp.cpp)
- Scans and reads Dallas DS18B20 sensors on GPIO0.

---

### Sensors & Signal Processing

#### [NEW] [SensorProcessing.h](../src/sensors/SensorProcessing.h) & [SensorProcessing.cpp](../src/sensors/SensorProcessing.cpp)
- Converts raw voltages using `enginesensors.cpp` algorithms:
  - Analytical Beta parameter model for NTC1–NTC5.
  - Oil pressure linear conversion: $(\text{Volts} - 0.5) \times 172,369\text{ Pa}$.
  - Engine block negative offset calculation: $V_{block} = 2 \cdot V_{adc} - 5.0\text{ V}$.
  - Coolant sensor lookup with $V_{block}$ offset subtraction and battery voltage ratiometric scaling.
  - Fuel level conversion (European 0–190 $\Omega$).
  - Voltage divider scaling ($14.7 / 4.7$).
  - Raw water flow collapse detection on NTC5 (Exhaust Elbow) using lines 720–800 rate-of-rise anchor and absolute temperature thresholds.

#### [NEW] [EngineState.h](../src/sensors/EngineState.h)
- Struct containing all sensor readings, valid flags, discrete alarm status bitmasks, and engine runtime.

---

### Alarms & Persistence

#### [NEW] [AlarmManager.h](../src/alarms/AlarmManager.h) & [AlarmManager.cpp](../src/alarms/AlarmManager.cpp)
- Evaluates Discrete Status 1 & 2 bitmasks with grace period and persistence windows.
- Emits alarm event transitions for persistent logging.

#### [NEW] [StorageManager.h](../src/storage/StorageManager.h) & [StorageManager.cpp](../src/storage/StorageManager.cpp)
- Mounts SPIFFS filesystem.
- Stores engine hours accumulator in NVS Preferences (accumulated when $\text{RPM} \ge 800$).
- Appends alarm events to `/events.csv` on SPIFFS with timestamp (UTC time if GPS received, otherwise engine hours/uptime).

---

### NMEA 2000 Stack

#### [NEW] [N2kManager.h](../src/n2k/N2kManager.h) & [N2kManager.cpp](../src/n2k/N2kManager.cpp)
- Configures `tNMEA2000_esp32` TWAI driver with GPIO3 (TX) and GPIO1 (RX).
- Configures ISO Address Claim (address 24, Device Class 50, Device Function 140, Manufacturer Code 2048).
- Registers Product Information and Configuration Information.
- Listens for GPS System Time (PGN 126992).
- Periodic transmitter for PGN 127488, 127489, 127493, 127508, 127505, 130312, 130316.

---

### Application Entry Point & CLI

#### [NEW] [main.cpp](../src/main.cpp)
- Setup and cooperative non-blocking scheduling.
- Interactive Serial CLI (`h` help, `s` status, `E` set hours, `F` fake RPM, `d`/`D`/`M` debug toggles).

---

## Verification Plan

### Automated / Build Verification
- Compile with PlatformIO:
  ```bash
  pio run
  ```

### Functional Validation
- Unit / test verification for sensor calibrations, block offset compensation, alarm timers, and message packing.
