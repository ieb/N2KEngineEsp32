# A NMEA2000 Engine inerface with BLE

This is a NMEA2000 interface to an marine diesel engine built arround a ESP32-C3 and ADS1115 16 bit ADCs.

NB: All paths in this file are relative to the root of the git repo.

The curcuit is defined in pcb/esp32board/esp32board.kicad_sch


# Hardware

## Peripherals

* CAN bus connection is using the ESP32 CAN interface, driven by the NMEA2000 library from ttlappalainen and a TWAI driver for the same. CAN-TX GPIO3, CAN-RX GPIO1
* 3x ADS1115 I2C 16bit ADCs  running at 5V via a level shifter.
	* ADC1 Adress 1001000b (Addr pin GND) 
		* AIN0 - NTC1 10K NTC  5V 4K7 Pullup
		* AIN1 - NTC2 10K NTC  5V 4K7 Pullup
		* AIN2 - NTC3 10K NTC  5V 4K7 Pullup
		* AIN3 - Alternator Voltage Divider 10K/4K7
	* ADC2 Address 1001001b (Addr pin Vdd)
		* AIN0 - Oil Pressure 0-5V Sensor
		* AIN1 - Coolant 12V 1K Pullup
		* AIN2 - Engine Block, see below 
		* AIN3 - Engine Battery, Voltage Divider 10K/4K7 Voltage Divider 10K/4K7
	* ADC3 Address 1001011b (Addr pin SCL)
		* AIN0 - NTC4 10K NTC  5V 4K7 Pullup
		* AIN1 - NTC5 10K NTC  5V 4K7 Pullup
		* AIN2 - Fuel 10K NTC  5V 1K Pullup
		* AIN3 - V sense, spare Voltage Divider 10K/4K7

* Flywheel Pulse on GPIO4, 30 pulses per revolution
* Stop Relay Coil 10K/2K2+LED Divider
* Start Relay Coil 10K/2K2+LED Divider
* Glow Relay Coil 10K/2K2+LED Divider
* 1 Wire GPIO10

## GPIO Connections

* GPIO1 CAN-RX
* GPIO3 CAN-TX
* GPIO4 Flywheel Pulse
* GPIO5 StopCoil
* GPIO6 StartCoil
* GPIO7 GlowCoil
* GPIO8 I2C SDA 
* GPIO9 I2C SCL
* GPI10 1 Wire

## ADC Voltage Dividers

The ADS1115 will be running at 64sps, giving a Nyquist frequency of 32Hz due to Sigma Delta, so no RC used in Dividers. 

12V

    12V  ----
            |
           10K
            |---- 2K2 --- ADC Pin
            4K7
            |
     0V -----

NTC

    5V ------
            |
           4K7
            | --- 2K2 --- ADC Pin
            NTC
            |
    0V ------

Coils

    12V Coil ----
                |
                10K
                | --- 2K2 --- ADC Pin
                2k2
                |
                LED
                |
     0V ---------

Engine Block - -ve relative to 0V

     5V ----------
                 |
                10K 
                 | --- 2K2 --- ADC Pin
                10K
                 |
    Engine Block -         


## NTC Mappings

* NTC1 == Exhaust Water trap
* NTC2 == Alternator
* NTC3 == Engine Room
* NTC4 == Gearbox
* NTC5 == Exhaust Elbow

# Firmware 

Built using platform IO, platform.ini and the structure is already setup.

## Libraries

* NMEA2000 https://github.com/ttlappalainen/NMEA2000.git
* TWAI NMEA2000 driver https://github.com/ieb/NMEA2000_twai.git#platformIOUse
* ADS1115 https://github.com/RobTillaart/ADS1x15

## BLE Interface

There will be a BLE GATT interface, but not just yet.

# NMEA2000 PGN Specification

The device acts as a marine propulsion engine monitor (Device Class 50 `Propulsion`, Device Function 140 `Engine`, Industry Group 4 `Marine`). Default CAN address is 24 with ISO address claiming supported.

To allow standard marine multifunction displays (MFDs) and dedicated instruments (such as Raymarine i70) to present all engine and electrical metrics, several standard PGN fields and instances are deliberately remapped to convey key temperatures and alternator data without requiring proprietary display widgets.

## Summary of Remapped / Custom Instrument Fields

| Physical Measurement | N2K PGN | Field Name in PGN | Target Display / Rationale |
| :--- | :--- | :--- | :--- |
| **Alternator Temperature** (NTC2) | **127489** (Engine Dynamic) | `Engine Oil Temperature` | Standard engine gauge displays alternator temperature (critical for LiFePO4 charging). |
| **Exhaust Temperature** (NTC1 / NTC5) | **127493** (Transmission Dynamic) | `Transmission Oil Temperature` | Displayed on Raymarine i70 gearbox/transmission temperature gauge (which lacks exhaust temp widget). |
| **Alternator Voltage & Temperature** (ADC1 AIN3 & NTC2) | **127508** (Battery Status) | `Battery Instance 2` (Voltage & Temperature) | Alternator monitored as a dedicated battery source on battery status pages (Instance 0 = Engine Battery, Instance 1 reserved for Service Shunt). |
| **Exhaust Temperature** (NTC1 / NTC5) | **130312** / **130316** (Temperature) | `Source 14` (`Exhaust Gas`) | Standard N2K exhaust temperature for compatible MFDs. |
| **Engine Room Temperature** (NTC3) | **130312** / **130316** (Temperature) | `Source 3` (`Engine Room`) | Standard engine compartment temperature. |
| **Alternator & 1-Wire Sensors** | **130312** / **130316** (Temperature) | `Source 30`, `31+i` (User/Custom Sources) | Custom temperature sources for auxiliary logging / MFD data pages. |

---

## PGN Message Directory

| PGN | Name | Frame Type | Priority | Rate | Condition |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **127488** | Engine Parameters, Rapid Update | Single frame (8 B) | 2 | 500 ms (0.1s ideal) | Engine running (RPM >= 800) |
| **127489** | Engine Parameters, Dynamic | Fast packet (26 B) | 2 | 1000 ms (0.5s ideal) | Engine running (RPM >= 800) |
| **127493** | Transmission Parameters, Dynamic | Single frame (8 B) | 2 | ~4.85 s | Continuous |
| **127508** | Battery Status | Single frame (8 B) | 6 | ~1.0 s | Continuous (Inst 0 & Inst 2) |
| **127505** | Fluid Level (Fuel) | Single frame (8 B) | 6 | 2.5 s - 4.9 s | Continuous |
| **130312** | Temperature (Legacy) | Single frame (8 B) | 5 | ~4.85 s | Continuous (multiple sources) |
| **130316** | Extended Temperature | Single frame (8 B) | 5 | 2.5 s | Continuous (high resolution) |
| **65305** | Proprietary Single Frame | Single frame (8 B) | 6 | On request | Diagnostics / event management |
| **130817** | Proprietary Fast Packet | Fast packet (var) | 6 | On request | Event history dump response |
| **59392** | ISO Acknowledgment | Single frame (8 B) | 6 | On request | ISO network management |
| **59904** | ISO Request | Single frame (3 B) | 6 | On receive | ISO network management |
| **60928** | ISO Address Claim | Single frame (8 B) | 6 | At boot / claim | ISO network management |
| **126464** | PGN List (TX / RX) | Fast packet (var) | 6 | On request | ISO network management |
| **126996** | Product Information | Fast packet (134 B) | 6 | On request | ISO network management |
| **126998** | Configuration Information | Fast packet (var) | 6 | On request | ISO network management |

---

## Detailed Message Structures

Note that NMEA2000 message handling will be performed by the https://github.com/ttlappalainen/NMEA2000.git Library. The message structures are here for reference noting where the data comes from and any abuse of the standard fields. 

### 1. PGN 127488 – Engine Parameters, Rapid Update
* **Transmission**: Single frame (8 bytes), broadcast (0xFF), Priority 2.
* **Period**: 500 ms (nominally 100 ms). Sent only when engine is running (`RPM >= 800`).

| Byte | Field Name | Data Type | Units / Resolution | Value / Source |
| :---: | :--- | :--- | :--- | :--- |
| 0 | Engine Instance | `uint8` | 1 | `0` (`ENGINE_INSTANCE`) |
| 1–2 | Engine Speed | `uint16` | 0.25 RPM / bit | From flywheel pulses (GPIO4, 30 teeth, `RPM = Hz * 2.0`). Range: 0–16,383.75 RPM. `0xFFFF` = N/A. |
| 3–4 | Engine Boost Pressure | `uint16` | 100 Pa / bit | `0xFFFF` (N/A) |
| 5 | Engine Tilt / Trim | `int8` | 1 % / bit | `0x7F` (N/A) |
| 6–7 | Reserved | `uint16` | — | `0xFFFF` |

---

### 2. PGN 127489 – Engine Parameters, Dynamic
* **Transmission**: Fast packet (26 data bytes), broadcast (0xFF), Priority 2.
* **Period**: 1000 ms (nominally 500 ms). Sent only when engine is running (`RPM >= 800`).

| Byte | Field Name | Data Type | Units / Resolution | Value / Source |
| :---: | :--- | :--- | :--- | :--- |
| 0 | Engine Instance | `uint8` | 1 | `0` (`ENGINE_INSTANCE`) |
| 1–2 | Engine Oil Pressure | `uint16` | 100 Pa / bit (hPa) | Measured from 0–5V oil pressure sensor (ADC2 AIN0). `0xFFFF` = N/A. |
| 3–4 | **Engine Oil Temperature [REMAPPED]** | `uint16` | 0.1 K / bit | **Alternator Temperature** from NTC2 (ADC1 AIN1). Remapped so MFDs display alternator thermal load. `0xFFFF` = N/A. |
| 5–6 | Engine Coolant Temperature | `uint16` | 0.01 K / bit | Coolant thermistor sensor (ADC2 AIN1). `0xFFFF` = N/A. |
| 7–8 | Alternator Potential (Voltage) | `int16` | 0.01 V / bit | Alternator voltage divider (ADC1 AIN3). `0x7FFF` = N/A. |
| 9–10 | Fuel Rate | `int16` | 0.1 L/h / bit | `0x7FFF` (N/A) |
| 11–14 | Total Engine Hours | `uint32` | 1 s / bit | Total accumulated runtime seconds when `RPM >= 800`. Stored in non-volatile memory. `0xFFFFFFFF` = N/A. |
| 15–16 | Engine Coolant Pressure | `uint16` | 100 Pa / bit | `0xFFFF` (N/A) |
| 17–18 | Engine Fuel Pressure | `uint16` | 1000 Pa / bit | `0xFFFF` (N/A) |
| 19 | Reserved | `uint8` | — | `0xFF` |
| 20–21 | Discrete Status 1 | `uint16` | Bitfield | Engine alarm bitmask (see table below). |
| 22–23 | Discrete Status 2 | `uint16` | Bitfield | Warning & shutdown bitmask (see table below). |
| 24 | Engine Percent Load | `int8` | 1 % / bit | `0x7F` (N/A) |
| 25 | Engine Percent Torque | `int8` | 1 % / bit | `0x7F` (N/A) |

#### Discrete Status 1 Bitfield (Bytes 20–21)
| Bit | Hex Mask | Flag Name | Trigger Condition / Description |
| :---: | :---: | :--- | :--- |
| 0 | `0x0001` | Check Engine | General engine trouble warning. |
| 1 | `0x0002` | Over Temperature | Coolant temp > 97.0 °C sustained for > 15 seconds. |
| 2 | `0x0004` | Low Oil Pressure | Oil pressure < 10.0 PSI (68.9 kPa) sustained for > 5 seconds while running. |
| 3 | `0x0008` | Low Oil Level | Unused (`0`). |
| 4 | `0x0010` | Low Fuel Pressure | Unused (`0`). |
| 5 | `0x0020` | Low System Voltage | Engine battery voltage < 11.8 V sustained for > 5 seconds. |
| 6 | `0x0040` | Low Coolant Level | Unused (`0`). |
| 7 | `0x0080` | Water Flow | Raw water flow collapse: Exhaust elbow rate-of-rise > 8.0 °C in 30s or absolute temp > 45.0 °C. |
| 8 | `0x0100` | Water in Fuel | Unused (`0`). |
| 9 | `0x0200` | Charge Indicator | Alternator voltage < 12.2 V sustained for > 5 seconds while running. |
| 10 | `0x0400` | Preheat Indicator | Glow plug relay active (GPIO7 GlowCoil input). |
| 11 | `0x0800` | High Boost Pressure | Unused (`0`). |
| 12 | `0x1000` | Rev Limit Exceeded | Unused (`0`). |
| 13 | `0x2000` | EGR System | Unused (`0`). |
| 14 | `0x4000` | Throttle Position Sensor | Unused (`0`). |
| 15 | `0x8000` | Emergency Stop | Severe exhaust over-temperature (> 45.0 °C) sustained for > 5 seconds. |

#### Discrete Status 2 Bitfield (Bytes 22–23)
| Bit | Hex Mask | Flag Name | Trigger Condition / Description |
| :---: | :---: | :--- | :--- |
| 0 | `0x0001` | Warning Level 1 | Alternator temp > 110.0 °C or Engine room temp > 70.0 °C. |
| 1 | `0x0002` | Warning Level 2 | Unused (`0`). |
| 2 | `0x0004` | Power Reduction | Unused (`0`). |
| 3 | `0x0008` | Maintenance Needed | Unused (`0`). |
| 4 | `0x0010` | Engine Comm Error | Unused (`0`). |
| 5 | `0x0020` | Sub / Secondary Throttle | Unused (`0`). |
| 6 | `0x0040` | Neutral Start Protect | Unused (`0`). |
| 7 | `0x0080` | Engine Shutting Down | RPM < 500 RPM while engine is stopping / Stop relay active (GPIO5). |

---

### 3. PGN 127493 – Transmission Parameters, Dynamic [REMAPPED]
* **Transmission**: Single frame (8 bytes), broadcast (0xFF), Priority 2.
* **Period**: ~4.85 s. Sent continuously.
* **Purpose**: Remapped to carry **Exhaust Temperature** so Raymarine i70 and other MFDs can show exhaust temperature on standard transmission gauge displays.

| Byte | Field Name | Data Type | Units / Resolution | Value / Source |
| :---: | :--- | :--- | :--- | :--- |
| 0 | Engine Instance | `uint8` | 1 | `0` (`ENGINE_INSTANCE`) |
| 1 | Transmission Gear | `uint8` | Enum | `0x03` (Invalid / N/A; 0=Fwd, 1=Neutral, 2=Rev) |
| 2–3 | Transmission Oil Pressure | `uint16` | 100 Pa / bit | `0xFFFF` (N/A) |
| 4–5 | **Transmission Oil Temperature [REMAPPED]** | `uint16` | 0.1 K / bit | **Exhaust Temperature** from NTC1 / NTC5. `0xFFFF` = N/A. |
| 6 | Transmission Status | `uint8` | Bitfield | `0x00` |
| 7 | Reserved / Padding | `uint8` | — | `0x00` |

---

### 4. PGN 127508 – Battery Status (DC Detailed Status)
* **Transmission**: Single frame (8 bytes), broadcast (0xFF), Priority 6.
* **Period**: ~1.0 s (`VOLTAGE_UPDATE_PERIOD`). Sent continuously.
* **Instances**: Two separate messages sent per cycle:
  1. **Instance 0 (`ENGINE_BATTERY_INSTANCE`)**: Engine starting battery.
  2. **Instance 2 (`ALTERNATOR_BATTERY_INSTANCE`) [REMAPPED]**: Alternator electrical output and operating temperature. (Instance 1 is kept open for an external service battery shunt).

| Byte | Field Name | Data Type | Units / Resolution | Instance 0 (Engine Battery) | Instance 2 (Alternator) |
| :---: | :--- | :--- | :--- | :--- | :--- |
| 0 | Battery Instance | `uint8` | 1 | `0` | `2` |
| 1–2 | Battery Voltage | `int16` | 0.01 V / bit | Engine battery voltage (ADC2 AIN3) | Alternator voltage (ADC1 AIN3) |
| 3–4 | Battery Current | `int16` | 0.1 A / bit | `0x7FFF` (N/A) | `0x7FFF` (N/A) |
| 5–6 | Battery Temperature | `uint16` | 0.01 K / bit | `0xFFFF` (N/A) | **Alternator Temp** from NTC2 (ADC1 AIN1) |
| 7 | Sequence ID (SID) | `uint8` | 1 | Rolling counter (0–255) | Rolling counter (0–255) |

---

### 5. PGN 127505 – Fluid Level
* **Transmission**: Single frame (8 bytes), broadcast (0xFF), Priority 6.
* **Period**: 2.5 s (nominal specification) / 4.9 s. Sent continuously.

| Byte | Field Name | Data Type | Units / Resolution | Value / Source |
| :---: | :--- | :--- | :--- | :--- |
| 0 | Fluid Type & Instance | `uint8` | Nibbles | High nibble (bits 4–7): Fluid Type = `0` (`Diesel / Fuel`).<br>Low nibble (bits 0–3): Tank Instance = `0` (`FUEL_LEVEL_INSTANCE`). |
| 1–2 | Fluid Level | `int16` | 0.004 % / bit | Measured from resistive fuel tank sender (ADC3 AIN2). Range: 0.0%–100.0%. `0x7FFF` = N/A. |
| 3–6 | Tank Capacity | `uint32` | 0.1 L / bit | Total tank capacity (default: 60.0 L). `0xFFFFFFFF` = N/A. |
| 7 | Reserved | `uint8` | — | `0xFF` |

---

### 6. PGN 130312 – Temperature (Legacy Single Frame)
* **Transmission**: Single frame (8 bytes), broadcast (0xFF), Priority 5.
* **Period**: ~4.85 s (`TEMPERATURE_UPDATE_PERIOD`). Sent continuously for each temperature channel.

| Byte | Field Name | Data Type | Units / Resolution | Value / Source |
| :---: | :--- | :--- | :--- | :--- |
| 0 | Sequence ID (SID) | `uint8` | 1 | Rolling sequence counter. |
| 1 | Temperature Instance | `uint8` | 1 | `0` |
| 2 | Temperature Source | `uint8` | Enum / ID | Source selector (see below). |
| 3–4 | Actual Temperature | `uint16` | 0.01 K / bit | Measured sensor temperature in Kelvin. `0xFFFF` = N/A. |
| 5–6 | Set / Requested Temperature | `uint16` | 0.01 K / bit | `0xFFFF` (N/A) |
| 7 | Reserved | `uint8` | — | `0xFF` |

#### Temperature Sources Emitted
* **Source 14 (`Exhaust Gas`)**: Exhaust Water Trap (NTC1) or Exhaust Elbow (NTC5).
* **Source 3 (`Engine Room`)**: Engine compartment air temperature (NTC3).
* **Source 30 (Custom / Manufacturer)**: Alternator temperature (NTC2).
* **Source 31+i (Custom / Manufacturer)**: Auxiliary 1-Wire Dallas DS18B20 digital temperature sensors (GPIO10).

---

### 7. PGN 130316 – Temperature, Extended Range
* **Transmission**: Single frame (8 bytes), broadcast (0xFF), Priority 5.
* **Period**: 2.5 s. Sent continuously. Provides 0.001 K resolution for modern NMEA2000 displays.

| Byte | Field Name | Data Type | Units / Resolution | Value / Source |
| :---: | :--- | :--- | :--- | :--- |
| 0 | Sequence ID (SID) | `uint8` | 1 | Rolling sequence counter. |
| 1 | Temperature Instance | `uint8` | 1 | Instance ID (e.g. `0`). |
| 2 | Temperature Source | `uint8` | Enum | Source selector (matches standard N2K temperature sources). |
| 3–5 | Actual Temperature | `uint24` | 0.001 K / bit | High-resolution temperature in Kelvin. `0xFFFFFF` = N/A. |
| 6–7 | Set Temperature | `uint16` | 0.1 K / bit | `0xFFFF` (N/A). |

---

### 8. Proprietary PGNs – Event History & Diagnostics
Used for interrogating fault history and resetting stored engine trip alarms over CAN.
* **Proprietary ID Code**: `0x9FFE` (Manufacturer Code 2046 + Marine Industry bit).

#### PGN 65305 (0x0FF19) – Proprietary Single Frame (Request & Simple Response)
* **Function 11 (`FN_DUMP_EVENTS`)**: Request event history.
  * Byte 0–1: `0x9FFE`
  * Byte 2: `11`
* **Function 13 (`FN_CLEAR_EVENTS`)**: Command to clear all stored non-volatile event records.
  * Byte 0–1: `0x9FFE`
  * Byte 2: `13`
* **Function 14 (`FN_CLEAR_EVENTS_RESP`)**: Clear confirmation response.
  * Byte 0–1: `0x9FFE`
  * Byte 2: `14`

#### PGN 130817 (0x1FF01) – Proprietary Fast Packet (Dump Events Response)
* **Function 12 (`FN_DUMP_EVENTS_RESP`)**: Transmits stored alarm events with runtime timestamps.
  * Bytes 0–1: `0x9FFE`
  * Byte 2: `12` (`FN_DUMP_EVENTS_RESP`)
  * Byte 3: Number of events + 1
  * Byte 4: Reserved (`0`)
  * Bytes 5–7: Current Engine Hours (`uint24`, resolution 0.001 h / bit)
  * Followed by array of 4-byte event records:
    * Byte 0: `Event ID` (`uint8`):
      * `1` = `EVENT_ENGINE_STOP`
      * `2` = `EVENT_LOW_OIL_PRES`
      * `3` = `EVENT_HIGH_COOLANT`
      * `4` = `EVENT_EXHAUST_TEMP`
      * `5` = `EVENT_ALTERNATOR_TEMP`
      * `6` = `EVENT_ENGINE_ROOM_TEMP`
    * Bytes 1–3: Timestamp when event occurred (`uint24`, resolution 0.001 h / bit).

---

### 9. Standard Network Management & ISO PGNs
* **PGN 59392 (ISO Acknowledgment)**: Positive / negative acknowledgment of ISO requests.
* **PGN 59904 (ISO Request)**: Polls for specific PGNs (Address Claim, Product Info, PGN List, etc.).
* **PGN 60928 (ISO Address Claim)**: 64-bit NAME structure containing:
  * Unique ID (serial number 21 bits), Manufacturer Code 2048 (11 bits)
  * Device Instance: `0` (or dynamically reassigned on collision)
  * Device Function: `140` (`Engine`)
  * Device Class: `50` (`Propulsion`)
  * Industry Group: `4` (`Marine`)
* **PGN 126464 (PGN List)**: Fast packet reporting supported Transmit and Receive PGN lists.
* **PGN 126996 (Product Information)**: Fast packet reporting Model ID (`"EMS"`), Software Version, Model Version, Serial Number, Certification Level `0`, Load Equivalency `1`.
* **PGN 126998 (Configuration Information)**: Fast packet reporting Manufacturer (`"Luna Technical Area"`), Installation Description 1 & 2.

