# NMEA 2000 Marine Diesel Engine Interface (ESP32-C3)

This project provides an open, high-reliability NMEA 2000 marine engine interface for diesel propulsion engines. Built on the **ESP32-C3** microcontroller and **ADS1115 16-bit ADCs**, it monitors engine speed, operating pressures, temperatures, electrical systems, and alarm conditions, broadcasting them across the vessel's NMEA 2000 CAN network.

---

## High-Level Architecture

```
                    +------------------------------------+
                    |             ESP32-C3               |
                    |                                    |
Flywheel Sensor --->| GPIO4 (Pulse Interrupt)            |
Stop Relay      --->| GPIO5 (Coil Sense)                 |
Start Relay     --->| GPIO6 (Coil Sense)                 |
Glow Relay      --->| GPIO7 (Coil Sense)                 |
1-Wire DS18B20  --->| GPIO10 (Dallas Temp Bus)           |
                    |                                    |
I2C Bus (SDA/SCL) ->| GPIO8 / GPIO9                      |
                    +-----+------------------------+-----+
                          |                        |
                          v                        v
             +-------------------------+     +------------+
             | 3x ADS1115 16-bit ADCs  |     | CAN / TWAI | (GPIO3 TX, GPIO1 RX)
             | (Running at 5V via LS)  |     +-----+------+
             +------------+------------+           |
                          |                        v
                          |                 +--------------+
  NTC1: Exhaust Water Trap|                 |  NMEA 2000   |
  NTC2: Alternator Temp   |                 |     Bus      |
  NTC3: Engine Room Temp  |                 +--------------+
  NTC4: Gearbox Temp      |
  NTC5: Exhaust Elbow Temp|
  Oil Pressure (0-5V)     |
  Coolant Sensor (12V)    |
  Engine Block Offset     |
  Engine Battery Voltage  |
  Alternator B+ Voltage   |
  Fuel Sender (0-190 ohm) |
```

---

## Key Hardware Peripherals & Pinout

| Pin | Peripheral | Purpose |
| :--- | :--- | :--- |
| **GPIO1** | CAN-RX | ESP32 TWAI CAN bus receive |
| **GPIO3** | CAN-TX | ESP32 TWAI CAN bus transmit |
| **GPIO4** | Pulse Input | Flywheel magnetic pickup (30 teeth per revolution) |
| **GPIO5** | Digital Input | Stop Relay Coil (10k / 2.2k + LED divider from 12V) |
| **GPIO6** | Digital Input | Start Relay Coil (10k / 2.2k + LED divider from 12V) |
| **GPIO7** | Digital Input | Glow Plug Relay Coil (10k / 2.2k + LED divider from 12V) |
| **GPIO8** | I2C SDA | 400 kHz I2C data to 3x ADS1115 ADCs |
| **GPIO9** | I2C SCL | 400 kHz I2C clock to 3x ADS1115 ADCs |
| **GPIO10** | 1-Wire | Dallas DS18B20 digital temperature bus |

### ADC Mapping (3x ADS1115 @ 5V, 64 SPS)
* **ADC1 (`0x48`, Addr $\to$ GND)**:
  * `AIN0`: NTC1 (10K NTC, 5V 4k7 pullup) — Exhaust Water Trap
  * `AIN1`: NTC2 (10K NTC, 5V 4k7 pullup) — Alternator Temperature
  * `AIN2`: NTC3 (10K NTC, 5V 4k7 pullup) — Engine Room Air Temperature
  * `AIN3`: Alternator Voltage Divider (10k / 4.7k)
* **ADC2 (`0x49`, Addr $\to$ VDD)**:
  * `AIN0`: Oil Pressure Sender (0–5V sensor)
  * `AIN1`: Engine Coolant Sensor (12V 1k pullup)
  * `AIN2`: Engine Block Negative Offset Divider (5V — 10k — ADC — 10k — Engine Block -)
  * `AIN3`: Engine Starting Battery Voltage Divider (10k / 4.7k)
* **ADC3 (`0x4B`, Addr $\to$ SCL)**:
  * `AIN0`: NTC4 (10K NTC, 5V 4k7 pullup) — Gearbox Temperature
  * `AIN1`: NTC5 (10K NTC, 5V 4k7 pullup) — Exhaust Elbow Temperature
  * `AIN2`: Fuel Tank Level Sender (European 0–190 $\Omega$, 5V 1k pullup)
  * `AIN3`: Spare Voltage Divider (10k / 4.7k)

---

## Signal Processing & Calibrations

1. **Engine Speed (RPM)**:
   * Computed from flywheel pulses: $\text{RPM} = \text{Hz} \times 2.0$.
   * Engine is considered **Running** when $\text{RPM} \ge 800$ and **Stopping** when $\text{RPM} < 500$.
2. **10K NTC Thermistors (NTC1–NTC5)**:
   * Converted analytically using the exact Beta parameter model ($\frac{1}{T} = \frac{1}{T_0} + \frac{1}{\beta} \ln\left(\frac{R}{R_0}\right)$ with $R_0 = 10000\,\Omega$, $T_0 = 298.15\text{ K}$, $\beta = 3950\text{ K}$, $R_{\text{top}} = 4700\,\Omega$).
   * Provides smooth, continuous sub-0.01°C resolution matching the 16-bit ADS1115 without piecewise discretization or boundary kinks.
3. **Engine Block Negative Offset Compensation**:
   * Measures voltage drop between boat DC ground and engine block negative caused by heavy alternator charging current:
     $$V_{block} = 2.0 \cdot V_{adc} - 5.0\text{ V}$$
   * The Volvo Penta coolant sensor is connected to the engine block; its voltage reading and supply rail are automatically adjusted relative to $V_{block}$ to prevent false temperature spikes during high charging.
4. **Volvo Penta Coolant Temperature Sender**:
   * Evaluated through a dedicated lookup table (`coolantTable`, 10°C to 120°C) with ratiometric battery supply voltage compensation.
5. **Oil Pressure Sensor**:
   * Linear 0.5V (0 psi) to 4.5V (100 psi) transfer function with disconnection detection.
6. **Fuel Tank Level**:
   * Supports European 0–190 $\Omega$ resistive senders (0 $\Omega$ = Empty, 190 $\Omega$ = Full), scaled to 0.0%–100.0%.

---

## Alarms & Raw Water Flow Collapse Protection

* **15-Second Startup Grace Period**: Prevents spurious alarms while the engine spins up to idle.
* **Raw Water Flow Collapse Protection (Exhaust Elbow NTC5)**:
  * Evaluates rate-of-rise over a 30-second sliding anchor window. If exhaust elbow temperature climbs $\ge +8.0\text{°C}$ inside 30 seconds once above 35°C baseline, a raw water flow failure alarm is tripped immediately.
  * Absolute over-temperature alarm at $> 45.0\text{°C}$. If sustained for $> 5$ seconds, triggers an **Emergency Stop** discrete alarm.
* **Engine Protection Alarms**:
  * Coolant Over-Temperature: $> 97.0\text{°C}$ sustained $> 15$ seconds.
  * Low Oil Pressure: $< 10.0\text{ PSI}$ (68.9 kPa) sustained $> 5$ seconds while running.
  * Low System Voltage: Engine battery $< 11.8\text{ V}$ sustained $> 5$ seconds.
  * Alternator Charge Indicator: Alternator voltage $< 12.2\text{ V}$ sustained $> 5$ seconds.
  * High Alternator Temperature: Warning Level 1 triggered at $> 110.0\text{°C}$.
  * High Engine Room Temperature: Warning Level 1 triggered at $> 70.0\text{°C}$.

---

## Non-Volatile Storage & Event Logging

The flash memory uses a **2MB Application / 2MB SPIFFS** partition layout (`src/no_ota.csv`):
* **Engine Hours Accumulator**: Accumulated per second while $\text{RPM} \ge 800$, committed to wear-leveled NVS Preferences.
* **Persistent Event History**: Historical alarms are appended to `/events.csv` on the SPIFFS filesystem.
* **GPS UTC Synchronization**: Automatically synchronizes timestamps when NMEA 2000 System Time (**PGN 126992**) is received from on-bus GPS receivers.

---

## NMEA 2000 Messages & Display Remapping

To allow standard marine multifunction displays (MFDs) and dedicated instruments (such as Raymarine i70) to present all engine and electrical metrics without proprietary software:

| PGN | Name | Period | Special Mapping / Rationale |
| :--- | :--- | :--- | :--- |
| **127488** | Engine Parameters, Rapid Update | 500 ms | RPM broadcast when engine is running ($\text{RPM} \ge 800$). |
| **127489** | Engine Parameters, Dynamic | 1000 ms | **Alternator Temp (NTC2)** sent as `Engine Oil Temperature` so standard gauges display alternator thermal load. Also conveys coolant temp, alternator voltage, total engine hours, oil pressure, and discrete alarm bitfields. |
| **127493** | Transmission Parameters, Dynamic | 4.85 s | **Exhaust Water Trap (NTC1)** sent as `Transmission Oil Temperature` for displays lacking an exhaust gauge. |
| **127508** | Battery Status | 1.0 s | Broadcasts two instances:<br>• **Instance 0**: Engine Starting Battery Voltage.<br>• **Instance 2**: Alternator Output Voltage & Operating Temperature. |
| **127505** | Fluid Level | 2.5 s | Tank Instance 0, Fuel level % (60.0 L capacity default). |
| **130312** | Temperature (Legacy) | 4.85 s | Emits Exhaust Gas (Source 14), Engine Room (Source 3), Alternator (Source 30), and Dallas 1-Wire sensors (Source 31+i). |
| **130316** | Extended Temperature | 2.5 s | Emits high-resolution (0.001 K) temperatures. |

---

## Interactive Serial Console

Connect to the USB Serial port at **115200 baud** for real-time monitoring and commands:

* `h` — Show command menu
* `s` — Print complete formatted engine status dashboard
* `F` — Toggle Fake 1000 RPM mode for bench testing
* `E` — Set / calibrate accumulated engine hours
* `l` — Dump SPIFFS alarm event log
* `C` — Clear SPIFFS event history
* `d` — Toggle raw ADC voltage debug stream
* `R` — Software restart of the ESP32-C3

---

## Building & Flashing

This project uses [PlatformIO](https://platformio.org/):

```bash
# Build firmware
pio run

# Flash to ESP32-C3
pio run -t upload

# Open Serial Monitor
pio device monitor
```
