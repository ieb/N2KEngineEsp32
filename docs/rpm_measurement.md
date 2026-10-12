# Flywheel RPM Measurement: Approaches and Findings

This document compares the ways engine speed can be measured from the flywheel
pickup on the ESP32-C3. It explains why the firmware now defaults to an
edge-timing implementation, and how to compare implementations on the bench.

NB: All paths in this file are relative to the root of the git repo.

## Signal chain

* A reactive (magnetic) pickup produces roughly ±20–60 V, rising with RPM
  (about 60 V peak-to-peak at 3000 RPM). See "Flyweel sense" in `docs/spec.md`.
* The signal goes through 22K + 22K series resistors, a 1nF filter capacitor
  and ±0.6 V diode clamps into an LM393 comparator. The comparator has
  hysteresis via a 470K feedback resistor and a 10K pull-up to 3V3.
* The comparator output drives GPIO4. Each tooth produces one rising edge.
* There are 30 teeth per revolution, so `RPM = Hz × 2.0` (`RPM_FACTOR`).

| Engine speed | Pulse rate | Time between pulses |
| :--- | :--- | :--- |
| Cranking, ~200 RPM | 100 Hz | 10 ms |
| Idle, 800 RPM | 400 Hz | 2.5 ms |
| 3000 RPM | 1500 Hz | 0.67 ms |

NMEA 2000 PGN 127488 carries engine speed at 0.25 RPM per bit and is sent
every 500 ms (100 ms ideal).

## Approaches considered

### 1. Window counting with a GPIO interrupt (`src/hal/RpmSensor.*`)

The original implementation. The interrupt increments a counter on each rising
edge. Every ~100 ms, `update()` divides the pulse count by the elapsed
`millis()`.

* **Quantisation:** one pulse in a 100 ms window is 20 RPM at every speed.
  That is about ±10% when cranking, ±2.5% at idle and ±0.7% at 3000 RPM.
* **Timing:** the window is measured with `millis()` (1 ms resolution), which
  adds about ±1% for a 100 ms window.
* **Result:** the reading jumps by roughly 20–40 RPM at idle.
* **Responsiveness:** the value is at most about 100–150 ms old, which is better
  than the 500 ms transmit period.
* **CPU load:** negligible. 1500 interrupts per second at a few µs each is well
  under 1%.
* **Lost pulses during flash writes:** see the dedicated section below.

### 2. PCNT hardware pulse counter: not available on the ESP32-C3

The ESP32-C3 has **no PCNT peripheral**. ESP-IDF's `soc_caps.h` defines
`SOC_PCNT_SUPPORTED` for the ESP32 (8 units) and ESP32-S3 (4 units), but not
for the ESP32-C3. A PCNT implementation therefore cannot run on this board.

On a chip that has PCNT, it would:

* count every edge in hardware, with a glitch filter and no CPU load;
* keep counting during flash writes, so no pulses would be lost.

It would **not** improve precision. The count is still read over a time
window, so the same ±1 pulse (±20 RPM) quantisation applies. PCNT also cannot
timestamp edges. To get edge-timing accuracy it would need to be paired with a
timestamp captured every N pulses (for example via a PCNT watch-point
interrupt).

### 3. Edge timing (reciprocal counting) over whole revolutions (`src/hal/EdgeRpmSensor.*`)

This is the new default implementation.

* The interrupt timestamps every rising edge with a 1 µs clock
  (`esp_timer_get_time()`). It also records the timestamp of every 30th edge,
  i.e. once per flywheel revolution.
* `update()` computes `RPM = revolutions / time between revolution edges`.
  There is no pulse-count quantisation and no dependence on main-loop timing.
* **Accuracy** is limited only by interrupt-latency jitter. A few µs over one
  revolution (20 ms at 3000 RPM, 75 ms at idle) is about 0.01–0.04%, well below
  the 0.25 RPM resolution of the PGN.
* **Averaging over whole revolutions** cancels tooth-spacing errors and the
  speed ripple from individual cylinder firings. A fixed 100 ms window is not
  aligned to revolutions, so it adds a small beat ripple.
* **Responsiveness:** a new value every revolution (20 ms at 3000 RPM, 75 ms at
  800 RPM).
* **Low speed:** if a full revolution has not completed within 250 ms
  (`RPM_EDGE_PARTIAL_TIMEOUT_US`, i.e. below about 240 RPM: cranking, start-up,
  run-down), RPM is calculated from the edges seen so far. With no edge for
  500 ms (`RPM_EDGE_ZERO_TIMEOUT_US`) RPM is 0.
* **Glitch rejection:** edges closer than 150 µs (`RPM_EDGE_MIN_PERIOD_US`,
  equivalent to 13,333 RPM) are rejected and counted (`getGlitchCount()`).
* **The interrupt is IRAM-safe**, so pulses are not lost during flash writes
  (see below).

### 4. RMT receiver capture (not implemented)

The ESP32-C3's RMT peripheral can timestamp edges in hardware. However, the
receive buffer is small (2 RX channels × 48 words, about 130 ms at 3000 RPM),
continuous capture needs ping-pong buffer handling, and the code is
considerably more complex. At these pulse rates (≤ 1.5 kHz) it offers no
practical advantage over approach 3.

## Lost pulses during flash writes

This build does not place interrupts in IRAM: `CONFIG_ARDUINO_ISR_IRAM` is not
set in the prebuilt Arduino-ESP32 sdkconfig. As a result:

* `attachInterrupt()` installs the GPIO ISR service without
  `ESP_INTR_FLAG_IRAM`, and the Arduino dispatcher (`__onPinInterrupt`) is not
  in IRAM.
* While flash is being written, non-IRAM interrupts are held off. The firmware
  originally wrote flash every 60 s while running (engine-hours NVS save) and
  on every alarm event (SPIFFS log and NVS event counter). Flash writes are now
  deferred while the engine runs (see "Flash write policy" below).
* The GPIO interrupt status is latched. However many edges arrive during the
  hold-off, only **one** interrupt fires afterwards, so the extra pulses are
  lost.
* For example, a 20 ms flash write at 3000 RPM loses about 30 pulses. With
  window counting this shows as a short RPM dip about once a minute.

`EdgeRpmSensor` avoids this as follows:

* It installs the GPIO ISR service itself with
  `gpio_install_isr_service(ESP_INTR_FLAG_IRAM)`.
* It registers an `IRAM_ATTR` handler directly with `gpio_isr_handler_add()`.
* The ISR uses only IRAM-safe calls: `esp_timer_get_time()` and the
  `portENTER_CRITICAL_ISR` spinlock.
* In the ESP32-C3 build, `EdgeRpmSensor::isrHandler`,
  `EdgeRpmSensor::recordEdge`, `esp_timer_get_time` and `vPortEnterCritical`
  all link at `0x4038xxxx` (IRAM).

> **⚠️ Other interrupts:** the new sensor installs the GPIO interrupt service in
> IRAM mode. Nothing else in this firmware uses `attachInterrupt()`. If
> something does later, its handler would be called from that IRAM-mode
> service, which can crash if the interrupt fires during a flash write.
>
> Any future GPIO interrupt user must either register its own `IRAM_ATTR`
> handler with `gpio_isr_handler_add()` (touching only IRAM/DRAM code and
> data), or the firmware must be rebuilt with `CONFIG_ARDUINO_ISR_IRAM` enabled.

**Related existing issue:** the build warns that `CONFIG_TWAI_ISR_IN_IRAM` is
not set, so the CAN (TWAI) driver's interrupt is also delayed during flash
writes. CAN reception can therefore be delayed or overrun during those
writes. This is independent of RPM measurement.

## Flash write policy

Each engine-hours save writes a few small NVS entries, holding non-IRAM
interrupts off for a fraction of a millisecond each. Roughly every 126 saves,
NVS reclaims a page with a 4 KB sector erase, which holds them off for
~30–50 ms typical and up to a few hundred ms worst case. At 60% load on a
250 kbit/s bus a frame arrives about every 0.9 ms, and the TWAI receive FIFO
holds about 4 frames. Small writes therefore lose nothing, but an erase loses
a burst of ~30–50 received frames. Transmitted frames are only delayed. BLE
(planned) is more sensitive, because the controller must service connection
events on time.

The firmware therefore avoids routine flash writes while the engine runs
(`src/storage/StorageManager.*`):

* Engine hours and alarm events are mirrored in RTC no-init memory. This
  survives crashes, watchdog resets and software restarts, but not power loss.
* Flash is written when the stop relay is energised, when the engine stops,
  and before a CLI restart. The board stays powered 30–60 s after shutdown.
* A valid RTC copy that is ahead of flash is adopted and written at boot.
* An optional backstop save of engine hours runs every
  `ENGINE_HOURS_BACKSTOP_SAVE_S` (default 3600 s, `0` disables). It protects
  against an abnormal power cut while running.
* User-initiated actions (set engine hours, clear events) write immediately.

## Simulation results

`./run_tests.sh` runs both implementations against the same simulated
flywheel signal and prints a side-by-side comparison. The simulation models:

* a main loop with a pseudo-random 1–6 ms period (sensor reads, I2C and N2K
  traffic make the real loop irregular);
* 0–8 µs of interrupt latency on each edge (edge timing only);
* frequencies that are not round numbers, so windows don't align with pulses;
* statistics sampled after a 1 s settle time, over 2 s.

| Expected RPM | Edge timing: min–max (sd) | Window counting: min–max (sd) |
| :--- | :--- | :--- |
| 202.7 | 202.73–202.75 (0.00) | 190.5–220.0 (8.2) |
| 807.4 | 807.34–807.50 (0.04) | 792.1–820.0 (7.9) |
| 2994.6 | 2993.4–2995.8 (0.61) | 2980.0–3009.9 (8.5) |

Edge timing is roughly 15–200× steadier. The simulation does **not** model
pulses lost during flash writes, so the real-board difference may be larger.
Only the bench test below can show that.

## Configuration

In `src/config.h`:

| Setting | Default | Meaning |
| :--- | :--- | :--- |
| `RPM_SENSOR_EDGE_TIMING` | `1` | `1` = `EdgeRpmSensor` (edge timing), `0` = `RpmSensor` (window counting) |
| `RPM_EDGE_MIN_PERIOD_US` | `150` | Edges closer than this are rejected as glitches |
| `RPM_EDGE_ZERO_TIMEOUT_US` | `500000` | No edge for this long means RPM 0 |
| `RPM_EDGE_PARTIAL_TIMEOUT_US` | `250000` | Use a partial revolution if a full one hasn't completed in this time |
| `PIN_BENCH_SIGNAL` | `2` | GPIO for the bench test signal |
| `BENCH_LEDC_CHANNEL` | `0` | LEDC channel for the bench test signal |

Both implementations derive from `RpmSensorBase` (`src/hal/RpmSensorBase.*`),
which holds the shared running/stopping state machine and fake-RPM mode.
`SensorProcessing` accepts either implementation.

## Bench comparison procedure

1. **Wiring:** connect GPIO2 → 1K → GPIO4, and short the flywheel input (J15)
   so the LM393 open-collector output stays released. GPIO2 is a boot-mode
   pin, but the 10K pull-up on GPIO4 keeps it high at boot as required.
2. **First build:** flash with `RPM_SENSOR_EDGE_TIMING` set to `1`. On the
   serial console (115200 baud), press `T` to step the test signal:
   off → 100 Hz → 400 Hz → 1500 Hz (200, 800 and 3000 RPM).
3. **Read the results:** after a 1 s settle time, every 2 s the console prints
   the following for the active sensor. Engine hours don't accumulate while
   the test signal is on.
   * expected RPM
   * mean and its % error
   * min and max
   * standard deviation
   * new glitch count
4. **Second build:** reflash with `RPM_SENSOR_EDGE_TIMING` set to `0` and
   repeat.
5. **Show the flash-write effect:** flash writes are now deferred while the
   engine is "running" (and engine hours do not accumulate while the bench
   signal is on), so trigger one manually instead, e.g. with `E` (set engine
   hours) or `C` (clear events) while the signal is at 1500 Hz. With window
   counting, expect occasional low readings.

The LEDC test signal and `esp_timer` both derive from the same crystal, so
crystal tolerance cancels out of the comparison.

## Recommendation

For this board, use edge timing over whole revolutions with an IRAM interrupt
(`EdgeRpmSensor`, the default). PCNT is only worth considering on a chip that
has it, such as the ESP32-S3 or ESP32-C6, and even then it should be combined
with a periodic edge timestamp to match edge-timing accuracy.
