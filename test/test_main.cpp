#include <cstdio>
#include <cassert>
#include <cmath>
#include <iostream>

#include "Arduino.h"
#include "N2kMsg.h"
#include "config.h"
#include "sensors/EngineState.h"
#include "sensors/SensorProcessing.h"
#include "alarms/AlarmManager.h"
#include "storage/StorageManager.h"
#include "hal/AdcManager.h"
#include "hal/RpmSensor.h"
#include "hal/EdgeRpmSensor.h"
#include "hal/DigitalInputs.h"
#include "hal/OneWireTemp.h"

// Instantiate globals
unsigned long g_simulated_millis = 0;
MockSerial Serial;
SPIFFSFS SPIFFS;
TwoWire Wire;

// Test Statistics
static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        g_tests_failed++; \
        printf("  [FAIL] Line %d: %s\n", __LINE__, msg); \
    } \
} while(0)

#define TEST_ASSERT_NEAR(val, target, tolerance, msg) do { \
    g_tests_run++; \
    double _diff = std::abs((double)(val) - (double)(target)); \
    if (_diff <= (tolerance)) { \
        g_tests_passed++; \
    } else { \
        g_tests_failed++; \
        printf("  [FAIL] Line %d: %s (Expected %f, Got %f, Diff %f > %f)\n", \
               __LINE__, msg, (double)(target), (double)(val), _diff, (double)(tolerance)); \
    } \
} while(0)

// Helper class to expose private converter methods in SensorProcessing
class TestableSensorProcessing : public SensorProcessing {
public:
    double testNtc(float v, int16_t &deciC) { return convertNtc(v, deciC); }
    double testCoolant(float cv, float bv, float batv, int16_t &deciC) { return convertCoolant(cv, bv, batv, deciC); }
    double testOil(float v) { return convertOilPressure(v); }
    double testFuel(float v) { return convertFuelLevel(v); }
};

// ============================================================================
// 1. SENSOR PROCESSING TESTS
// ============================================================================
void run_sensor_processing_tests() {
    printf("\n=== Running Sensor Processing Tests ===\n");
    TestableSensorProcessing sp;
    int16_t deciC = 0;

    // --- NTC Thermistor Tests ---
    // 25C: R = 10000 ohm. V = 5.0 * 10000 / (4700 + 10000) = 3.40136V
    float v25 = 5.0f * 10000.0f / (4700.0f + 10000.0f);
    double k25 = sp.testNtc(v25, deciC);
    TEST_ASSERT_NEAR(k25 - 273.15, 25.0, 0.1, "NTC 25C calibration");
    TEST_ASSERT_NEAR(deciC, 250, 1, "NTC deciC at 25C");

    // 0C: R = 10000 * exp(3950 * (1/273.15 - 1/298.15)) = 33620.6 ohm
    // V = 5.0 * 33620.6 / (4700 + 33620.6) = 4.3867V
    float r0 = 10000.0f * expf(3950.0f * (1.0f / 273.15f - 1.0f / 298.15f));
    float v0 = 5.0f * r0 / (4700.0f + r0);
    double k0 = sp.testNtc(v0, deciC);
    TEST_ASSERT_NEAR(k0 - 273.15, 0.0, 0.2, "NTC 0C calibration");

    // 85C: R = 10000 * exp(3950 * (1/(85+273.15) - 1/298.15)) = 1086.6 ohm
    // V = 5.0 * 1086.6 / (4700 + 1086.6) = 0.9389V
    float r85 = 10000.0f * expf(3950.0f * (1.0f / (85.0f + 273.15f) - 1.0f / 298.15f));
    float v85 = 5.0f * r85 / (4700.0f + r85);
    double k85 = sp.testNtc(v85, deciC);
    TEST_ASSERT_NEAR(k85 - 273.15, 85.0, 0.2, "NTC 85C calibration");

    // Open-circuit / Disconnected wire (e.g. 4.98V)
    double kOpen = sp.testNtc(4.98f, deciC);
    TEST_ASSERT(kOpen == N2kDoubleNA, "NTC open-circuit returns N2kDoubleNA");
    TEST_ASSERT(deciC == -9999, "NTC open-circuit deciC is -9999");

    // Short-circuit (e.g. 0.02V)
    double kShort = sp.testNtc(0.02f, deciC);
    TEST_ASSERT(kShort == N2kDoubleNA, "NTC short-circuit returns N2kDoubleNA");

    // --- Oil Pressure Tests ---
    // 0.5V = 0 Pa (0 PSI)
    TEST_ASSERT_NEAR(sp.testOil(0.5f), 0.0, 1.0, "Oil pressure at 0.5V is 0 Pa");

    // 2.5V = 2.0 * 172369 = 344738 Pa (50.0 PSI)
    TEST_ASSERT_NEAR(sp.testOil(2.5f), 344738.0, 10.0, "Oil pressure at 2.5V is 50 PSI (344.7 kPa)");

    // 4.5V = 4.0 * 172369 = 689476 Pa (100.0 PSI)
    TEST_ASSERT_NEAR(sp.testOil(4.5f), 689476.0, 10.0, "Oil pressure at 4.5V is 100 PSI (689.5 kPa)");

    // < 0.2V: Disconnected sensor
    TEST_ASSERT(sp.testOil(0.15f) == N2kDoubleNA, "Oil pressure disconnected returns N2kDoubleNA");

    // --- Fuel Level Tests ---
    // 0.0V: Empty
    TEST_ASSERT_NEAR(sp.testFuel(0.005f), 0.0, 0.1, "Fuel level at 0V is 0%");

    // Full: R = 190 ohm, V = 5.0 * 190 / (1000 + 190) = 0.7983V
    float vFull = 5.0f * 190.0f / (1000.0f + 190.0f);
    TEST_ASSERT_NEAR(sp.testFuel(vFull), 100.0, 0.5, "Fuel level at 190 ohm is 100%");

    // Half tank: R = 95 ohm, V = 5.0 * 95 / (1000 + 95) = 0.4338V
    float vHalf = 5.0f * 95.0f / (1000.0f + 95.0f);
    TEST_ASSERT_NEAR(sp.testFuel(vHalf), 50.0, 1.0, "Fuel level at 95 ohm is 50%");

    // Open circuit (V >= 4.5V)
    TEST_ASSERT(sp.testFuel(4.8f) == N2kDoubleNA, "Fuel level open-circuit returns N2kDoubleNA");

    // --- Engine Block Offset & Coolant Tests ---
    // Test Volvo Penta coolant sensor at 60C
    // At 60C, table has 1162 counts on 4096 scale at 12V supply.
    // 1162 / 4096 * 5.0V = 1.4184V.
    // When Engine Block = 0V:
    double kCoolant = sp.testCoolant(1.4184f, 0.0f, 12.0f, deciC);
    TEST_ASSERT_NEAR(kCoolant - 273.15, 60.0, 1.5, "Coolant at 60C with 0V block offset");

    // When Engine Block has -0.5V drop (blockV = -0.5V):
    // Sensor voltage to GND is 1.4184 - 0.5 = 0.9184V.
    // Battery supply to GND is 12.0 - 0.5 = 11.5V.
    // The offset compensation should restore true 60C!
    double kCoolantOffset = sp.testCoolant(0.9184f, -0.5f, 11.5f, deciC);
    TEST_ASSERT_NEAR(kCoolantOffset - 273.15, 60.0, 1.5, "Coolant at 60C with -0.5V block offset correctly adjusted");

    // Supply rail too low (< 5.0V)
    TEST_ASSERT(sp.testCoolant(1.4f, 0.0f, 4.0f, deciC) == N2kDoubleNA, "Coolant supply < 5V returns N2kDoubleNA");

    // Shorted sender must not read as 120C
    TEST_ASSERT(sp.testCoolant(0.0f, 0.0f, 12.0f, deciC) == N2kDoubleNA, "Coolant shorted sender returns N2kDoubleNA");
    TEST_ASSERT(deciC == -9999, "Coolant shorted sender deciC is -9999");

    // Sense node at the 5V clamp (open sender / too cold to measure)
    TEST_ASSERT(sp.testCoolant(5.2f, 0.0f, 12.0f, deciC) == N2kDoubleNA, "Coolant at clamp returns N2kDoubleNA");

    // --- Full update(): disconnected engine block wire and missing ADC chips ---
    {
        EngineState st;
        AdcReadings adc;
        adc.adc1Ok = adc.adc2Ok = adc.adc3Ok = true;
        adc.coolant_V = 1.4184f;              // 60C
        adc.engineBattery_V = 12.0f / (float)VOLTAGE_SCALE;
        adc.engineBlock_V = 5.0f;              // block wire disconnected (offset +5V)
        RpmSensor rpm;
        RelayCoilStates relays;
        OneWireTemp ow;
        sp.update(st, adc, rpm, relays, ow);
        TEST_ASSERT_NEAR(st.coolantTempK - 273.15, 60.0, 1.5, "Coolant ignores implausible block offset");

        // Ignition off: switched battery feed ~0V reports N/A, and coolant is N/A too
        adc.engineBattery_V = 0.01f;
        adc.engineBlock_V = 2.5f;
        sp.update(st, adc, rpm, relays, ow);
        TEST_ASSERT(st.engineBatteryVoltage == N2kDoubleNA, "Switched battery feed off reports N/A");
        TEST_ASSERT(st.coolantTempK == N2kDoubleNA, "Coolant N/A with battery feed off");

        AdcReadings missing; // all chips absent
        sp.update(st, missing, rpm, relays, ow);
        TEST_ASSERT(st.engineBatteryVoltage == N2kDoubleNA, "Missing ADC2: battery voltage N/A");
        TEST_ASSERT(st.alternatorVoltage == N2kDoubleNA, "Missing ADC1: alternator voltage N/A");
        TEST_ASSERT(st.coolantTempK == N2kDoubleNA, "Missing ADC2: coolant N/A");
        TEST_ASSERT(st.fuelLevelPercent == N2kDoubleNA, "Missing ADC3: fuel level N/A");
        TEST_ASSERT(st.oilPressurePa == N2kDoubleNA, "Missing ADC2: oil pressure N/A");
    }
}

// ============================================================================
// 2. ALARM MANAGER TESTS
// ============================================================================
void run_alarm_manager_tests() {
    printf("\n=== Running Alarm Manager Tests ===\n");
    AlarmManager am;
    StorageManager sm;
    EngineState state;

    set_simulated_millis(0);

    // Initial state: Engine STOPPED
    state.isEngineRunning = false;
    state.isEngineStopping = false;
    state.engineRpm = 0.0;
    state.oilPressurePa = 0.0; // Low, but engine stopped
    state.engineBatteryVoltage = 11.0; // Low, but engine stopped
    am.update(state, sm);

    TEST_ASSERT((state.discreteStatus1 & STATUS1_CHECK_ENGINE) == 0, "No alarms when engine stopped");
    TEST_ASSERT((state.discreteStatus1 & STATUS1_LOW_OIL_PRES) == 0, "No low oil alarm when engine stopped");

    // --- Startup Grace Period Test ---
    // Start engine at t = 1000 ms
    set_simulated_millis(1000);
    state.isEngineRunning = true;
    state.engineRpm = 1000.0;
    state.oilPressurePa = 30000.0; // Low (< 10 PSI)
    am.update(state, sm);

    // Advance 5 seconds to t = 6000 ms (inside 15s grace period)
    set_simulated_millis(6000);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_LOW_OIL_PRES) == 0, "Alarms suppressed during 15s grace period");

    // Advance past grace period to t = 17000 ms (16s since start)
    set_simulated_millis(17000);
    am.update(state, sm);
    // Low oil window is 5s, started at t = 17000ms, so not yet tripped
    TEST_ASSERT((state.discreteStatus1 & STATUS1_LOW_OIL_PRES) == 0, "Low oil starts timer after grace period");

    // Advance 5.1s to t = 22100 ms
    set_simulated_millis(22100);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_LOW_OIL_PRES) != 0, "Low oil trips after 5s sustained");
    TEST_ASSERT((state.discreteStatus1 & STATUS1_CHECK_ENGINE) != 0, "Check Engine set on low oil");
    TEST_ASSERT((state.discreteStatus2 & STATUS2_MAINTENANCE_NEEDED) != 0, "Maintenance Needed set on low oil");

    // Restore good oil pressure (200 kPa = ~29 PSI)
    state.oilPressurePa = 200000.0;
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_LOW_OIL_PRES) == 0, "Low oil clears immediately when pressure restored");

    // --- Coolant Over-Temperature Test ---
    state.coolantTemp_dC = 985; // 98.5C (> 97.0C)
    set_simulated_millis(25000);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_OVERTEMP) == 0, "Coolant overtemp starts 15s timer");

    // Advance 14s (not yet tripped)
    set_simulated_millis(39000);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_OVERTEMP) == 0, "Coolant overtemp not tripped at 14s");

    // Advance to 15.1s
    set_simulated_millis(40200);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_OVERTEMP) != 0, "Coolant overtemp trips after 15s sustained");

    // Clear coolant temp
    state.coolantTemp_dC = 850; // 85C
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_OVERTEMP) == 0, "Coolant overtemp clears when cooled");

    // --- Raw Water Flow Collapse Rate-of-Rise Test (Exhaust Elbow NTC5) ---
    // Warmup: temperature rises below 35C baseline (e.g. 20C -> 30C)
    state.exhaustElbowTemp_dC = 300; // 30C
    set_simulated_millis(50000);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_WATER_FLOW) == 0, "Rate-of-rise suppressed below 35C baseline");

    // Elbow reaches steady state: 36.0C at t = 60000 ms
    state.exhaustElbowTemp_dC = 360;
    set_simulated_millis(60000);
    am.update(state, sm);

    // Sudden rise: +8.5C in 25s (reaches 44.5C at t = 85000 ms)
    state.exhaustElbowTemp_dC = 445;
    set_simulated_millis(85000);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_WATER_FLOW) != 0, "Raw water flow collapse trips on +8.5C rise in 25s");

    // Absolute trip test (> 45.0C)
    state.exhaustElbowTemp_dC = 460; // 46C
    set_simulated_millis(90000);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_WATER_FLOW) != 0, "Water flow tripped on > 45C");
    TEST_ASSERT((state.discreteStatus1 & STATUS1_EMERGENCY_STOP) == 0, "Emergency stop not tripped before 5s");

    // Sustain for 5.1s
    set_simulated_millis(95200);
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_EMERGENCY_STOP) != 0, "Emergency stop trips on > 45C for > 5s");

    // Hysteresis: cool to 40C (still >= 38C clear threshold)
    state.exhaustElbowTemp_dC = 400;
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_WATER_FLOW) != 0, "Water flow remains latched above 38C clear point");

    // Cool to 37C (< 38C clear threshold)
    state.exhaustElbowTemp_dC = 370;
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_WATER_FLOW) == 0, "Water flow clears when elbow drops below 38C");

    // --- Alternator & Engine Room Over-Temperature ---
    state.alternatorTemp_dC = 1120; // 112C (> 110C)
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus2 & STATUS2_WARN_1) != 0, "Warning Level 1 trips when alternator > 110C");

    state.alternatorTemp_dC = 450; // cools < 50C
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus2 & STATUS2_WARN_1) == 0, "Warning Level 1 clears when alternator cools");

    // --- Preheat & Stop Relays ---
    state.glowRelayActive = true;
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus1 & STATUS1_PREHEAT_INDICATOR) != 0, "Preheat indicator active when glow coil on");

    state.glowRelayActive = false;
    state.stopRelayActive = true;
    am.update(state, sm);
    TEST_ASSERT((state.discreteStatus2 & STATUS2_ENGINE_SHUTTING_DOWN) != 0, "Engine Shutting Down active when stop coil on");

    // --- Event latches reset between runs ---
    {
        AlarmManager am2;
        StorageManager sm2;
        sm2.begin();
        sm2.clearEvents();
        EngineState st;
        st.oilPressurePa = 30000.0; // low
        st.engineBatteryVoltage = 12.5;
        st.alternatorVoltage = 14.0;

        // Run 1: low oil trips and is logged, then engine stops with alarm active
        set_simulated_millis(200000);
        st.isEngineRunning = true; st.engineRpm = 1000.0;
        am2.update(st, sm2);
        set_simulated_millis(216000); am2.update(st, sm2);
        set_simulated_millis(222000); am2.update(st, sm2);
        uint32_t afterRun1 = sm2.getEventCount();
        TEST_ASSERT(afterRun1 == 1, "Run 1 logs low oil once");

        st.isEngineRunning = false; st.isEngineStopping = true; st.engineRpm = 300.0;
        set_simulated_millis(223000); am2.update(st, sm2);
        TEST_ASSERT((st.discreteStatus2 & STATUS2_ENGINE_SHUTTING_DOWN) != 0, "Shutting down during RPM run-down");
        TEST_ASSERT(sm2.getEventCount() == afterRun1 + 1, "Engine stop logged once");
        st.isEngineStopping = false; st.engineRpm = 0.0;
        set_simulated_millis(224000); am2.update(st, sm2);
        TEST_ASSERT((st.discreteStatus2 & STATUS2_ENGINE_SHUTTING_DOWN) == 0, "Shutting down clears at rest");
        TEST_ASSERT(sm2.getEventCount() == afterRun1 + 1, "No duplicate stop event at rest");

        // Run 2: same low oil condition must be logged again
        set_simulated_millis(300000);
        st.isEngineRunning = true; st.engineRpm = 1000.0;
        am2.update(st, sm2);
        set_simulated_millis(316000); am2.update(st, sm2);
        set_simulated_millis(322000); am2.update(st, sm2);
        TEST_ASSERT(sm2.getEventCount() == afterRun1 + 2, "Run 2 logs low oil again");

        // Event dump reader returns the logged events oldest first
        StorageManager::EventRecord recs[2];
        uint8_t n = sm2.getRecentEvents(recs, 2);
        TEST_ASSERT(n == 2, "getRecentEvents returns last 2 events");
        TEST_ASSERT(n == 2 && recs[0].eventId == EVENT_ENGINE_STOP && recs[1].eventId == EVENT_LOW_OIL_PRES,
                    "getRecentEvents order is oldest first");
    }
}

// ============================================================================
// 3. STORAGE & EVENT LOGGING TESTS
// ============================================================================
void run_storage_tests() {
    printf("\n=== Running Storage & Event Logging Tests ===\n");
    StorageManager sm;
    sm.begin();

    // Engine hours test
    sm.setEngineHoursSeconds(3600); // 1.0 hour
    TEST_ASSERT(sm.getEngineHoursSeconds() == 3600, "Engine hours set to 3600s");

    sm.tickEngineHours(10);
    TEST_ASSERT(sm.getEngineHoursSeconds() == 3610, "Engine hours ticked to 3610s");

    // Event logging test
    sm.clearEvents();
    TEST_ASSERT(sm.getEventCount() == 0, "Event count after clear is 0");

    sm.logEvent(EVENT_LOW_OIL_PRES, 1.002, 1700000000);
    TEST_ASSERT(sm.getEventCount() == 1, "Event count after log is 1");

    sm.logEvent(EVENT_EXHAUST_TEMP, 1.005, 1700000030);
    TEST_ASSERT(sm.getEventCount() == 2, "Event count after second log is 2");
}

// ============================================================================
// 4. RPM SENSOR COMPARISON TESTS
// ============================================================================
struct RpmStats { double mean, minV, maxV, sd; int n; };

// Simulates a square wave of freqHz for durationMs in 1 ms loop steps, feeding
// edges to the given sensor and sampling getRpm() after a 500 ms settle time.
// The main loop is modelled with a deterministic pseudo-random 1..6 ms period
// (sensor reads, I2C, N2K traffic make the real loop irregular).
static RpmStats simulateRpm(RpmSensorBase &sensor, bool edgeImpl, double freqHz,
                            unsigned long startMs, unsigned long durationMs) {
    double periodUs = 1.0e6 / freqHz;
    double nextEdgeUs = (double)startMs * 1000.0 + periodUs * 0.37;
    RpmStats st = {0, 1e9, -1e9, 0, 0};
    double sumSq = 0.0;
    uint32_t lcg = 12345;
    uint32_t jit = 777;
    unsigned long t = startMs;
    while (t < startMs + durationMs) {
        lcg = lcg * 1103515245u + 12345u;
        t += 1 + ((lcg >> 16) % 6);
        set_simulated_millis(t);
        double limitUs = (double)t * 1000.0;
        while (nextEdgeUs < limitUs) {
            if (edgeImpl) {
                // Model 0..8 us of interrupt latency (other ISRs, TWAI, flash cache)
                jit = jit * 1103515245u + 12345u;
                EdgeRpmSensor::recordEdge((uint32_t)(nextEdgeUs + 0.5) + ((jit >> 16) % 9));
            } else {
                RpmSensor::isrHandler();
            }
            nextEdgeUs += periodUs;
        }
        sensor.update();
        if (t - startMs >= 1000) {
            double r = sensor.getRpm();
            st.mean += r; sumSq += r * r; st.n++;
            if (r < st.minV) st.minV = r;
            if (r > st.maxV) st.maxV = r;
        }
    }
    st.mean /= st.n;
    double var = sumSq / st.n - st.mean * st.mean;
    st.sd = var > 0 ? sqrt(var) : 0;
    return st;
}

void run_rpm_sensor_tests() {
    printf("\n=== Running RPM Sensor Tests ===\n");
    // Deliberately not round numbers: ~203, ~807 and ~2995 RPM
    const double freqs[] = {101.37, 403.71, 1497.29};
    unsigned long t0 = 1000000;

    printf("  %-13s %8s %10s %10s %10s %8s\n", "impl", "expect", "mean", "min", "max", "sd");
    for (double f : freqs) {
        double expected = f * RPM_FACTOR;

        EdgeRpmSensor::resetIsrState();
        EdgeRpmSensor edge;
        set_simulated_millis(t0);
        edge.begin();
        RpmStats e = simulateRpm(edge, true, f, t0, 3000);
        t0 += 10000;

        RpmSensor win;
        set_simulated_millis(t0);
        win.begin();
        RpmStats w = simulateRpm(win, false, f, t0, 3000);
        t0 += 10000;

        printf("  %-13s %8.1f %10.2f %10.2f %10.2f %8.2f\n", edge.name(), expected, e.mean, e.minV, e.maxV, e.sd);
        printf("  %-13s %8.1f %10.2f %10.2f %10.2f %8.2f\n", win.name(), expected, w.mean, w.minV, w.maxV, w.sd);

        // 8 us latency jitter over one revolution (20 ms at 3000 RPM) is <= 0.04%
        TEST_ASSERT_NEAR(e.minV, expected, expected * 0.0005, "Edge timing min within 0.05%");
        TEST_ASSERT_NEAR(e.maxV, expected, expected * 0.0005, "Edge timing max within 0.05%");
        TEST_ASSERT_NEAR(w.mean, expected, 25.0, "Window counting mean within 25 RPM");
    }

    // Glitch rejection: a spurious edge 50 us after a real one is ignored
    {
        EdgeRpmSensor::resetIsrState();
        EdgeRpmSensor edge;
        set_simulated_millis(t0);
        edge.begin();
        simulateRpm(edge, true, 400.0, t0, 1000);
        uint32_t lastUs = (uint32_t)((double)t0 * 1000.0); // any time; only the gap matters
        (void)lastUs;
        unsigned long now = t0 + 1000;
        EdgeRpmSensor::recordEdge((uint32_t)(now * 1000UL) - 10);
        EdgeRpmSensor::recordEdge((uint32_t)(now * 1000UL) + 40);
        TEST_ASSERT(edge.getGlitchCount() == 1, "Edge timing rejects edge closer than min period");
        t0 += 10000;
    }

    // Run-down and stop: running -> stopping -> stopped after pulses cease
    {
        EdgeRpmSensor::resetIsrState();
        EdgeRpmSensor edge;
        set_simulated_millis(t0);
        edge.begin();
        simulateRpm(edge, true, 400.0, t0, 1000);
        TEST_ASSERT(edge.isEngineRunning(), "Edge timing: running at 800 RPM");
        // Run down through 400 RPM, then pulses stop
        simulateRpm(edge, true, 200.0, t0 + 1000, 1000);
        TEST_ASSERT(edge.isEngineStopping(), "Edge timing: stopping state during run-down");
        unsigned long t = t0 + 2000;
        for (int i = 0; i < 700; i++) {
            set_simulated_millis(++t);
            edge.update();
        }
        TEST_ASSERT(edge.getRpm() == 0.0 && !edge.isEngineRunning() && !edge.isEngineStopping(),
                    "Edge timing: RPM 0 and stopped after pulses cease");
        t0 += 10000;
    }
}

int main() {
    printf("==================================================\n");
    printf("  N2KEngineEsp32 Native Local Unit Test Suite\n");
    printf("==================================================\n");

    run_sensor_processing_tests();
    run_alarm_manager_tests();
    run_storage_tests();
    run_rpm_sensor_tests();

    printf("\n==================================================\n");
    printf("Test Results: %d / %d passed", g_tests_passed, g_tests_run);
    if (g_tests_failed == 0) {
        printf(" (ALL TESTS PASSED)\n");
    } else {
        printf(" (%d FAILED)\n", g_tests_failed);
    }
    printf("==================================================\n");

    return (g_tests_failed == 0) ? 0 : 1;
}
