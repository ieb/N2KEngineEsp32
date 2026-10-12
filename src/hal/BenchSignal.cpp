#include "BenchSignal.h"
#include <math.h>

static const uint32_t BENCH_FREQS_HZ[] = {0, 100, 400, 1500};
static const uint8_t BENCH_STEPS = sizeof(BENCH_FREQS_HZ) / sizeof(BENCH_FREQS_HZ[0]);

#define BENCH_SAMPLE_MS   10
#define BENCH_REPORT_MS   2000
#define BENCH_SETTLE_MS   1000

BenchSignal::BenchSignal()
    : freqHz(0), stepIndex(0), windowStart(0), lastSample(0), lastGlitches(0),
      n(0), sum(0.0), sumSq(0.0), minRpm(0.0), maxRpm(0.0) {
}

void BenchSignal::start(uint32_t hz) {
    ledcAttachPin(PIN_BENCH_SIGNAL, BENCH_LEDC_CHANNEL);
    uint32_t actual = ledcWriteTone(BENCH_LEDC_CHANNEL, hz);
    freqHz = hz;
    Serial.printf("[BENCH] Signal %lu Hz on GPIO%d (LEDC reports %lu Hz) -> expect %.1f RPM\n",
                  (unsigned long)hz, PIN_BENCH_SIGNAL, (unsigned long)actual, hz * RPM_FACTOR);
    // Let the sensor settle before collecting statistics
    windowStart = millis() + BENCH_SETTLE_MS;
    resetStats();
}

void BenchSignal::stop() {
    ledcWriteTone(BENCH_LEDC_CHANNEL, 0);
    ledcDetachPin(PIN_BENCH_SIGNAL);
    pinMode(PIN_BENCH_SIGNAL, INPUT);
    freqHz = 0;
    Serial.println(F("[BENCH] Signal OFF"));
}

void BenchSignal::cycle() {
    stepIndex = (stepIndex + 1) % BENCH_STEPS;
    uint32_t hz = BENCH_FREQS_HZ[stepIndex];
    if (hz == 0) {
        stop();
    } else {
        start(hz);
    }
}

void BenchSignal::resetStats() {
    n = 0;
    sum = 0.0;
    sumSq = 0.0;
    minRpm = 1e9;
    maxRpm = -1e9;
}

void BenchSignal::update(const RpmSensorBase &rpm) {
    if (freqHz == 0) return;
    unsigned long now = millis();
    if ((long)(now - windowStart) < 0) return; // settling

    if (now - lastSample >= BENCH_SAMPLE_MS) {
        lastSample = now;
        double r = rpm.getRpm();
        n++;
        sum += r;
        sumSq += r * r;
        if (r < minRpm) minRpm = r;
        if (r > maxRpm) maxRpm = r;
    }

    if (now - windowStart >= BENCH_REPORT_MS && n > 0) {
        double expected = freqHz * RPM_FACTOR;
        double mean = sum / n;
        double var = sumSq / n - mean * mean;
        double sd = var > 0.0 ? sqrt(var) : 0.0;
        uint32_t glitches = rpm.getGlitchCount();
        Serial.printf("[BENCH] %s: expect %.1f | mean %.2f (err %+.3f%%) min %.2f max %.2f sd %.2f | glitches +%lu\n",
                      rpm.name(), expected, mean, (mean - expected) * 100.0 / expected,
                      minRpm, maxRpm, sd, (unsigned long)(glitches - lastGlitches));
        lastGlitches = glitches;
        windowStart = now;
        resetStats();
    }
}
