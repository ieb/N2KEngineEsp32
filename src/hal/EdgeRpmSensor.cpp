#include "EdgeRpmSensor.h"

#if defined(ESP_PLATFORM)
#include "driver/gpio.h"
#include "esp_timer.h"
static portMUX_TYPE s_edgeMux = portMUX_INITIALIZER_UNLOCKED;
#define EDGE_ENTER_CRITICAL()     portENTER_CRITICAL(&s_edgeMux)
#define EDGE_EXIT_CRITICAL()      portEXIT_CRITICAL(&s_edgeMux)
#define EDGE_ENTER_CRITICAL_ISR() portENTER_CRITICAL_ISR(&s_edgeMux)
#define EDGE_EXIT_CRITICAL_ISR()  portEXIT_CRITICAL_ISR(&s_edgeMux)
// esp_timer_get_time() is IRAM-safe; 32-bit microseconds wrap after ~71 minutes,
// which unsigned differences handle as long as intervals are shorter than that.
static inline uint32_t IRAM_ATTR edgeNowUs() { return (uint32_t)esp_timer_get_time(); }
#else
#define EDGE_ENTER_CRITICAL()     noInterrupts()
#define EDGE_EXIT_CRITICAL()      interrupts()
#define EDGE_ENTER_CRITICAL_ISR()
#define EDGE_EXIT_CRITICAL_ISR()
static inline uint32_t edgeNowUs() { return (uint32_t)micros(); }
static void edgeIsrTrampoline() { EdgeRpmSensor::isrHandler(nullptr); }
#endif

volatile bool EdgeRpmSensor::s_haveEdge = false;
volatile uint32_t EdgeRpmSensor::s_edgeCount = 0;
volatile uint32_t EdgeRpmSensor::s_lastEdgeUs = 0;
volatile uint32_t EdgeRpmSensor::s_revCount = 0;
volatile uint32_t EdgeRpmSensor::s_revEdgeUs = 0;
volatile uint32_t EdgeRpmSensor::s_glitchCount = 0;

void IRAM_ATTR EdgeRpmSensor::recordEdge(uint32_t tUs) {
    EDGE_ENTER_CRITICAL_ISR();
    if (s_haveEdge && (uint32_t)(tUs - s_lastEdgeUs) < RPM_EDGE_MIN_PERIOD_US) {
        s_glitchCount = s_glitchCount + 1;
    } else {
        s_haveEdge = true;
        s_lastEdgeUs = tUs;
        uint32_t count = s_edgeCount + 1;
        s_edgeCount = count;
        if ((count % FLYWHEEL_TEETH) == 0) {
            s_revCount = count;
            s_revEdgeUs = tUs;
        }
    }
    EDGE_EXIT_CRITICAL_ISR();
}

void IRAM_ATTR EdgeRpmSensor::isrHandler(void *arg) {
    (void)arg;
    recordEdge(edgeNowUs());
}

void EdgeRpmSensor::resetIsrState() {
    EDGE_ENTER_CRITICAL();
    s_haveEdge = false;
    s_edgeCount = 0;
    s_lastEdgeUs = 0;
    s_revCount = 0;
    s_revEdgeUs = 0;
    s_glitchCount = 0;
    EDGE_EXIT_CRITICAL();
}

EdgeRpmSensor::EdgeRpmSensor()
    : pin(PIN_FLYWHEEL),
      haveAnchor(false),
      anchorCount(0),
      anchorUs(0),
      measuredRpm(0.0) {
}

uint32_t EdgeRpmSensor::getGlitchCount() const {
    return s_glitchCount;
}

void EdgeRpmSensor::begin(uint8_t pin) {
    this->pin = pin;
#if defined(ESP_PLATFORM)
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;     // 10K pull-up to 3V3 on the board
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&cfg);

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err == ESP_ERR_INVALID_STATE) {
        Serial.println(F("[RPM] GPIO ISR service already installed; IRAM flag may not apply"));
    } else if (err != ESP_OK) {
        Serial.printf("[RPM] gpio_install_isr_service failed: %d\n", (int)err);
    }
    err = gpio_isr_handler_add((gpio_num_t)pin, isrHandler, nullptr);
    if (err != ESP_OK) {
        Serial.printf("[RPM] gpio_isr_handler_add failed: %d\n", (int)err);
    }
#else
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), edgeIsrTrampoline, RISING);
#endif
    haveAnchor = false;
    measuredRpm = 0.0;
}

void EdgeRpmSensor::update() {
    // Consistent snapshot of the ISR state
    EDGE_ENTER_CRITICAL();
    bool haveEdge = s_haveEdge;
    uint32_t edgeCount = s_edgeCount;
    uint32_t lastEdgeUs = s_lastEdgeUs;
    uint32_t revCount = s_revCount;
    uint32_t revEdgeUs = s_revEdgeUs;
    EDGE_EXIT_CRITICAL();

    uint32_t nowUs = edgeNowUs();

    // Signed differences: an edge recorded just after the snapshot must not look
    // ~71 minutes old.
    if (!haveEdge || (int32_t)(nowUs - lastEdgeUs) > (int32_t)RPM_EDGE_ZERO_TIMEOUT_US) {
        // No pulses: flywheel at rest
        measuredRpm = 0.0;
        haveAnchor = false;
    } else if (!haveAnchor) {
        // First edge after rest: start timing from it
        anchorCount = edgeCount;
        anchorUs = lastEdgeUs;
        haveAnchor = true;
    } else {
        int32_t revPulses = (int32_t)(revCount - anchorCount);
        if (revPulses >= FLYWHEEL_TEETH) {
            // At least one full revolution since the anchor: use the revolution edge.
            // In steady state the anchor is itself a revolution edge, so this always
            // averages over an exact number of revolutions.
            uint32_t dtUs = revEdgeUs - anchorUs;
            if (dtUs > 0) {
                measuredRpm = (double)revPulses * 60.0e6 / ((double)FLYWHEEL_TEETH * (double)dtUs);
            }
            anchorCount = revCount;
            anchorUs = revEdgeUs;
        } else if ((int32_t)(nowUs - anchorUs) > (int32_t)RPM_EDGE_PARTIAL_TIMEOUT_US &&
                   edgeCount != anchorCount) {
            // Slow rotation (cranking / run-down / start-up): use the pulses seen so far
            uint32_t pulses = edgeCount - anchorCount;
            uint32_t dtUs = lastEdgeUs - anchorUs;
            if (dtUs > 0) {
                measuredRpm = (double)pulses * 60.0e6 / ((double)FLYWHEEL_TEETH * (double)dtUs);
            }
            anchorCount = edgeCount;
            anchorUs = lastEdgeUs;
        }
    }

    setMeasuredRpm(measuredRpm, millis());
}
