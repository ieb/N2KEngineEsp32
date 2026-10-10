#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>

#define HIGH 1
#define LOW  0
#define INPUT 0
#define OUTPUT 1
#define RISING 1
#define FALLING 2
#define CHANGE 3
#define IRAM_ATTR

extern unsigned long g_simulated_millis;
inline unsigned long millis() { return g_simulated_millis; }
inline unsigned long micros() { return g_simulated_millis * 1000UL; }
inline void set_simulated_millis(unsigned long ms) { g_simulated_millis = ms; }
inline void advance_simulated_millis(unsigned long ms) { g_simulated_millis += ms; }
inline void delay(unsigned long) {}

inline void pinMode(int, int) {}
inline int digitalRead(int) { return 0; }
inline void digitalWrite(int, int) {}
inline int digitalPinToInterrupt(int p) { return p; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void interrupts() {}
inline void noInterrupts() {}

class Print {
public:
    virtual size_t write(uint8_t) { return 1; }
    virtual size_t write(const uint8_t *buffer, size_t size) { (void)buffer; return size; }
    void print(const char *s) { printf("%s", s); }
    void print(double d) { printf("%f", d); }
    void print(int i) { printf("%d", i); }
    void println(const char *s = "") { printf("%s\n", s); }
    void println(double d) { printf("%f\n", d); }
    void println(int i) { printf("%d\n", i); }
    template<typename... Args>
    void printf(const char *fmt, Args... args) { ::printf(fmt, args...); }
};

class MockSerial : public Print {};
extern MockSerial Serial;

#define F(x) x
