#pragma once
#include <cstdint>

class TwoWire {
public:
    void begin(int sda = 0, int scl = 0, uint32_t freq = 100000) {}
};

extern TwoWire Wire;
