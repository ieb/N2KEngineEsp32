#pragma once
#include <Arduino.h>
#include "config.h"

struct RelayCoilStates {
    bool stopActive  = false;
    bool startActive = false;
    bool glowActive  = false;
};

class DigitalInputs {
public:
    DigitalInputs();
    void begin();
    void update();
    const RelayCoilStates &getStates() const { return states; }

private:
    RelayCoilStates states;
    unsigned long lastReadTime;
};
