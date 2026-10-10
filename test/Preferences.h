#pragma once
#include <cstdint>
#include <unordered_map>
#include <string>

class Preferences {
public:
    bool begin(const char *name, bool readOnly = false) { return true; }
    void end() {}
    uint32_t getUInt(const char *key, uint32_t defaultVal = 0) {
        auto it = store.find(key);
        if (it != store.end()) return it->second;
        return defaultVal;
    }
    size_t putUInt(const char *key, uint32_t value) {
        store[key] = value;
        return sizeof(value);
    }
private:
    std::unordered_map<std::string, uint32_t> store;
};
