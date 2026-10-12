#pragma once
#include <cstdint>
#include <unordered_map>
#include <string>

// NVS mock. The store is shared by all instances (like real flash) so tests can
// simulate a reboot by constructing a new StorageManager. s_writeCount counts
// flash writes so tests can assert that none happen while the engine runs.
class Preferences {
public:
    bool begin(const char *name, bool readOnly = false) { return true; }
    void end() {}
    uint32_t getUInt(const char *key, uint32_t defaultVal = 0) {
        auto it = store().find(key);
        if (it != store().end()) return it->second;
        return defaultVal;
    }
    size_t putUInt(const char *key, uint32_t value) {
        store()[key] = value;
        s_writeCount++;
        return sizeof(value);
    }
    static void clearAll() { store().clear(); }
    static inline uint32_t s_writeCount = 0;
private:
    static std::unordered_map<std::string, uint32_t> &store() {
        static std::unordered_map<std::string, uint32_t> s;
        return s;
    }
};
