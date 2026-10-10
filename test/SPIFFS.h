#pragma once
#include "FS.h"

class SPIFFSFS : public FS {
public:
    bool begin(bool formatOnFail = false) { return true; }
    size_t totalBytes() { return 2 * 1024 * 1024; }
    size_t usedBytes() { return 1024; }
};

extern SPIFFSFS SPIFFS;
