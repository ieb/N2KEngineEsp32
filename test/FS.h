#pragma once
#include "Arduino.h"
#include <cstdio>
#include <string>

#define FILE_READ   "r"
#define FILE_WRITE  "w"
#define FILE_APPEND "a"

class File : public Print {
public:
    File(FILE *f = nullptr) : fp(f) {}
    operator bool() const { return fp != nullptr; }
    void close() { if (fp) { fclose(fp); fp = nullptr; } }
    bool available() {
        if (!fp) return false;
        int c = fgetc(fp);
        if (c == EOF) return false;
        ungetc(c, fp);
        return true;
    }
    int read() { return fp ? fgetc(fp) : -1; }
    size_t write(uint8_t c) override { return fp ? fputc(c, fp) : 0; }
    size_t write(const uint8_t *buf, size_t size) override { return fp ? fwrite(buf, 1, size, fp) : 0; }
    template<typename... Args>
    void printf(const char *fmt, Args... args) { if (fp) fprintf(fp, fmt, args...); }
private:
    FILE *fp;
};

class FS {
public:
    virtual bool exists(const char *path) {
        const char *localPath = (path && path[0] == '/') ? path + 1 : path;
        FILE *f = fopen(localPath, "r");
        if (f) { fclose(f); return true; }
        return false;
    }
    virtual bool remove(const char *path) {
        const char *localPath = (path && path[0] == '/') ? path + 1 : path;
        return ::remove(localPath) == 0;
    }
    virtual File open(const char *path, const char *mode = FILE_READ) {
        const char *localPath = (path && path[0] == '/') ? path + 1 : path;
        FILE *f = fopen(localPath, mode);
        return File(f);
    }
};

