#!/usr/bin/env bash
set -e

echo "Building and running native unit tests..."
clang++ -std=c++17 -Wall -Wextra -Wno-unused-parameter \
    -I test -I src \
    src/sensors/SensorProcessing.cpp \
    src/alarms/AlarmManager.cpp \
    src/storage/StorageManager.cpp \
    src/hal/AdcManager.cpp \
    src/hal/RpmSensor.cpp \
    src/hal/DigitalInputs.cpp \
    src/hal/OneWireTemp.cpp \
    test/test_main.cpp \
    -o test_runner

./test_runner
rm -f test_runner events.csv
echo "Done."
