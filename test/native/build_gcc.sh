#!/usr/bin/env sh
# Host tests for the portable core (Linux/macOS/MSYS2).
# Run from the repository root:  sh test/native/build_gcc.sh
set -e

mkdir -p build

g++ -std=c++14 -Wall -Wextra -O2 -I src \
    test/native/test_main.cpp \
    src/BNO085Types.cpp \
    src/BNO085Sensor.cpp \
    src/diag/Logger.cpp \
    src/output/BinaryEncoder.cpp \
    src/sh2/Sh2Client.cpp \
    src/sh2/Sh2Codes.cpp \
    src/sh2/Sh2Reports.cpp \
    src/shtp/ShtpTransport.cpp \
    src/util/Crc16.cpp \
    -o build/native_tests

./build/native_tests
