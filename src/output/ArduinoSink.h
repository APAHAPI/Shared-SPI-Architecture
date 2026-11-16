#ifndef BNO085_OUTPUT_ARDUINOSINK_H
#define BNO085_OUTPUT_ARDUINOSINK_H

#ifndef ARDUINO
#error "ArduinoSink requires an Arduino build."
#endif

#include <Arduino.h>

#include "diag/LogTypes.h"
#include "output/IOutputSink.h"

namespace bno085 {

class StreamSink : public IOutputSink {
 public:
  explicit StreamSink(Stream& stream) : stream_(stream) {}
  size_t write(const uint8_t* data, size_t len) override {
    return stream_.write(data, len);
  }

 private:
  Stream& stream_;
};

class StreamLogSink : public ILogSink {
 public:
  explicit StreamLogSink(Print& out) : out_(out) {}
  void write(const char* line) override { out_.println(line); }

 private:
  Print& out_;
};

} // namespace bno085

#endif // BNO085_OUTPUT_ARDUINOSINK_H
