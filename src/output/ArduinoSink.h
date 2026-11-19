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

  // HardwareSerial reports free TX-ring space; USB CDC reports free FIFO
  // space and zero while the host is not connected. A Stream that does not
  // implement availableForWrite() returns 0, which makes the encoder drop
  // frames instead of risking a blocking write.
  size_t availableForWrite() const override {
    const int free = stream_.availableForWrite();
    return (free > 0) ? static_cast<size_t>(free) : 0;
  }

  size_t write(const uint8_t* data, size_t len) override {
    if (len == 0) return 0;
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
