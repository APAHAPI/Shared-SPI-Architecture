#ifndef BNO085_OUTPUT_IOUTPUTSINK_H
#define BNO085_OUTPUT_IOUTPUTSINK_H

#include <cstddef>
#include <cstdint>

namespace bno085 {

// Destination for encoded frames. Implementations should not block
// indefinitely; a short write is treated as an output overflow by the encoder.
class IOutputSink {
 public:
  virtual ~IOutputSink() {}
  virtual size_t write(const uint8_t* data, size_t len) = 0;
};

// Keeps nothing; used when no output is configured.
class NullOutputSink : public IOutputSink {
 public:
  size_t write(const uint8_t*, size_t len) override { return len; }
};

} // namespace bno085

#endif // BNO085_OUTPUT_IOUTPUTSINK_H
