#ifndef BNO085_OUTPUT_IOUTPUTSINK_H
#define BNO085_OUTPUT_IOUTPUTSINK_H

#include <cstddef>
#include <cstdint>

namespace bno085 {

// Destination for encoded frames.
//
// `availableForWrite()` is the number of bytes the sink can accept right now.
// The encoder asks for it *before* writing, so a full sink causes the frame to
// be dropped atomically instead of emitting a partial frame or blocking.
// A sink that cannot report capacity may return `npos`, which means "attempt
// the write"; the encoder then relies on `write()`'s return value.
//
// `write()` returns the number of bytes accepted. It must not block
// indefinitely. Returning less than `len` is reported as a partial write; the
// encoder never retries the remainder.
class IOutputSink {
 public:
  static constexpr size_t npos = static_cast<size_t>(-1);

  virtual ~IOutputSink() {}
  virtual size_t availableForWrite() const { return npos; }
  virtual size_t write(const uint8_t* data, size_t len) = 0;
};

// Keeps nothing; used when no output is configured. Reports no frames as lost
// because nothing was ever expected to reach it.
class NullOutputSink : public IOutputSink {
 public:
  size_t availableForWrite() const override { return npos; }
  size_t write(const uint8_t*, size_t len) override { return len; }
};

} // namespace bno085

#endif // BNO085_OUTPUT_IOUTPUTSINK_H
