#ifndef BNO085_UTIL_TIMEDELTA_H
#define BNO085_UTIL_TIMEDELTA_H

#include <cstdint>

namespace bno085 {

// Signed difference `a - b` between two readings of a free-running 32-bit
// microsecond counter (the ESP32 `micros()` counter, or a mock in host tests).
//
// The raw subtraction is performed modulo 2^32, so a single counter rollover
// is handled transparently. The result is then mapped into the signed 32-bit
// range: positive means `a` is later than `b`, negative means `a` is earlier.
//
// Validity limit: the two readings must be less than 2^31 us apart
// (about 35.8 minutes). At exactly half the counter range the direction is
// ambiguous by construction; no arithmetic can resolve it without a wider
// timestamp.
//
// This is deliberately a pure function: it is the only timestamp arithmetic
// the example diagnostic performs, and it carries no assumption about whether
// the values describe latency, clock skew or a rollover.
inline int32_t timeDeltaUs(uint32_t a, uint32_t b) {
  const int64_t diff =
      static_cast<int64_t>(a) - static_cast<int64_t>(b);
  // Fold the (at most) one-counter-width difference into int32_t without an
  // implementation-defined unsigned-to-signed conversion.
  if (diff > 0x7FFFFFFFLL) {
    return static_cast<int32_t>(diff - 0x100000000LL);
  }
  if (diff < -0x80000000LL) {
    return static_cast<int32_t>(diff + 0x100000000LL);
  }
  return static_cast<int32_t>(diff);
}

} // namespace bno085

#endif // BNO085_UTIL_TIMEDELTA_H
