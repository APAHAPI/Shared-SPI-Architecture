#ifndef BNO085_SH2_REPORTS_H
#define BNO085_SH2_REPORTS_H

#include <cstdint>

namespace bno085 {
namespace sh2 {

// Decoded Rotation Vector report (report ID 0x05), fixed point preserved.
struct RotationVector {
  uint8_t sequence = 0;
  uint8_t status = 0;       // bits 1:0 accuracy, 7:2 delay exponent
  uint16_t delayTicks = 0;  // 100 us units
  int16_t i = 0;            // Q14
  int16_t j = 0;            // Q14
  int16_t k = 0;            // Q14
  int16_t real = 0;         // Q14
  uint16_t accuracyQ12 = 0; // radians, Q12
};

bool decodeRotationVector(const uint8_t* report, uint8_t length,
                          RotationVector& out);

// Shared little-endian readers. Unaligned-safe.
inline int16_t readI16(const uint8_t* p) {
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8));
}

inline uint16_t readU16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8));
}

inline uint32_t readU32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

} // namespace sh2
} // namespace bno085

#endif // BNO085_SH2_REPORTS_H
