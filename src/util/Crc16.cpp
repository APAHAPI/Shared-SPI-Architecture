#include "util/Crc16.h"

namespace bno085 {

uint16_t crc16CcittFalse(const uint8_t* data, size_t len, uint16_t initial) {
  uint16_t crc = initial;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000u) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

} // namespace bno085
