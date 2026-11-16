#ifndef BNO085_UTIL_CRC16_H
#define BNO085_UTIL_CRC16_H

#include <cstddef>
#include <cstdint>

namespace bno085 {

// CRC-16/CCITT-FALSE: init 0xFFFF, polynomial 0x1021, no reflection,
// xor-out 0x0000. Check value for "123456789" is 0x29B1.
uint16_t crc16CcittFalse(const uint8_t* data, size_t len,
                         uint16_t initial = 0xFFFF);

} // namespace bno085

#endif // BNO085_UTIL_CRC16_H
