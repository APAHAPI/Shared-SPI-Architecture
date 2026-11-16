#include "sh2/Sh2Reports.h"

#include "sh2/Sh2Codes.h"

namespace bno085 {
namespace sh2 {

bool decodeRotationVector(const uint8_t* report, uint8_t length,
                          RotationVector& out) {
  if (report == nullptr) return false;
  if (length < 14) return false;
  if (report[0] != 0x05) return false;

  out.sequence = report[1];
  out.status = report[kStatusByteOffset];
  // 14-bit delay in 100 us units: status bits 7:2 are the high six bits.
  out.delayTicks = static_cast<uint16_t>(
      (static_cast<uint16_t>(report[kStatusByteOffset] & 0xFCu) << 6) |
      report[kDelayLsbOffset]);
  out.i = readI16(report + 4);
  out.j = readI16(report + 6);
  out.k = readI16(report + 8);
  out.real = readI16(report + 10);
  out.accuracyQ12 = static_cast<uint16_t>(readI16(report + 12));
  return true;
}

} // namespace sh2
} // namespace bno085
