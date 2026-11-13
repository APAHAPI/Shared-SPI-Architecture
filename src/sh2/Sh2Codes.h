#ifndef BNO085_SH2_CODES_H
#define BNO085_SH2_CODES_H

#include <cstdint>

namespace bno085 {
namespace sh2 {

// --- SH-2 control channel report IDs -------------------------------------
constexpr uint8_t kProdIdReq           = 0xF9;  // host -> device
constexpr uint8_t kProdIdResp          = 0xF8;  // device -> host
constexpr uint8_t kCommandReq          = 0xF2;  // host -> device
constexpr uint8_t kCommandResp         = 0xF1;  // device -> host
constexpr uint8_t kSetFeatureCmd       = 0xFD;  // host -> device
constexpr uint8_t kGetFeatureReq       = 0xFE;  // host -> device
constexpr uint8_t kGetFeatureResp      = 0xFC;  // device -> host
constexpr uint8_t kBaseTimestampRef    = 0xFA;  // device -> host
constexpr uint8_t kTimestampRebase     = 0xFB;  // device -> host

// --- Executable channel commands -----------------------------------------
constexpr uint8_t kExecReset           = 0x01;
constexpr uint8_t kExecOn              = 0x02;
constexpr uint8_t kExecSleep           = 0x03;
constexpr uint8_t kExecResetComplete   = 0x01;

// --- Fixed lengths -------------------------------------------------------
constexpr uint8_t kProdIdRespLen       = 16;
constexpr uint8_t kCommandRespLen      = 16;
constexpr uint8_t kGetFeatureRespLen   = 17;
constexpr uint8_t kTimestampRefLen     = 5;
constexpr uint8_t kSetFeatureLen       = 17;

// Length of one report of this ID, or 0 when the ID is unknown.
// Values follow the SH-2 reference report layout.
uint8_t reportLength(uint8_t reportId);

// True for report IDs that carry sensor data (not control responses).
bool isSensorReport(uint8_t reportId);

// Offset of the 16-bit delay field inside a sensor report header.
constexpr uint8_t kStatusByteOffset = 2;
constexpr uint8_t kDelayLsbOffset = 3;

} // namespace sh2
} // namespace bno085

#endif // BNO085_SH2_CODES_H
