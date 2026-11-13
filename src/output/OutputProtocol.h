#ifndef BNO085_OUTPUT_PROTOCOL_H
#define BNO085_OUTPUT_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace bno085 {
namespace protocol {

constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kVersion = 1;

enum class PacketType : uint8_t {
  Quaternion         = 0x01,
  ImuSample          = 0x02,
  SensorStatus       = 0x10,
  ManagerDiagnostics = 0x11,
  InitResult         = 0x12,
  Error              = 0x20,
  Hello              = 0x7F,
};

//  A5 5A | ver | type | len(2) | seq(2) | payload | crc16(2)
//
// The frame sequence is 16 bit so that the header stays 8 bytes: a 32-bit
// sequence at offset 6 would overlap the payload at offset 8 and grow every
// frame by 2 bytes, contradicting the 34 B quaternion frame and the 74 B
// maximum the resource analysis is built on.
constexpr size_t kHeaderBytes = 8;
constexpr size_t kSequenceOffset = 6;
constexpr size_t kMaxPayloadBytes = 64;
constexpr size_t kMaxFrameBytes = kHeaderBytes + kMaxPayloadBytes + 2;

namespace flag {
constexpr uint8_t SourceIsGrv       = 1u << 0;
constexpr uint8_t FirstAfterRecovery = 1u << 1;
constexpr uint8_t GrvFallback       = 1u << 2;
constexpr uint8_t AccuracyValid     = 1u << 3;
} // namespace flag

constexpr uint8_t kQuaternionPayloadBytes = 24;
constexpr uint8_t kImuSamplePayloadBytes = 28;
constexpr uint8_t kStatusPayloadBytes = 24;
constexpr uint8_t kManagerDiagPayloadBytes = 20;
constexpr uint8_t kInitResultPayloadBytes = 16;
constexpr uint8_t kErrorPayloadBytes = 8;
constexpr uint8_t kHelloPayloadBytes = 10;

inline void putU16(uint8_t* p, uint16_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

inline void putU32(uint8_t* p, uint32_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

inline void putI16(uint8_t* p, int16_t value) {
  putU16(p, static_cast<uint16_t>(value));
}

} // namespace protocol
} // namespace bno085

#endif // BNO085_OUTPUT_PROTOCOL_H
