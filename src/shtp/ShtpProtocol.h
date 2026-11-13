#ifndef BNO085_SHTP_PROTOCOL_H
#define BNO085_SHTP_PROTOCOL_H

#include <cstdint>

namespace bno085 {
namespace shtp {

// SHTP channels used by the BNO08X (datasheet 1000-3927, section 1.3.1).
constexpr uint8_t kChannelCommand     = 0;  // SHTP control / advertisement
constexpr uint8_t kChannelExecutable  = 1;  // reset / reset-complete
constexpr uint8_t kChannelControl     = 2;  // SH-2 control
constexpr uint8_t kChannelReports     = 3;  // input sensor reports
constexpr uint8_t kChannelWakeReports = 4;  // wake sensor reports
constexpr uint8_t kChannelGyroRv      = 5;  // gyro rotation vector

constexpr uint8_t kMaxChannels = 8;

constexpr uint16_t kHeaderLen           = 4;
constexpr uint16_t kLengthMask          = 0x7FFFu;
constexpr uint16_t kContinuationBit     = 0x8000u;
constexpr uint16_t kReservedLength      = 0xFFFFu;  // "failed peripheral" value
constexpr uint16_t kMaxPayloadPerPacket = 0x7FFFu - kHeaderLen;

struct Header {
  uint16_t length;       // total bytes in this transfer, including the header
  uint16_t payloadLen;   // length - 4
  bool continuation;
  uint8_t channel;
  uint8_t sequence;
};

// Little-endian length field; bit 15 marks a continuation fragment.
inline Header parseHeader(const uint8_t* p) {
  Header h;
  const uint16_t raw = static_cast<uint16_t>(p[0]) |
                       static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8);
  h.continuation = (raw & kContinuationBit) != 0;
  h.length = static_cast<uint16_t>(raw & kLengthMask);
  h.payloadLen =
      (h.length >= kHeaderLen) ? static_cast<uint16_t>(h.length - kHeaderLen) : 0;
  h.channel = p[2];
  h.sequence = p[3];
  return h;
}

inline bool isSensorChannel(uint8_t channel) {
  return channel == kChannelReports || channel == kChannelWakeReports;
}

const char* channelName(uint8_t channel);

} // namespace shtp
} // namespace bno085

#endif // BNO085_SHTP_PROTOCOL_H
