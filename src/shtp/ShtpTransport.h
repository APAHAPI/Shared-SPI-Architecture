#ifndef BNO085_SHTP_TRANSPORT_H
#define BNO085_SHTP_TRANSPORT_H

#include <cstddef>
#include <cstdint>

#include "BNO085Config.h"
#include "hal/IHal.h"
#include "shtp/ShtpProtocol.h"

namespace bno085 {

class Logger;

enum class ShtpResult : uint8_t {
  NoData = 0,   // INT not asserted, or the device had nothing
  Ok,           // a complete payload was delivered
  Fragment,     // a payload fragment was stored; more transfers required
  Spurious,     // INT asserted but the header length was zero
  Malformed,    // header/channel/fragment violated the protocol
  Oversize,     // declared length exceeded the transfer/payload buffers
  BusError,     // the HAL reported a bus-level failure
};

struct ShtpStats {
  uint32_t packets = 0;
  uint32_t bytes = 0;
  uint32_t txPackets = 0;
  uint32_t rxErrors = 0;
  uint32_t txErrors = 0;
  uint16_t shortFragments = 0;
  uint16_t badChannels = 0;
  uint16_t badFragments = 0;
  uint16_t oversizePayloads = 0;
  uint16_t sequenceGaps = 0;
  uint16_t spurious = 0;
  uint16_t lastChannel = 0;
  uint16_t lastLength = 0;
  uint32_t lastTimestampUs = 0;
};

// One instance per sensor. Owns sequence numbers, reassembly state and buffers;
// shares nothing with any other instance.
class ShtpTransport {
 public:
  using ChannelHandler = void (*)(void* context, uint8_t channel,
                                  const uint8_t* payload, uint16_t len,
                                  uint32_t tUs);

  ShtpTransport();

  void attach(IHal* hal, Logger* log, uint8_t sensorId);
  void setHandler(ChannelHandler handler, void* context);

  // Clears protocol state (sequences, partial payload). Statistics are kept.
  void reset();

  // Reads at most one SHTP transfer. Pass the host timestamp captured when the
  // interrupt was first observed; it is preserved as the reception timestamp.
  ShtpResult service(uint32_t tEdgeUs);

  // Sends one payload, fragmenting only if it exceeds one transfer.
  // Never retries: returns false if the bus or the device refused the write.
  bool send(uint8_t channel, const uint8_t* payload, uint16_t len);

  const ShtpStats& stats() const { return stats_; }

 private:
  // Diagnostics for one transfer. They are recorded while CS is asserted and
  // emitted by logRead() only after CS has been released, so no serial or other
  // slow work can stretch the chip-select window.
  enum class ReadIssue : uint8_t {
    None = 0,
    StartFailed,
    HeaderFailed,
    BodyFailed,
    Spurious,
    ReservedLength,
    ShortFragment,
    BadChannel,
    UnexpectedContinuation,
    InterruptedPayload,
    OversizeDrained,
    PayloadTooLarge,
  };

  ShtpResult readTransfer(uint32_t tEdgeUs);
  bool drainWithinTransfer(uint32_t totalLength);
  void logRead(ShtpResult result);

  IHal* hal_;
  Logger* log_;
  uint8_t sensorId_;

  ChannelHandler handler_;
  void* handlerContext_;

  uint8_t rxTransfer_[BNO085_RX_TRANSFER_BYTES];
  uint8_t rxPayload_[BNO085_RX_PAYLOAD_BYTES];
  uint8_t txTransfer_[BNO085_TX_TRANSFER_BYTES];

  uint16_t inRemaining_;
  uint16_t inCursor_;
  uint8_t inChannel_;
  uint32_t inTimestampUs_;

  uint8_t nextInSeq_[shtp::kMaxChannels];
  uint8_t nextOutSeq_[shtp::kMaxChannels];
  bool channelSeen_[shtp::kMaxChannels];

  ReadIssue issue_;
  bool haveHeader_;
  uint16_t hdrLength_;
  uint8_t hdrChannel_;
  uint8_t hdrSequence_;
  bool hdrContinuation_;
  bool gapSeen_;
  uint8_t gapExpected_;
  uint8_t gapActual_;
  uint8_t gapChannel_;
  uint32_t extraValue_;

  // Payload is handed to the channel handler only after CS has been released.
  bool deliver_;
  uint16_t assembledLength_;
  uint8_t assembledChannel_;
  uint32_t assembledTimestampUs_;

  ShtpStats stats_;
};

} // namespace bno085

#endif // BNO085_SHTP_TRANSPORT_H
