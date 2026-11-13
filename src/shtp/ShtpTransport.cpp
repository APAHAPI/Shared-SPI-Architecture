#include "shtp/ShtpTransport.h"

#include <cstring>

#include "diag/Logger.h"

namespace bno085 {

namespace {
// MOSI is ignored during reads; a shared read-only filler avoids a per-instance
// scratch buffer and keeps every transfer byte-deterministic.
const uint8_t kZeroFill[BNO085_RX_TRANSFER_BYTES] = {0};

size_t minSize(size_t a, size_t b) { return (a < b) ? a : b; }
} // namespace

const char* shtp::channelName(uint8_t channel) {
  switch (channel) {
    case shtp::kChannelCommand:     return "command";
    case shtp::kChannelExecutable:  return "executable";
    case shtp::kChannelControl:     return "control";
    case shtp::kChannelReports:     return "reports";
    case shtp::kChannelWakeReports: return "wake-reports";
    case shtp::kChannelGyroRv:      return "gyro-rv";
    default:                        return "unknown";
  }
}

ShtpTransport::ShtpTransport()
    : hal_(nullptr), log_(nullptr), sensorId_(0), handler_(nullptr),
      handlerContext_(nullptr), rxTransfer_(), rxPayload_(), txTransfer_(),
      inRemaining_(0), inCursor_(0), inChannel_(0), inTimestampUs_(0),
      nextInSeq_(), nextOutSeq_(), channelSeen_(), stats_() {}

void ShtpTransport::attach(IHal* hal, Logger* log, uint8_t sensorId) {
  hal_ = hal;
  log_ = log;
  sensorId_ = sensorId;
  reset();
}

void ShtpTransport::setHandler(ChannelHandler handler, void* context) {
  handler_ = handler;
  handlerContext_ = context;
}

void ShtpTransport::reset() {
  inRemaining_ = 0;
  inCursor_ = 0;
  inChannel_ = 0;
  inTimestampUs_ = 0;
  for (uint8_t i = 0; i < shtp::kMaxChannels; ++i) {
    nextInSeq_[i] = 0;
    nextOutSeq_[i] = 0;
    channelSeen_[i] = false;
  }
}

bool ShtpTransport::readRaw(uint8_t* dst, size_t len) {
  if (hal_ == nullptr || len == 0) return false;
  return hal_->transfer(kZeroFill, dst, len);
}

// Reads and throws away `len` bytes so a rejected transfer does not leave the
// device asserting INT forever. Bounded to a few transfers.
bool ShtpTransport::discard(size_t len) {
  const size_t limit = 4 * sizeof(rxTransfer_);
  size_t remaining = minSize(len, limit);
  while (remaining > 0) {
    const size_t chunk = minSize(remaining, sizeof(rxTransfer_));
    if (!readRaw(rxTransfer_, chunk)) return false;
    remaining -= chunk;
  }
  return true;
}

ShtpResult ShtpTransport::service(uint32_t tEdgeUs) {
  if (hal_ == nullptr) return ShtpResult::BusError;
  if (!hal_->intAsserted()) return ShtpResult::NoData;

  uint8_t peek[shtp::kHeaderLen];
  if (!readRaw(peek, sizeof(peek))) {
    ++stats_.rxErrors;
    if (log_) log_->log(LogLevel::Error, LogSpi, "s%u peek transfer failed", sensorId_);
    return ShtpResult::BusError;
  }

  const uint16_t rawLength = static_cast<uint16_t>(
      static_cast<uint16_t>(peek[0]) |
      static_cast<uint16_t>(static_cast<uint16_t>(peek[1]) << 8));
  const bool continuation = (rawLength & shtp::kContinuationBit) != 0;
  const uint16_t length = static_cast<uint16_t>(rawLength & shtp::kLengthMask);
  const uint8_t channel = peek[2];
  const uint8_t sequence = peek[3];

  if (log_) {
    log_->log(LogLevel::Trace, LogShtp,
              "s%u hdr len=%u cont=%u ch=%u(%s) seq=%u", sensorId_,
              static_cast<unsigned>(length), continuation ? 1u : 0u,
              static_cast<unsigned>(channel), shtp::channelName(channel),
              static_cast<unsigned>(sequence));
  }

  if (length == 0) {
    ++stats_.spurious;
    if (log_) log_->log(LogLevel::Debug, LogInt, "s%u spurious INT (len 0)", sensorId_);
    return ShtpResult::Spurious;
  }

  if (rawLength == shtp::kReservedLength) {
    ++stats_.rxErrors;
    ++stats_.badFragments;
    discard(length);
    if (log_) log_->log(LogLevel::Warn, LogShtp, "s%u reserved length 0xFFFF", sensorId_);
    return ShtpResult::Malformed;
  }

  if (length < shtp::kHeaderLen) {
    ++stats_.shortFragments;
    discard(length);
    if (log_) log_->log(LogLevel::Warn, LogShtp, "s%u short fragment (%u)", sensorId_,
                        static_cast<unsigned>(length));
    return ShtpResult::Malformed;
  }

  if (channel >= shtp::kMaxChannels) {
    ++stats_.badChannels;
    discard(length);
    if (log_) log_->log(LogLevel::Warn, LogShtp, "s%u bad channel %u", sensorId_,
                        static_cast<unsigned>(channel));
    return ShtpResult::Malformed;
  }

  // Sequence tracking is diagnostic only: a gap is counted, never repaired.
  if (channelSeen_[channel] && sequence != nextInSeq_[channel]) {
    ++stats_.sequenceGaps;
    if (log_) log_->log(LogLevel::Warn, LogShtp,
                        "s%u ch%u seq gap: expected %u got %u", sensorId_,
                        static_cast<unsigned>(channel),
                        static_cast<unsigned>(nextInSeq_[channel]),
                        static_cast<unsigned>(sequence));
  }
  channelSeen_[channel] = true;
  nextInSeq_[channel] = static_cast<uint8_t>(sequence + 1);

  if (length > sizeof(rxTransfer_)) {
    // Legal partial read: the device re-presents the remainder with a new,
    // shorter length on the next transfer.
    ++stats_.oversizePayloads;
    ++stats_.rxErrors;
    if (!readRaw(rxTransfer_, sizeof(rxTransfer_))) return ShtpResult::BusError;
    if (log_) log_->log(LogLevel::Warn, LogShtp,
                        "s%u oversize transfer %u > %u, partial read", sensorId_,
                        static_cast<unsigned>(length),
                        static_cast<unsigned>(sizeof(rxTransfer_)));
    return ShtpResult::Oversize;
  }

  if (!readRaw(rxTransfer_, length)) {
    ++stats_.rxErrors;
    if (log_) log_->log(LogLevel::Error, LogSpi, "s%u body transfer failed", sensorId_);
    return ShtpResult::BusError;
  }

  ++stats_.packets;
  stats_.bytes += length;
  stats_.lastChannel = channel;
  stats_.lastLength = length;
  stats_.lastTimestampUs = tEdgeUs;

  if (log_) log_->hex(LogLevel::Trace, LogShtp, "shtp-rx", rxTransfer_, length);

  const uint16_t payloadLen = static_cast<uint16_t>(length - shtp::kHeaderLen);

  if (continuation) {
    if (inRemaining_ == 0 || channel != inChannel_ || payloadLen > inRemaining_) {
      ++stats_.badFragments;
      inRemaining_ = 0;
      inCursor_ = 0;
      if (log_) log_->log(LogLevel::Warn, LogShtp,
                          "s%u unexpected continuation (ch=%u, %u bytes)", sensorId_,
                          static_cast<unsigned>(channel),
                          static_cast<unsigned>(payloadLen));
      return ShtpResult::Malformed;
    }
  } else {
    if (inRemaining_ != 0) {
      // A new payload started while an assembly was in progress.
      ++stats_.badFragments;
      if (log_) log_->log(LogLevel::Warn, LogShtp,
                          "s%u interrupted payload (%u bytes lost)", sensorId_,
                          static_cast<unsigned>(inRemaining_));
    }
    inCursor_ = 0;
    inChannel_ = channel;
    inRemaining_ = payloadLen;
    inTimestampUs_ = tEdgeUs;
  }

  if (payloadLen > 0) {
    const size_t space = static_cast<size_t>(sizeof(rxPayload_) - inCursor_);
    if (payloadLen > space) {
      ++stats_.oversizePayloads;
      inRemaining_ = 0;
      inCursor_ = 0;
      if (log_) log_->log(LogLevel::Error, LogShtp,
                          "s%u payload %u exceeds buffer space %u", sensorId_,
                          static_cast<unsigned>(payloadLen),
                          static_cast<unsigned>(space));
      return ShtpResult::Oversize;
    }
    std::memcpy(rxPayload_ + inCursor_, rxTransfer_ + shtp::kHeaderLen, payloadLen);
    inCursor_ = static_cast<uint16_t>(inCursor_ + payloadLen);
    inRemaining_ = static_cast<uint16_t>(inRemaining_ - payloadLen);
  }

  if (inRemaining_ != 0) {
    return ShtpResult::Fragment;
  }

  const uint16_t assembledLen = inCursor_;
  const uint8_t assembledChannel = inChannel_;
  const uint32_t assembledTime = inTimestampUs_;
  inCursor_ = 0;

  if (handler_ != nullptr) {
    handler_(handlerContext_, assembledChannel, rxPayload_, assembledLen,
             assembledTime);
  }
  return ShtpResult::Ok;
}

bool ShtpTransport::send(uint8_t channel, const uint8_t* payload, uint16_t len) {
  if (hal_ == nullptr) return false;
  if (channel == shtp::kChannelCommand || channel >= shtp::kMaxChannels) {
    ++stats_.txErrors;
    if (log_) log_->log(LogLevel::Error, LogShtp, "s%u invalid tx channel %u",
                        sensorId_, static_cast<unsigned>(channel));
    return false;
  }
  if (len > shtp::kMaxPayloadPerPacket) {
    ++stats_.txErrors;
    return false;
  }
  if (!hal_->prepareWrite()) {
    ++stats_.txErrors;
    if (log_) log_->log(LogLevel::Debug, LogSpi, "s%u device not ready to write", sensorId_);
    return false;
  }

  const uint16_t chunkSize = static_cast<uint16_t>(BNO085_TX_TRANSFER_BYTES - 4);
  uint16_t cursor = 0;
  uint16_t remaining = len;
  bool continuation = false;

  do {
    const uint16_t chunk = static_cast<uint16_t>(
        (remaining < chunkSize) ? remaining : chunkSize);
    const uint16_t total = static_cast<uint16_t>(chunk + 4);

    txTransfer_[0] = static_cast<uint8_t>(total & 0xFF);
    txTransfer_[1] = static_cast<uint8_t>((total >> 8) & 0x7F);
    if (continuation) txTransfer_[1] |= 0x80;
    txTransfer_[2] = channel;
    txTransfer_[3] = nextOutSeq_[channel]++;
    if (chunk > 0) {
      std::memcpy(txTransfer_ + 4, payload + cursor, chunk);
    }

    if (log_) log_->hex(LogLevel::Trace, LogShtp, "shtp-tx", txTransfer_, total);

    if (!hal_->transfer(txTransfer_, nullptr, total)) {
      ++stats_.txErrors;
      if (log_) log_->log(LogLevel::Error, LogSpi, "s%u tx transfer failed", sensorId_);
      return false;
    }

    ++stats_.txPackets;
    cursor = static_cast<uint16_t>(cursor + chunk);
    remaining = static_cast<uint16_t>(remaining - chunk);
    continuation = true;
  } while (remaining > 0);

  return true;
}

} // namespace bno085
