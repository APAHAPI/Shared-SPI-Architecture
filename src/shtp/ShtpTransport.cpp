#include "shtp/ShtpTransport.h"

#include <cstring>

#include "diag/Logger.h"

namespace bno085 {

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
      nextInSeq_(), nextOutSeq_(), channelSeen_(), issue_(ReadIssue::None),
      haveHeader_(false), hdrLength_(0), hdrChannel_(0), hdrSequence_(0),
      hdrContinuation_(false), gapSeen_(false), gapExpected_(0), gapActual_(0),
      gapChannel_(0), extraValue_(0), deliver_(false), assembledLength_(0),
      assembledChannel_(0), assembledTimestampUs_(0), stats_() {}

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
  issue_ = ReadIssue::None;
  deliver_ = false;
}

ShtpResult ShtpTransport::service(uint32_t tEdgeUs) {
  if (hal_ == nullptr) return ShtpResult::BusError;
  if (!hal_->intAsserted()) return ShtpResult::NoData;

  // A BNO085 re-presents (and drops) a pending transfer when CS is released, so
  // the 4-byte header and every payload byte must be clocked inside ONE CS
  // assertion. readTransfer() opens that assertion and does nothing but SPI;
  // endRead() closes it on every exit path, including bus failures, and is
  // idempotent. Logging and payload delivery happen only after CS is released.
  const ShtpResult result = readTransfer(tEdgeUs);
  hal_->endRead();

  logRead(result);

  if (deliver_) {
    deliver_ = false;
    if (handler_ != nullptr) {
      handler_(handlerContext_, assembledChannel_, rxPayload_, assembledLength_,
               assembledTimestampUs_);
    }
  }
  return result;
}

// Clocks and discards the rest of a transfer whose 4-byte header has already
// been read. Stays inside the caller's CS assertion.
bool ShtpTransport::drainWithinTransfer(uint32_t totalLength) {
  if (totalLength <= shtp::kHeaderLen) return true;

  uint32_t remaining = totalLength - shtp::kHeaderLen;
  // Bounded: a rogue length field cannot spin here.
  const uint32_t limit = 8u * static_cast<uint32_t>(sizeof(rxTransfer_));
  if (remaining > limit) remaining = limit;

  while (remaining > 0) {
    const size_t chunk = (remaining < sizeof(rxTransfer_))
                             ? static_cast<size_t>(remaining)
                             : sizeof(rxTransfer_);
    if (!hal_->read(rxTransfer_, chunk)) return false;
    remaining -= static_cast<uint32_t>(chunk);
  }
  return true;
}

// SPI only: no logging, no payload delivery and no other work may happen between
// beginRead() and the matching endRead() in service(), because CS stays asserted
// for the whole time.
ShtpResult ShtpTransport::readTransfer(uint32_t tEdgeUs) {
  issue_ = ReadIssue::None;
  haveHeader_ = false;
  gapSeen_ = false;
  deliver_ = false;

  if (!hal_->beginRead()) {
    issue_ = ReadIssue::StartFailed;
    ++stats_.rxErrors;
    return ShtpResult::BusError;
  }

  // Header first, still inside the same CS assertion.
  if (!hal_->read(rxTransfer_, shtp::kHeaderLen)) {
    issue_ = ReadIssue::HeaderFailed;
    ++stats_.rxErrors;
    return ShtpResult::BusError;
  }

  const uint16_t rawLength = static_cast<uint16_t>(
      static_cast<uint16_t>(rxTransfer_[0]) |
      static_cast<uint16_t>(static_cast<uint16_t>(rxTransfer_[1]) << 8));
  const bool continuation = (rawLength & shtp::kContinuationBit) != 0;
  const uint16_t length = static_cast<uint16_t>(rawLength & shtp::kLengthMask);
  const uint8_t channel = rxTransfer_[2];
  const uint8_t sequence = rxTransfer_[3];

  haveHeader_ = true;
  hdrLength_ = length;
  hdrChannel_ = channel;
  hdrSequence_ = sequence;
  hdrContinuation_ = continuation;

  if (length == 0) {
    issue_ = ReadIssue::Spurious;
    ++stats_.spurious;
    return ShtpResult::Spurious;
  }

  if (rawLength == shtp::kReservedLength) {
    issue_ = ReadIssue::ReservedLength;
    ++stats_.rxErrors;
    ++stats_.badFragments;
    if (!drainWithinTransfer(length)) {
      issue_ = ReadIssue::BodyFailed;
      return ShtpResult::BusError;
    }
    return ShtpResult::Malformed;
  }

  if (length < shtp::kHeaderLen) {
    // The header is already on the wire, so the over-read is bounded by the
    // header size. A declared length below the header is malformed either way.
    issue_ = ReadIssue::ShortFragment;
    ++stats_.shortFragments;
    return ShtpResult::Malformed;
  }

  if (channel >= shtp::kMaxChannels) {
    issue_ = ReadIssue::BadChannel;
    ++stats_.badChannels;
    if (!drainWithinTransfer(length)) {
      issue_ = ReadIssue::BodyFailed;
      ++stats_.rxErrors;
      return ShtpResult::BusError;
    }
    return ShtpResult::Malformed;
  }

  // Sequence tracking is diagnostic only: a gap is counted, never repaired.
  // It runs before the oversize check so a drained transfer does not make the
  // following one look like a gap.
  if (channelSeen_[channel] && sequence != nextInSeq_[channel]) {
    ++stats_.sequenceGaps;
    gapSeen_ = true;
    gapChannel_ = channel;
    gapExpected_ = nextInSeq_[channel];
    gapActual_ = sequence;
  }
  channelSeen_[channel] = true;
  nextInSeq_[channel] = static_cast<uint8_t>(sequence + 1);

  if (length > sizeof(rxTransfer_)) {
    // Legal but unbufferable transfer: counted, then the remainder of THIS
    // transfer is drained inside the same CS assertion, so no tail is stranded
    // and CS is released exactly once. Not an rx error - the transfer behaved
    // exactly as the protocol allows, and an error flag here would make a
    // healthy boot indistinguishable from a desync.
    //
    // Only this transfer is drained. A following continuation fragment is a
    // separate device presentation with its own CS assertion, so it is left for
    // the next service() call rather than clocked with CS still low.
    issue_ = ReadIssue::OversizeDrained;
    ++stats_.oversizePayloads;

    if (!drainWithinTransfer(length)) {
      issue_ = ReadIssue::BodyFailed;
      ++stats_.rxErrors;
      return ShtpResult::BusError;
    }
    return ShtpResult::Oversize;
  }

  const uint16_t payloadLen = static_cast<uint16_t>(length - shtp::kHeaderLen);

  // Payload bytes, still inside the same CS assertion. `length` counts the
  // header, and the header is already consumed, so exactly payloadLen remain.
  if (payloadLen > 0) {
    if (!hal_->read(rxTransfer_ + shtp::kHeaderLen, payloadLen)) {
      issue_ = ReadIssue::BodyFailed;
      ++stats_.rxErrors;
      return ShtpResult::BusError;
    }
  }

  ++stats_.packets;
  stats_.bytes += length;
  stats_.lastChannel = channel;
  stats_.lastLength = length;
  stats_.lastTimestampUs = tEdgeUs;

  if (continuation) {
    // The sender sets the continuation bit on every fragment after the first, so
    // a continuation arriving with no assembly in progress is out of sequence.
    if (inRemaining_ == 0 || channel != inChannel_ || payloadLen > inRemaining_) {
      issue_ = ReadIssue::UnexpectedContinuation;
      ++stats_.badFragments;
      inRemaining_ = 0;
      inCursor_ = 0;
      return ShtpResult::Malformed;
    }
  } else {
    if (inRemaining_ != 0) {
      // A new payload started while an assembly was in progress.
      issue_ = ReadIssue::InterruptedPayload;
      extraValue_ = inRemaining_;
      ++stats_.badFragments;
    }
    inCursor_ = 0;
    inChannel_ = channel;
    inRemaining_ = payloadLen;
    inTimestampUs_ = tEdgeUs;
  }

  if (payloadLen > 0) {
    const size_t space = static_cast<size_t>(sizeof(rxPayload_) - inCursor_);
    if (payloadLen > space) {
      issue_ = ReadIssue::PayloadTooLarge;
      extraValue_ = static_cast<uint32_t>(space);
      ++stats_.oversizePayloads;
      inRemaining_ = 0;
      inCursor_ = 0;
      return ShtpResult::Oversize;
    }
    std::memcpy(rxPayload_ + inCursor_, rxTransfer_ + shtp::kHeaderLen, payloadLen);
    inCursor_ = static_cast<uint16_t>(inCursor_ + payloadLen);
    inRemaining_ = static_cast<uint16_t>(inRemaining_ - payloadLen);
  }

  if (inRemaining_ != 0) {
    return ShtpResult::Fragment;
  }

  assembledLength_ = inCursor_;
  assembledChannel_ = inChannel_;
  assembledTimestampUs_ = inTimestampUs_;
  inCursor_ = 0;
  deliver_ = true;
  return ShtpResult::Ok;
}

// All diagnostics for one service() call, emitted after CS has been released.
void ShtpTransport::logRead(ShtpResult result) {
  if (log_ == nullptr) return;

  if (gapSeen_) {
    gapSeen_ = false;
    log_->log(LogLevel::Warn, LogShtp, "s%u ch%u seq gap: expected %u got %u",
              sensorId_, static_cast<unsigned>(gapChannel_),
              static_cast<unsigned>(gapExpected_),
              static_cast<unsigned>(gapActual_));
  }

  switch (issue_) {
    case ReadIssue::StartFailed:
      log_->log(LogLevel::Error, LogSpi, "s%u read start failed", sensorId_);
      return;
    case ReadIssue::HeaderFailed:
      log_->log(LogLevel::Error, LogSpi, "s%u header read failed", sensorId_);
      return;
    case ReadIssue::BodyFailed:
      log_->log(LogLevel::Error, LogSpi, "s%u body read failed", sensorId_);
      return;
    case ReadIssue::Spurious:
      log_->log(LogLevel::Debug, LogInt, "s%u spurious INT (len 0)", sensorId_);
      return;
    case ReadIssue::ReservedLength:
      log_->log(LogLevel::Warn, LogShtp, "s%u reserved length 0xFFFF", sensorId_);
      return;
    case ReadIssue::ShortFragment:
      log_->log(LogLevel::Warn, LogShtp, "s%u short fragment (%u)", sensorId_,
                static_cast<unsigned>(hdrLength_));
      return;
    case ReadIssue::BadChannel:
      log_->log(LogLevel::Warn, LogShtp, "s%u bad channel %u", sensorId_,
                static_cast<unsigned>(hdrChannel_));
      return;
    case ReadIssue::UnexpectedContinuation:
      log_->log(LogLevel::Warn, LogShtp,
                "s%u unexpected continuation (ch=%u, %u bytes)", sensorId_,
                static_cast<unsigned>(hdrChannel_),
                static_cast<unsigned>(hdrLength_ - shtp::kHeaderLen));
      return;
    case ReadIssue::InterruptedPayload:
      log_->log(LogLevel::Warn, LogShtp, "s%u interrupted payload (%u bytes lost)",
                sensorId_, static_cast<unsigned>(extraValue_));
      return;
    case ReadIssue::OversizeDrained:
      log_->log(LogLevel::Warn, LogShtp, "s%u oversize transfer %u > %u, drained",
                sensorId_, static_cast<unsigned>(hdrLength_),
                static_cast<unsigned>(sizeof(rxTransfer_)));
      return;
    case ReadIssue::PayloadTooLarge:
      log_->log(LogLevel::Error, LogShtp, "s%u payload exceeds buffer space %u",
                sensorId_, static_cast<unsigned>(extraValue_));
      return;
    case ReadIssue::None:
      break;
  }

  if (!haveHeader_) return;

  log_->log(LogLevel::Trace, LogShtp, "s%u hdr len=%u cont=%u ch=%u(%s) seq=%u",
            sensorId_, static_cast<unsigned>(hdrLength_),
            hdrContinuation_ ? 1u : 0u, static_cast<unsigned>(hdrChannel_),
            shtp::channelName(hdrChannel_),
            static_cast<unsigned>(hdrSequence_));

  if (result == ShtpResult::Ok || result == ShtpResult::Fragment) {
    log_->hex(LogLevel::Trace, LogShtp, "shtp-rx", rxTransfer_, hdrLength_);
  }
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
