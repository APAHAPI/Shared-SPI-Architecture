#include "output/BinaryEncoder.h"

#include <cstring>

#include "diag/Logger.h"
#include "util/Crc16.h"

namespace bno085 {

BinaryEncoder::BinaryEncoder()
    : sink_(nullptr), log_(nullptr), sequence_(0), frames_(0), dropped_(0),
      frame_() {}

void BinaryEncoder::attach(IOutputSink* sink, Logger* log) {
  sink_ = sink;
  log_ = log;
}

void BinaryEncoder::reset() {
  sequence_ = 0;
  frames_ = 0;
  dropped_ = 0;
}

bool BinaryEncoder::emit(protocol::PacketType type, const uint8_t* payload,
                         uint16_t len) {
  if (sink_ == nullptr) {
    ++dropped_;
    return false;
  }
  if (len > protocol::kMaxPayloadBytes) {
    ++dropped_;
    if (log_) log_->log(LogLevel::Error, LogData, "frame payload %u too large",
                        static_cast<unsigned>(len));
    return false;
  }

  frame_[0] = protocol::kSync0;
  frame_[1] = protocol::kSync1;
  frame_[2] = protocol::kVersion;
  frame_[3] = static_cast<uint8_t>(type);
  protocol::putU16(frame_ + 4, len);
  protocol::putU16(frame_ + protocol::kSequenceOffset, sequence_++);
  if (len > 0 && payload != nullptr) {
    std::memcpy(frame_ + protocol::kHeaderBytes, payload, len);
  }

  const size_t covered = 6u + len;  // version .. last payload byte
  const uint16_t crc = crc16CcittFalse(frame_ + 2, covered);
  protocol::putU16(frame_ + protocol::kHeaderBytes + len, crc);

  const size_t total = protocol::kHeaderBytes + len + 2u;
  const size_t written = sink_->write(frame_, total);
  if (written != total) {
    ++dropped_;
    if (log_) {
      log_->log(LogLevel::Warn, LogData, "short output write %u/%u",
                static_cast<unsigned>(written), static_cast<unsigned>(total));
    }
    return false;
  }

  ++frames_;
  return true;
}

bool BinaryEncoder::writeHello(uint8_t sensorCount, uint16_t libraryVersion,
                               uint32_t tBootUs) {
  uint8_t p[protocol::kHelloPayloadBytes] = {0};
  p[0] = protocol::kVersion;
  p[1] = sensorCount;
  // bit0 binary, bit3 reception timestamps present
  protocol::putU16(p + 2, 0x0009u);
  protocol::putU32(p + 4, tBootUs);
  protocol::putU16(p + 8, libraryVersion);
  return emit(protocol::PacketType::Hello, p, sizeof(p));
}

bool BinaryEncoder::writeQuaternion(uint8_t sensorId, const Quaternion& q) {
  uint8_t p[protocol::kQuaternionPayloadBytes] = {0};
  uint8_t flags = 0;
  if (q.source != Report::RotationVector) flags |= protocol::flag::SourceIsGrv;
  flags |= protocol::flag::AccuracyValid;
  p[0] = sensorId;
  p[1] = flags;
  p[2] = static_cast<uint8_t>(q.accuracy);
  p[3] = q.seq;
  p[4] = q.epoch;
  p[5] = 14;  // Q14
  protocol::putI16(p + 6, q.rawX);
  protocol::putI16(p + 8, q.rawY);
  protocol::putI16(p + 10, q.rawZ);
  protocol::putI16(p + 12, q.rawW);
  protocol::putI16(p + 14, q.rawAccuracy);
  protocol::putU32(p + 16, q.tMeasUs);
  protocol::putU32(p + 20, q.tHostUs);
  return emit(protocol::PacketType::Quaternion, p, sizeof(p));
}

bool BinaryEncoder::writeInitResult(uint8_t sensorId, uint8_t result,
                                    uint8_t stage, const ProductInfo& product,
                                    uint32_t grantedIntervalUs) {
  uint8_t p[protocol::kInitResultPayloadBytes] = {0};
  p[0] = sensorId;
  p[1] = result;
  p[2] = stage;
  p[3] = product.major;
  p[4] = product.minor;
  protocol::putU16(p + 5, product.patch);
  protocol::putU32(p + 7, product.partNumber);
  protocol::putU32(p + 11, grantedIntervalUs);
  p[15] = 0;
  return emit(protocol::PacketType::InitResult, p, sizeof(p));
}

bool BinaryEncoder::writeStatus(uint8_t sensorId, const Diagnostics& d) {
  uint8_t p[protocol::kStatusPayloadBytes] = {0};
  p[0] = sensorId;
  p[1] = static_cast<uint8_t>(d.state);
  p[2] = static_cast<uint8_t>(d.lastAccuracy);
  p[3] = static_cast<uint8_t>(d.lastError);
  protocol::putU32(p + 4, d.missedReports);
  protocol::putU32(p + 8, d.reportsDecoded);
  protocol::putU16(p + 12, d.sequenceGaps);
  protocol::putU16(p + 14, d.duplicateReports);
  protocol::putU16(p + 16, static_cast<uint16_t>(d.spiErrors));
  protocol::putU16(p + 18, static_cast<uint16_t>(d.resetPulses));
  protocol::putU32(p + 20, d.lastReportUs);
  return emit(protocol::PacketType::SensorStatus, p, sizeof(p));
}

bool BinaryEncoder::writeError(uint8_t sensorId, ErrorCode code,
                               uint8_t context, uint32_t tUs) {
  uint8_t p[protocol::kErrorPayloadBytes] = {0};
  p[0] = sensorId;
  p[1] = static_cast<uint8_t>(code);
  p[2] = context;
  protocol::putU32(p + 4, tUs);
  return emit(protocol::PacketType::Error, p, sizeof(p));
}

bool BinaryEncoder::writeManagerDiagnostics(uint8_t sensorCount,
                                            uint8_t streamingCount,
                                            uint16_t busBusy,
                                            uint32_t queueDrops,
                                            uint32_t uptimeMs,
                                            uint32_t maxUpdateUs) {
  uint8_t p[protocol::kManagerDiagPayloadBytes] = {0};
  p[0] = sensorCount;
  p[1] = streamingCount;
  protocol::putU16(p + 2, busBusy);
  protocol::putU32(p + 4, frames_);
  protocol::putU32(p + 8, queueDrops);
  protocol::putU32(p + 12, uptimeMs);
  protocol::putU32(p + 16, maxUpdateUs);
  return emit(protocol::PacketType::ManagerDiagnostics, p, sizeof(p));
}

} // namespace bno085
