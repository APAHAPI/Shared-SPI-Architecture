#include "sh2/Sh2Client.h"

#include <climits>

#include "diag/Logger.h"
#include "sh2/Sh2Codes.h"

namespace bno085 {

namespace {
// Datasheet 1000-3927 section 1.3.5: the host timestamps at H_INTN assertion,
// the timebase reference is 120 * 100 us, and each report adds its own delay.
constexpr int32_t kTimebaseReferenceUs = -12000;
constexpr float kQ14Scale = 1.0f / 16384.0f;
constexpr int32_t kRebaseUnitUs = 100;
} // namespace

Sh2Client::Sh2Client()
    : transport_(nullptr), log_(nullptr), sensorId_(0), handlers_(),
      timeoutMs_(100), product_(), prodEntries_(0), prodExpected_(4),
      prodPending_(false), prodDeadlineUs_(0), cmdPending_(false), cmdId_(0),
      cmdSeq_(0), nextCmdSeq_(0), cmdDeadlineUs_(0), cmdState_(CmdState::Idle),
      featurePending_(false), featureId_(0), featureDeadlineUs_(0),
      featureState_(CmdState::Idle), grantedIntervalUs_(0),
      rebaseOffsetUs_(0), epoch_(0), haveQuaternion_(false), lastQuatSeq_(0),
      lastQuatTMeasUs_(0), stats_(), lastError_(ErrorCode::None) {}

void Sh2Client::attach(ShtpTransport* transport, Logger* log, uint8_t sensorId) {
  transport_ = transport;
  log_ = log;
  sensorId_ = sensorId;
  reset();
}

void Sh2Client::reset() {
  prodPending_ = false;
  prodEntries_ = 0;
  prodExpected_ = 4;
  product_ = ProductInfo();
  cmdPending_ = false;
  cmdState_ = CmdState::Idle;
  featurePending_ = false;
  featureState_ = CmdState::Idle;
  grantedIntervalUs_ = 0;
  haveQuaternion_ = false;
  lastQuatSeq_ = 0;
  lastQuatTMeasUs_ = 0;
  lastError_ = ErrorCode::None;
}

void Sh2Client::handlePayload(uint8_t channel, const uint8_t* payload,
                              uint16_t len, uint32_t tUs) {
  switch (channel) {
    case shtp::kChannelCommand:
      if (log_) log_->hex(LogLevel::Debug, LogShtp, "advertisement", payload, len);
      break;
    case shtp::kChannelExecutable:
      handleExecutable(payload, len);
      break;
    case shtp::kChannelControl:
      handleControl(payload, len, tUs);
      break;
    case shtp::kChannelReports:
    case shtp::kChannelWakeReports:
      handleReports(payload, len, tUs);
      break;
    case shtp::kChannelGyroRv:
      if (log_) log_->log(LogLevel::Debug, LogSh2, "s%u gyro-RV channel ignored", sensorId_);
      break;
    default:
      break;
  }
}

void Sh2Client::handleExecutable(const uint8_t* payload, uint16_t len) {
  if (len < 1) return;
  if (payload[0] == sh2::kExecResetComplete) {
    ++epoch_;
    if (log_) {
      log_->log(LogLevel::Info, LogReset, "s%u reset complete (epoch %u)", sensorId_,
                static_cast<unsigned>(epoch_));
    }
    // Any in-flight request is meaningless across a reset.
    if (cmdPending_) {
      cmdPending_ = false;
      cmdState_ = CmdState::Timeout;
    }
    if (featurePending_) {
      featurePending_ = false;
      featureState_ = CmdState::Timeout;
    }
    if (handlers_.resetComplete != nullptr) {
      handlers_.resetComplete(handlers_.context);
    }
  }
}

void Sh2Client::handleControl(const uint8_t* payload, uint16_t len, uint32_t tUs) {
  (void)tUs;
  uint16_t cursor = 0;
  while (cursor < len) {
    const uint8_t id = payload[cursor];
    const uint8_t reportLen = sh2::reportLength(id);
    if (reportLen == 0 || static_cast<uint16_t>(cursor + reportLen) > len) {
      ++stats_.unknownReports;
      if (log_) {
        log_->log(LogLevel::Warn, LogSh2, "s%u unknown control report 0x%02X at %u/%u",
                  sensorId_, static_cast<unsigned>(id),
                  static_cast<unsigned>(cursor), static_cast<unsigned>(len));
      }
      break;
    }

    switch (id) {
      case sh2::kProdIdResp:
        handleProdId(payload + cursor);
        break;
      case sh2::kCommandResp:
        handleCommandResponse(payload + cursor);
        break;
      case sh2::kGetFeatureResp:
        handleGetFeatureResponse(payload + cursor);
        break;
      case sh2::kTimestampRebase:
        handleTimestampRebase(payload + cursor);
        break;
      case sh2::kBaseTimestampRef:
        ++stats_.baseTimestampRefs;
        if (log_) {
          log_->log(LogLevel::Debug, LogSh2, "s%u base timestamp ref = %u",
                    sensorId_, static_cast<unsigned>(sh2::readU32(payload + cursor + 1)));
        }
        break;
      default:
        if (log_) {
          log_->log(LogLevel::Debug, LogSh2, "s%u control report 0x%02X (%u bytes)",
                    sensorId_, static_cast<unsigned>(id),
                    static_cast<unsigned>(reportLen));
        }
        break;
    }

    cursor = static_cast<uint16_t>(cursor + reportLen);
  }
}

void Sh2Client::handleProdId(const uint8_t* report) {
  if (prodEntries_ == 0) {
    product_.resetCause = report[1];
    product_.major = report[2];
    product_.minor = report[3];
    product_.partNumber = sh2::readU32(report + 4);
    product_.buildNumber = sh2::readU32(report + 8);
    product_.patch = sh2::readU16(report + 12);
    product_.valid = true;

    // FSP200-class parts report five product IDs instead of four.
    if (product_.partNumber == 10004095u || product_.partNumber == 10004818u ||
        product_.partNumber == 10005028u) {
      prodExpected_ = 5;
    }
    if (log_) {
      log_->log(LogLevel::Info, LogSh2,
                "s%u product id part=%lu build=%lu fw=%u.%u.%u resetCause=%u",
                sensorId_, static_cast<unsigned long>(product_.partNumber),
                static_cast<unsigned long>(product_.buildNumber),
                static_cast<unsigned>(product_.major),
                static_cast<unsigned>(product_.minor),
                static_cast<unsigned>(product_.patch),
                static_cast<unsigned>(product_.resetCause));
    }
  }

  if (prodEntries_ < 0xFF) ++prodEntries_;
  product_.entries = prodEntries_;

  if (prodPending_ && prodEntries_ >= prodExpected_) {
    prodPending_ = false;
    if (log_) {
      log_->log(LogLevel::Debug, LogSh2, "s%u product id complete (%u entries)",
                sensorId_, static_cast<unsigned>(prodEntries_));
    }
  }
}

void Sh2Client::handleCommandResponse(const uint8_t* report) {
  const uint8_t command = report[2];
  const uint8_t commandSeq = report[3];
  const uint8_t hubStatus = report[5];

  if (!cmdPending_) {
    ++stats_.unexpectedResponses;
    if (log_) {
      log_->log(LogLevel::Debug, LogSh2, "s%u unsolicited command response 0x%02X",
                sensorId_, static_cast<unsigned>(command));
    }
    return;
  }

  if (command != cmdId_) {
    ++stats_.badResponses;
    if (log_) {
      log_->log(LogLevel::Warn, LogSh2,
                "s%u command response mismatch: expected 0x%02X got 0x%02X",
                sensorId_, static_cast<unsigned>(cmdId_),
                static_cast<unsigned>(command));
    }
    return;
  }

  if (commandSeq != cmdSeq_ && log_) {
    log_->log(LogLevel::Debug, LogSh2,
              "s%u command 0x%02X seq echo %u (sent %u)", sensorId_,
              static_cast<unsigned>(command), static_cast<unsigned>(commandSeq),
              static_cast<unsigned>(cmdSeq_));
  }

  cmdPending_ = false;
  if (hubStatus == 0) {
    cmdState_ = CmdState::Ok;
    if (log_) {
      log_->log(LogLevel::Info, LogSh2, "s%u command 0x%02X accepted", sensorId_,
                static_cast<unsigned>(command));
    }
  } else {
    cmdState_ = CmdState::HubError;
    lastError_ = ErrorCode::Sh2BadResponse;
    ++stats_.badResponses;
    if (log_) {
      log_->log(LogLevel::Warn, LogSh2, "s%u command 0x%02X rejected (hub status %u)",
                sensorId_, static_cast<unsigned>(command),
                static_cast<unsigned>(hubStatus));
    }
  }
}

void Sh2Client::handleGetFeatureResponse(const uint8_t* report) {
  if (!featurePending_ || report[1] != featureId_) {
    ++stats_.unexpectedResponses;
    if (log_) {
      log_->log(LogLevel::Debug, LogSh2,
                "s%u unsolicited get-feature response for 0x%02X", sensorId_,
                static_cast<unsigned>(report[1]));
    }
    return;
  }

  grantedIntervalUs_ = sh2::readU32(report + 5);
  featurePending_ = false;
  featureState_ = CmdState::Ok;
  if (log_) {
    log_->log(LogLevel::Info, LogSh2, "s%u report 0x%02X granted interval %lu us",
              sensorId_, static_cast<unsigned>(featureId_),
              static_cast<unsigned long>(grantedIntervalUs_));
  }
}

void Sh2Client::handleTimestampRebase(const uint8_t* report) {
  const int32_t ticks = static_cast<int32_t>(sh2::readU32(report + 1));
  const int64_t delta = static_cast<int64_t>(ticks) * kRebaseUnitUs;
  int64_t accumulated = static_cast<int64_t>(rebaseOffsetUs_) + delta;
  if (accumulated > INT32_MAX) accumulated = INT32_MAX;
  if (accumulated < INT32_MIN) accumulated = INT32_MIN;
  rebaseOffsetUs_ = static_cast<int32_t>(accumulated);
  ++epoch_;
  if (log_) {
    log_->log(LogLevel::Info, LogSh2,
              "s%u timestamp rebase %ld ticks -> offset %ld us (epoch %u)", sensorId_,
              static_cast<long>(ticks), static_cast<long>(rebaseOffsetUs_),
              static_cast<unsigned>(epoch_));
  }
}

void Sh2Client::handleReports(const uint8_t* payload, uint16_t len, uint32_t tUs) {
  uint16_t cursor = 0;
  while (cursor < len) {
    const uint8_t id = payload[cursor];
    const uint8_t reportLen = sh2::reportLength(id);
    if (reportLen == 0 || static_cast<uint16_t>(cursor + reportLen) > len) {
      ++stats_.unknownReports;
      if (log_) {
        log_->log(LogLevel::Warn, LogSh2,
                  "s%u cannot parse report 0x%02X at %u/%u (length %u)", sensorId_,
                  static_cast<unsigned>(id), static_cast<unsigned>(cursor),
                  static_cast<unsigned>(len), static_cast<unsigned>(reportLen));
      }
      break;
    }

    if (id == 0x05) {
      handleRotationVector(payload + cursor, reportLen, tUs);
    } else if (handlers_.sensorReport != nullptr) {
      handlers_.sensorReport(handlers_.context, id, payload + cursor, reportLen, tUs);
    }

    cursor = static_cast<uint16_t>(cursor + reportLen);
  }
}

void Sh2Client::handleRotationVector(const uint8_t* report, uint8_t length,
                                     uint32_t tUs) {
  sh2::RotationVector rv;
  if (!sh2::decodeRotationVector(report, length, rv)) {
    ++stats_.badResponses;
    lastError_ = ErrorCode::Sh2BadResponse;
    if (log_) {
      log_->log(LogLevel::Warn, LogSh2, "s%u malformed rotation vector (%u bytes)",
                sensorId_, static_cast<unsigned>(length));
    }
    return;
  }

  Quaternion q;
  q.rawX = rv.i;
  q.rawY = rv.j;
  q.rawZ = rv.k;
  q.rawW = rv.real;
  q.x = static_cast<float>(rv.i) * kQ14Scale;
  q.y = static_cast<float>(rv.j) * kQ14Scale;
  q.z = static_cast<float>(rv.k) * kQ14Scale;
  q.w = static_cast<float>(rv.real) * kQ14Scale;
  q.rawAccuracy = static_cast<int16_t>(rv.accuracyQ12);
  q.accuracy = accuracyFromStatus(rv.status);
  q.tMeasUs = computeMeasTime(tUs, rv.delayTicks);
  q.tHostUs = tUs;
  q.seq = rv.sequence;
  q.epoch = static_cast<uint8_t>(epoch_);
  q.source = Report::RotationVector;
  q.status = rv.status;
  q.delayTicks = rv.delayTicks;
  q.valid = true;

  // The BNO085 retries a report if the host misses its response window, so an
  // identical (sequence, measurement time) pair is a retransmission.
  if (haveQuaternion_ && rv.sequence == lastQuatSeq_ &&
      q.tMeasUs == lastQuatTMeasUs_) {
    ++stats_.duplicates;
    if (log_) {
      log_->log(LogLevel::Debug, LogData, "s%u duplicate report seq=%u ignored",
                sensorId_, static_cast<unsigned>(rv.sequence));
    }
    return;
  }

  if (haveQuaternion_) {
    const uint8_t expected = static_cast<uint8_t>(lastQuatSeq_ + 1);
    if (rv.sequence != expected) {
      const uint8_t gap = static_cast<uint8_t>(rv.sequence - expected);
      stats_.reportGaps = static_cast<uint32_t>(stats_.reportGaps + gap);
      if (log_) {
        log_->log(LogLevel::Warn, LogData, "s%u report gap: %u missing", sensorId_,
                  static_cast<unsigned>(gap));
      }
    }
  }

  haveQuaternion_ = true;
  lastQuatSeq_ = rv.sequence;
  lastQuatTMeasUs_ = q.tMeasUs;
  ++stats_.reports;

  if (log_) {
    log_->log(LogLevel::Debug, LogData,
              "s%u RV q=(%.4f,%.4f,%.4f,%.4f) acc=%s seq=%u delay=%u tMeas=%lu", sensorId_,
              static_cast<double>(q.w), static_cast<double>(q.x),
              static_cast<double>(q.y), static_cast<double>(q.z),
              accuracyName(q.accuracy), static_cast<unsigned>(q.seq),
              static_cast<unsigned>(q.delayTicks),
              static_cast<unsigned long>(q.tMeasUs));
  }

  if (handlers_.quaternion != nullptr) {
    handlers_.quaternion(handlers_.context, q);
  }
}

uint32_t Sh2Client::computeMeasTime(uint32_t tEdgeUs, uint16_t delayTicks) const {
  int64_t t = static_cast<int64_t>(tEdgeUs) + kTimebaseReferenceUs;
  t += static_cast<int64_t>(delayTicks) * 100;
  t += rebaseOffsetUs_;
  if (t < 0) t = 0;
  return static_cast<uint32_t>(t);
}

bool Sh2Client::requestProductIds(uint32_t nowUs) {
  if (transport_ == nullptr) return false;
  const uint8_t request[2] = {sh2::kProdIdReq, 0x00};
  if (!transport_->send(shtp::kChannelControl, request, sizeof(request))) {
    return false;
  }
  prodPending_ = true;
  prodEntries_ = 0;
  prodExpected_ = 4;
  prodDeadlineUs_ = nowUs + timeoutUs();
  if (log_) log_->log(LogLevel::Debug, LogSh2, "s%u -> product id request", sensorId_);
  return true;
}

bool Sh2Client::setFeature(uint8_t reportId, uint32_t intervalUs, uint32_t nowUs) {
  if (transport_ == nullptr) return false;

  uint8_t payload[sh2::kSetFeatureLen];
  for (uint8_t i = 0; i < sizeof(payload); ++i) payload[i] = 0;

  const uint8_t sequence = nextCmdSeq_++;
  payload[0] = sh2::kSetFeatureCmd;
  payload[1] = sequence;
  payload[2] = 0x00;  // flags: no change sensitivity, no wake, no always-on
  // changeSensitivity (3..4) = 0
  payload[5] = static_cast<uint8_t>(intervalUs & 0xFF);
  payload[6] = static_cast<uint8_t>((intervalUs >> 8) & 0xFF);
  payload[7] = static_cast<uint8_t>((intervalUs >> 16) & 0xFF);
  payload[8] = static_cast<uint8_t>((intervalUs >> 24) & 0xFF);
  // batchInterval (9..12) = 0, sensorSpecific (13..16) = 0

  if (!transport_->send(shtp::kChannelControl, payload, sizeof(payload))) {
    return false;
  }

  cmdPending_ = true;
  cmdId_ = sh2::kSetFeatureCmd;
  cmdSeq_ = sequence;
  cmdDeadlineUs_ = nowUs + timeoutUs();
  cmdState_ = CmdState::Pending;

  if (log_) {
    log_->log(LogLevel::Info, LogSh2, "s%u -> set feature 0x%02X @ %lu us", sensorId_,
              static_cast<unsigned>(reportId),
              static_cast<unsigned long>(intervalUs));
  }
  (void)reportId;
  return true;
}

bool Sh2Client::getFeature(uint8_t reportId, uint32_t nowUs) {
  if (transport_ == nullptr) return false;
  const uint8_t payload[2] = {sh2::kGetFeatureReq, reportId};
  if (!transport_->send(shtp::kChannelControl, payload, sizeof(payload))) {
    return false;
  }
  featurePending_ = true;
  featureId_ = reportId;
  featureDeadlineUs_ = nowUs + timeoutUs();
  featureState_ = CmdState::Pending;
  if (log_) {
    log_->log(LogLevel::Debug, LogSh2, "s%u -> get feature 0x%02X", sensorId_,
              static_cast<unsigned>(reportId));
  }
  return true;
}

bool Sh2Client::sendReset() {
  if (transport_ == nullptr) return false;
  const uint8_t command = sh2::kExecReset;
  const bool ok = transport_->send(shtp::kChannelExecutable, &command, 1);
  if (log_) {
    log_->log(ok ? LogLevel::Info : LogLevel::Error, LogReset,
              "s%u software reset %s", sensorId_, ok ? "sent" : "failed");
  }
  return ok;
}

void Sh2Client::pollTimeouts(uint32_t nowUs) {
  if (prodPending_ && expired(nowUs, prodDeadlineUs_)) {
    prodPending_ = false;
    if (!product_.valid) {
      ++stats_.timeouts;
      lastError_ = ErrorCode::Sh2Timeout;
      if (log_) {
        log_->log(LogLevel::Warn, LogSh2, "s%u product id request timed out", sensorId_);
      }
    }
  }

  if (cmdPending_ && expired(nowUs, cmdDeadlineUs_)) {
    cmdPending_ = false;
    cmdState_ = CmdState::Timeout;
    ++stats_.timeouts;
    lastError_ = ErrorCode::Sh2Timeout;
    if (log_) {
      log_->log(LogLevel::Error, LogSh2,
                "s%u command 0x%02X timed out after %u ms", sensorId_,
                static_cast<unsigned>(cmdId_),
                static_cast<unsigned>(timeoutMs_));
    }
  }

  if (featurePending_ && expired(nowUs, featureDeadlineUs_)) {
    featurePending_ = false;
    featureState_ = CmdState::Timeout;
    ++stats_.timeouts;
    lastError_ = ErrorCode::Sh2Timeout;
    if (log_) {
      log_->log(LogLevel::Error, LogSh2,
                "s%u get-feature 0x%02X timed out after %u ms", sensorId_,
                static_cast<unsigned>(featureId_),
                static_cast<unsigned>(timeoutMs_));
    }
  }
}

bool Sh2Client::busy() const {
  return prodPending_ || cmdPending_ || featurePending_;
}

} // namespace bno085
