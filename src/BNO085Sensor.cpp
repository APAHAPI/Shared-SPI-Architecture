#include "BNO085Sensor.h"

#include <cstring>

namespace bno085 {

namespace {
constexpr uint32_t kRetryIntervalUs = 10000;  // re-send cadence while identifying
constexpr uint32_t kSilenceGraceUs = 50000;
constexpr uint32_t kSilenceFactor = 5;  // 2.5 intervals expressed as 5/2
} // namespace

Sensor::Sensor(IHal& hal)
    : hal_(hal), log_(), shtp_(), sh2_(), encoder_(), output_(nullptr),
      manager_(), cfg_(), diag_(), latest_(), newData_(false), started_(false),
      state_(State::Unconfigured), stage_(StageIdle), index_(0),
      stageDeadlineUs_(0), nextAttemptUs_(0), bootPacketBaseline_(0),
      setFeatureSent_(false), getFeatureSent_(false), intPending_(false),
      intEdgeUs_(0), lastStatusUs_(0), lastReportUs_(0), retryAtUs_(0),
      recovering_(false), recoveryAttempts_(0) {}

Status Sensor::begin(const ManagerConfig& managerConfig,
                     const SensorConfig& sensorConfig, uint8_t index) {
  if (started_) {
    return Status{Status::AlreadyStarted, index};
  }

  manager_ = managerConfig;
  cfg_ = sensorConfig;
  index_ = index;

  if (cfg_.csPin < 0) {
    diag_.lastError = ErrorCode::InvalidConfig;
    return Status{Status::InvalidConfig, index};
  }
  if (cfg_.report != Report::RotationVector) {
    // Phase 1 baseline supports the Rotation Vector only. Game Rotation Vector
    // is never substituted silently.
    diag_.lastError = ErrorCode::UnsupportedReport;
    return Status{Status::InvalidConfig, index};
  }
  if (cfg_.intervalUs < kMinIntervalUs || !cfg_.reportsValid()) {
    diag_.lastError = ErrorCode::InvalidConfig;
    return Status{Status::InvalidConfig, index};
  }
  if (manager_.spiHz == 0 || manager_.spiHz > kSpiMaxHz) {
    diag_.lastError = ErrorCode::InvalidConfig;
    return Status{Status::InvalidConfig, index};
  }

  log_.attach(manager_.log);
  log_.setLevel(manager_.logLevel);
  log_.setCategories(manager_.logCategories);

  output_ = manager_.output;
  encoder_.attach(manager_.emitBinaryFrames ? output_ : nullptr, &log_);

  shtp_.attach(&hal_, &log_, index_);
  shtp_.setHandler(&Sensor::channelThunk, this);

  sh2_.attach(&shtp_, &log_, index_);
  sh2_.setTimeoutMs(manager_.recovery.commandTimeoutMs);
  Sh2Client::Handlers handlers;
  handlers.context = this;
  handlers.resetComplete = &Sensor::resetThunk;
  handlers.quaternion = &Sensor::quaternionThunk;
  handlers.sensorReport = nullptr;
  sh2_.setHandlers(handlers);

  const Status halStatus = hal_.begin();
  if (!halStatus.ok()) {
    setState(State::Failed);
    diag_.lastError = ErrorCode::InvalidConfig;
    if (log_.enabled(LogLevel::Error, LogInit)) {
      log_.log(LogLevel::Error, LogInit, "s%u HAL begin failed: %s", index_,
               halStatus.str());
    }
    return halStatus;
  }

  started_ = true;
  recoveryAttempts_ = 0;
  recovering_ = false;
  encoder_.reset();
  encoder_.writeHello(1, 0x0001, hal_.timeUs());
  if (log_.enabled(LogLevel::Info, LogInit)) {
    log_.log(LogLevel::Info, LogInit,
             "s%u begin cs=%d int=%d rst=%d wake=%d rate=%luus", index_,
             static_cast<int>(cfg_.csPin), static_cast<int>(cfg_.intPin),
             static_cast<int>(cfg_.resetPin), static_cast<int>(cfg_.wakePin),
             static_cast<unsigned long>(cfg_.intervalUs));
  }

  startBoot();
  return Status{Status::Ok, index};
}

void Sensor::end() {
  hal_.end();
  started_ = false;
  setState(State::Unconfigured);
}

void Sensor::startBoot() {
  shtp_.reset();
  sh2_.reset();

  stage_ = StageBoot;
  setState(State::Booting);
  diag_.initStage = static_cast<uint8_t>(stage_);
  bootPacketBaseline_ = shtp_.stats().packets;
  setFeatureSent_ = false;
  getFeatureSent_ = false;
  intPending_ = false;

  const uint32_t now = hal_.timeUs();
  stageDeadlineUs_ = now + static_cast<uint32_t>(manager_.recovery.bootTimeoutMs) * 1000u;
  nextAttemptUs_ = now;

  if (cfg_.resetPin >= 0) {
    hal_.pulseReset();
    ++diag_.resetPulses;
    if (log_.enabled(LogLevel::Info, LogReset)) {
      log_.log(LogLevel::Info, LogReset, "s%u reset pulse on pin %d (attempt %u)",
               index_, static_cast<int>(cfg_.resetPin),
               static_cast<unsigned>(recoveryAttempts_));
    }
  } else if (recovering_) {
    // No reset line: ask the device to reset itself over SHTP.
    sh2_.sendReset();
  }

  if (log_.enabled(LogLevel::Info, LogInit)) {
    log_.log(LogLevel::Info, LogInit, "s%u boot wait, budget %ums", index_,
             static_cast<unsigned>(manager_.recovery.bootTimeoutMs));
  }
}

void Sensor::update() {
  if (!started_) return;
  if (state_ == State::Failed || state_ == State::Disabled ||
      state_ == State::Unconfigured) {
    return;
  }

  const uint32_t startUs = hal_.timeUs();

  if (state_ == State::Recovering) {
    if (reached(startUs, retryAtUs_)) {
      startBoot();
    }
  } else {
    serviceBus(startUs);
    sh2_.pollTimeouts(hal_.timeUs());
    advanceInit(hal_.timeUs());
    checkSilence(hal_.timeUs());
    emitStatusIfDue(hal_.timeUs());
  }

  const uint32_t elapsed = hal_.timeUs() - startUs;
  if (elapsed > diag_.maxServiceUs) diag_.maxServiceUs = elapsed;
  ++diag_.serviceCalls;
}

void Sensor::serviceBus(uint32_t nowUs) {
  const bool asserted = hal_.intAsserted();
  if (asserted && !intPending_) {
    intPending_ = true;
    intEdgeUs_ = nowUs;
    ++diag_.intAssertions;
    diag_.lastIntUs = nowUs;
    if (log_.enabled(LogLevel::Debug, LogInt)) {
      log_.log(LogLevel::Debug, LogInt, "s%u INT asserted", index_);
    }
  } else if (!asserted) {
    if (intPending_ && log_.enabled(LogLevel::Trace, LogInt)) {
      log_.log(LogLevel::Trace, LogInt, "s%u INT released", index_);
    }
    intPending_ = false;
  }

  if (!intPending_) {
    mirrorStats();
    return;
  }

  uint8_t visits = 0;
  while (intPending_ && visits < manager_.maxTransfersPerVisit) {
    const ShtpResult result = shtp_.service(intEdgeUs_);
    ++visits;

    switch (result) {
      case ShtpResult::NoData:
        intPending_ = false;
        break;
      case ShtpResult::Spurious:
        ++diag_.intSpurious;
        intPending_ = false;
        break;
      case ShtpResult::Ok:
      case ShtpResult::Fragment:
        break;
      case ShtpResult::BusError:
        setError(ErrorCode::SpiRead, "shtp-read");
        intPending_ = false;
        break;
      case ShtpResult::Malformed:
        setError(ErrorCode::ShtpBadFrame, "shtp-frame");
        break;
      case ShtpResult::Oversize:
        setError(ErrorCode::ShtpBadFrame, "shtp-oversize");
        break;
    }

    if (!hal_.intAsserted()) intPending_ = false;
    if ((hal_.timeUs() - nowUs) > manager_.serviceBudgetUs) break;
  }

  mirrorStats();
}

void Sensor::advanceInit(uint32_t nowUs) {
  switch (stage_) {
    case StageBoot:
      if (shtp_.stats().packets > bootPacketBaseline_) {
        stage_ = StageIdentify;
        setState(State::Identifying);
        diag_.initStage = static_cast<uint8_t>(stage_);
        stageDeadlineUs_ = nowUs + commandTimeoutUs();
        nextAttemptUs_ = nowUs;
        if (log_.enabled(LogLevel::Info, LogInit)) {
          log_.log(LogLevel::Info, LogInit, "s%u device responded, identifying",
                   index_);
        }
      } else if (reached(nowUs, stageDeadlineUs_)) {
        enterFailed(ErrorCode::BootTimeout);
      }
      break;

    case StageIdentify:
      if (sh2_.productInfo().valid) {
        stage_ = StageSetFeature;
        setState(State::Configuring);
        diag_.initStage = static_cast<uint8_t>(stage_);
        stageDeadlineUs_ = nowUs + commandTimeoutUs();
        if (log_.enabled(LogLevel::Info, LogInit)) {
          log_.log(LogLevel::Info, LogInit, "s%u configuring rotation vector",
                   index_);
        }
      } else if (reached(nowUs, stageDeadlineUs_)) {
        enterFailed(ErrorCode::Sh2Timeout);
      } else if (!sh2_.busy() && reached(nowUs, nextAttemptUs_)) {
        sh2_.requestProductIds(nowUs);
        nextAttemptUs_ = nowUs + kRetryIntervalUs;
      }
      break;

    case StageSetFeature:
      if (!setFeatureSent_) {
        setFeatureSent_ = sh2_.setFeature(static_cast<uint8_t>(Report::RotationVector),
                                         cfg_.intervalUs, nowUs);
        if (!setFeatureSent_) {
          nextAttemptUs_ = nowUs + kRetryIntervalUs;
        }
        break;
      }
      if (sh2_.cmdState() == Sh2Client::CmdState::Ok) {
        stage_ = StageGetFeature;
        diag_.initStage = static_cast<uint8_t>(stage_);
        stageDeadlineUs_ = nowUs + commandTimeoutUs();
      } else if (sh2_.cmdState() == Sh2Client::CmdState::HubError) {
        enterFailed(ErrorCode::Sh2BadResponse);
      } else if (sh2_.cmdState() == Sh2Client::CmdState::Timeout) {
        enterFailed(ErrorCode::Sh2Timeout);
      } else if (!sh2_.busy() && reached(nowUs, nextAttemptUs_)) {
        setFeatureSent_ = sh2_.setFeature(
            static_cast<uint8_t>(Report::RotationVector), cfg_.intervalUs, nowUs);
        nextAttemptUs_ = nowUs + kRetryIntervalUs;
      } else if (reached(nowUs, stageDeadlineUs_)) {
        enterFailed(ErrorCode::Sh2Timeout);
      }
      break;

    case StageGetFeature:
      if (!getFeatureSent_) {
        getFeatureSent_ =
            sh2_.getFeature(static_cast<uint8_t>(Report::RotationVector), nowUs);
        break;
      }
      if (sh2_.featureState() == Sh2Client::CmdState::Ok) {
        enterStreaming();
      } else if (sh2_.featureState() == Sh2Client::CmdState::HubError ||
                 sh2_.featureState() == Sh2Client::CmdState::Timeout) {
        enterFailed(ErrorCode::Sh2BadResponse);
      } else if (reached(nowUs, stageDeadlineUs_)) {
        enterFailed(ErrorCode::Sh2Timeout);
      }
      break;

    case StageStream:
      break;

    case StageIdle:
    default:
      break;
  }
}

void Sensor::enterStreaming() {
  stage_ = StageStream;
  setState(State::Streaming);
  diag_.initStage = static_cast<uint8_t>(stage_);
  diag_.grantedIntervalUs = sh2_.grantedIntervalUs();
  recoveryAttempts_ = 0;
  recovering_ = false;
  lastReportUs_ = hal_.timeUs();
  lastStatusUs_ = lastReportUs_;

  encoder_.writeInitResult(index_, 0, static_cast<uint8_t>(StageStream),
                           sh2_.productInfo(), diag_.grantedIntervalUs);
  if (log_.enabled(LogLevel::Info, LogInit)) {
    log_.log(LogLevel::Info, LogInit,
             "s%u streaming at %luus (requested %luus)", index_,
             static_cast<unsigned long>(diag_.grantedIntervalUs),
             static_cast<unsigned long>(cfg_.intervalUs));
  }
}

void Sensor::checkSilence(uint32_t nowUs) {
  if (stage_ != StageStream) return;

  const uint32_t limit =
      (cfg_.intervalUs * kSilenceFactor) / 2u + kSilenceGraceUs;
  if (static_cast<uint32_t>(nowUs - lastReportUs_) <= limit) return;

  if (state_ == State::Streaming) {
    setState(State::Degraded);
    setError(ErrorCode::NoReports, "silence");
    encoder_.writeError(index_, ErrorCode::NoReports,
                        static_cast<uint8_t>(stage_), nowUs);
    if (log_.enabled(LogLevel::Warn, LogData)) {
      log_.log(LogLevel::Warn, LogData, "s%u no reports for %luus", index_,
               static_cast<unsigned long>(nowUs - lastReportUs_));
    }
  }
}

void Sensor::emitStatusIfDue(uint32_t nowUs) {
  if (manager_.statusIntervalMs == 0) return;
  const uint32_t interval = static_cast<uint32_t>(manager_.statusIntervalMs) * 1000u;
  if (static_cast<uint32_t>(nowUs - lastStatusUs_) < interval) return;
  lastStatusUs_ = nowUs;
  encoder_.writeStatus(index_, diag_);
}

bool Sensor::latest(Quaternion& out) {
  if (!newData_) return false;
  out = latest_;
  newData_ = false;
  return true;
}

bool Sensor::get(Report report, ImuSample& out) const {
  (void)report;
  (void)out;
  return false;  // optional reports arrive in Phase 2
}

Status Sensor::recover() {
  if (!started_) {
    return Status{Status::InvalidConfig, index_};
  }
  if (recoveryAttempts_ >= manager_.recovery.maxAttempts) {
    enterFailed(ErrorCode::BootTimeout);
    return Status{Status::SensorFailed, index_};
  }

  ++recoveryAttempts_;
  recovering_ = true;
  stage_ = StageBoot;
  setState(State::Recovering);
  diag_.initStage = static_cast<uint8_t>(stage_);

  const uint32_t backoffMs = manager_.recovery.backoffForAttempt(recoveryAttempts_);
  retryAtUs_ = hal_.timeUs() + static_cast<uint32_t>(backoffMs) * 1000u;

  if (log_.enabled(LogLevel::Warn, LogReset)) {
    log_.log(LogLevel::Warn, LogReset, "s%u recover attempt %u/%u in %ums", index_,
             static_cast<unsigned>(recoveryAttempts_),
             static_cast<unsigned>(manager_.recovery.maxAttempts),
             static_cast<unsigned>(backoffMs));
  }
  return Status{Status::Ok, index_};
}

bool Sensor::configure(Report report, uint32_t intervalUs) {
  if (stage_ != StageStream) return false;
  if (report != Report::RotationVector) return false;  // Phase 1 baseline
  if (intervalUs < kMinIntervalUs) return false;

  const bool sent =
      sh2_.setFeature(static_cast<uint8_t>(report), intervalUs, hal_.timeUs());
  if (sent) {
    cfg_.intervalUs = intervalUs;
    if (log_.enabled(LogLevel::Info, LogSh2)) {
      log_.log(LogLevel::Info, LogSh2, "s%u interval -> %luus", index_,
               static_cast<unsigned long>(intervalUs));
    }
  }
  return sent;
}

void Sensor::disable() {
  setState(State::Disabled);
  diag_.initStage = static_cast<uint8_t>(StageIdle);
  if (log_.enabled(LogLevel::Info, LogInit)) {
    log_.log(LogLevel::Info, LogInit, "s%u disabled", index_);
  }
}

void Sensor::onChannel(uint8_t channel, const uint8_t* payload, uint16_t len,
                       uint32_t tUs) {
  sh2_.handlePayload(channel, payload, len, tUs);
}

void Sensor::onQuaternion(const Quaternion& q) {
  latest_ = q;
  newData_ = true;
  diag_.lastReportUs = q.tHostUs;
  diag_.lastAccuracy = q.accuracy;
  ++diag_.reportsDecoded;
  lastReportUs_ = q.tHostUs;

  if (state_ == State::Degraded) {
    setState(State::Streaming);
    if (log_.enabled(LogLevel::Info, LogData)) {
      log_.log(LogLevel::Info, LogData, "s%u reports resumed", index_);
    }
  }

  if (manager_.emitBinaryFrames) {
    encoder_.writeQuaternion(index_, q);
  }
}

void Sensor::onResetComplete() {
  ++diag_.resets;
  if (stage_ == StageBoot || stage_ == StageIdentify) {
    return;  // part of the normal boot sequence
  }

  if (log_.enabled(LogLevel::Warn, LogReset)) {
    log_.log(LogLevel::Warn, LogReset, "s%u unexpected device reset, reconfiguring",
             index_);
  }
  stage_ = StageSetFeature;
  setState(State::Configuring);
  diag_.initStage = static_cast<uint8_t>(stage_);
  setFeatureSent_ = false;
  getFeatureSent_ = false;
  stageDeadlineUs_ = hal_.timeUs() + commandTimeoutUs();
  nextAttemptUs_ = hal_.timeUs();
}

void Sensor::mirrorStats() {
  const ShtpStats& s = shtp_.stats();
  diag_.shtpPackets = s.packets;
  diag_.shtpBytes = s.bytes;
  diag_.shtpTxPackets = s.txPackets;
  diag_.shortFragments = s.shortFragments;
  diag_.badChannels = s.badChannels;
  diag_.badFragments = s.badFragments;
  diag_.oversizePayloads = s.oversizePayloads;
  diag_.sequenceGaps = s.sequenceGaps;
  diag_.spuriousInterrupts = s.spurious;
  diag_.spiErrors = s.rxErrors + s.txErrors;
  diag_.shtpErrors = s.rxErrors;

  const Sh2Client::Stats& h = sh2_.stats();
  diag_.sh2Timeouts = h.timeouts;
  diag_.sh2BadResponses = h.badResponses;
  diag_.unknownReports = h.unknownReports;
  diag_.duplicateReports = h.duplicates;
  diag_.missedReports = h.reportGaps;
}

void Sensor::setState(State state) {
  if (state_ == state) return;
  state_ = state;
  diag_.state = state;
  if (log_.enabled(LogLevel::Debug, LogInit)) {
    log_.log(LogLevel::Debug, LogInit, "s%u state -> %s", index_, stateName(state));
  }
}

void Sensor::setError(ErrorCode code, const char* where) {
  diag_.lastError = code;
  diag_.lastErrorUs = hal_.timeUs();
  if (log_.enabled(LogLevel::Warn, LogSh2)) {
    log_.log(LogLevel::Warn, LogSh2, "s%u error %s (%s)", index_, errorName(code),
             where);
  }
}

void Sensor::enterFailed(ErrorCode code) {
  setState(State::Failed);
  setError(code, "init");
  encoder_.writeInitResult(index_, static_cast<uint8_t>(code),
                           static_cast<uint8_t>(stage_), sh2_.productInfo(),
                           diag_.grantedIntervalUs);
  encoder_.writeError(index_, code, static_cast<uint8_t>(stage_), hal_.timeUs());
  if (log_.enabled(LogLevel::Error, LogInit)) {
    log_.log(LogLevel::Error, LogInit, "s%u init failed: %s", index_,
             errorName(code));
  }
}

void Sensor::channelThunk(void* context, uint8_t channel, const uint8_t* payload,
                          uint16_t len, uint32_t tUs) {
  static_cast<Sensor*>(context)->onChannel(channel, payload, len, tUs);
}

void Sensor::quaternionThunk(void* context, const Quaternion& q) {
  static_cast<Sensor*>(context)->onQuaternion(q);
}

void Sensor::resetThunk(void* context) {
  static_cast<Sensor*>(context)->onResetComplete();
}

} // namespace bno085
