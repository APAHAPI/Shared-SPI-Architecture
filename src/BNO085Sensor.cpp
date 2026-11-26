#include "BNO085Sensor.h"

#include <cstring>

namespace bno085 {

namespace {
constexpr uint32_t kRetryIntervalUs = 10000;  // re-send cadence while identifying
// The hub emits its Get Feature Response only after the new report period has
// been applied. Measured on the bench at ~120 ms when the RV starts at 100 Hz,
// so the command timeout (100 ms) is not enough for this exchange.
constexpr uint32_t kFeatureApplyTimeoutUs = 250000;
constexpr uint32_t kSilenceGraceUs = 50000;
constexpr uint32_t kSilenceFactor = 5;  // 2.5 intervals expressed as 5/2
} // namespace

Sensor::Sensor(IHal& hal)
    : hal_(hal), log_(), shtp_(), sh2_(), encoder_(), output_(nullptr),
      manager_(), cfg_(), diag_(), latest_(), newData_(false), started_(false),
      state_(State::Unconfigured), stage_(StageIdle), index_(0),
      stageDeadlineUs_(0), nextAttemptUs_(0), bootEpochBaseline_(0),
      setFeatureSent_(false), intPending_(false),
      intEdgeUs_(0), lastStatusUs_(0), lastReportUs_(0), retryAtUs_(0),
      recovering_(false), recoveryAttempts_(0),
      lastManualRecoverUs_(0), manualRecoverUsed_(false),
      consecutiveBusErrors_(0),
      consecutiveProtocolErrors_(0), silenceSinceUs_(0), outputEnabled_(false),
      lastOutputLogUs_(0) {}

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
  outputEnabled_ = manager_.emitBinaryFrames && output_ != nullptr;
  if (manager_.emitBinaryFrames && output_ == nullptr) {
    // Keep the historical default (binary output on) from silently doing
    // nothing when no sink was supplied. The sensor still runs; the reason is
    // visible in diagnostics and the log.
    diag_.lastOutputError = ErrorCode::OutputUnavailable;
    if (log_.enabled(LogLevel::Warn, LogData)) {
      log_.log(LogLevel::Warn, LogData,
               "s%u binary output requested but no sink; disabled", index_);
    }
  }
  encoder_.attach(outputEnabled_ ? output_ : nullptr, &log_);

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
  lastManualRecoverUs_ = 0;
  manualRecoverUsed_ = false;
  consecutiveBusErrors_ = 0;
  consecutiveProtocolErrors_ = 0;
  silenceSinceUs_ = 0;
  diag_.outputEnabled = outputEnabled_;
  diag_.recoveryAttempts = 0;
  diag_.recoverySuccesses = 0;
  diag_.recoveryFailures = 0;
  diag_.recoveryTerminal = false;
  diag_.lastRecoveryReason = ErrorCode::None;
  diag_.lastRecoveryUs = 0;
  encoder_.reset();
  if (outputEnabled_) {
    observeOutput(encoder_.writeHello(1, 0x0001, hal_.timeUs()), "hello");
  }
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
  recovering_ = false;
  outputEnabled_ = false;
  diag_.outputEnabled = false;
  setState(State::Unconfigured);
}

void Sensor::startBoot() {
  shtp_.reset();
  sh2_.reset();

  stage_ = StageBoot;
  setState(State::Booting);
  diag_.initStage = static_cast<uint8_t>(stage_);
  // The hub is not ready for commands until it has reported reset-complete on
  // the executable channel. The advertisement alone proves the link works but
  // can arrive while the hub is still booting, so it must not open the command
  // phase: a command sent that early is answered with an unsolicited hub status
  // and then times out.
  bootEpochBaseline_ = sh2_.epoch();
  setFeatureSent_ = false;
  intPending_ = false;
  consecutiveBusErrors_ = 0;
  consecutiveProtocolErrors_ = 0;
  silenceSinceUs_ = 0;

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
    // A transport fault handled inside the drain can start recovery (or hit
    // the terminal state) on this same pass. Do not let the remaining init
    // stages run against the stage that recovery just rewound.
    if (state_ != State::Recovering && state_ != State::Failed) {
      sh2_.pollTimeouts(hal_.timeUs());
      advanceInit(hal_.timeUs());
      checkSilence(hal_.timeUs());
      emitStatusIfDue(hal_.timeUs());
    }
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

    ErrorCode recoveryReason = ErrorCode::None;
    switch (result) {
      case ShtpResult::NoData:
        intPending_ = false;
        break;
      case ShtpResult::Spurious:
        ++diag_.intSpurious;
        consecutiveBusErrors_ = 0;
        consecutiveProtocolErrors_ = 0;
        intPending_ = false;
        break;
      case ShtpResult::Ok:
        consecutiveBusErrors_ = 0;
        consecutiveProtocolErrors_ = 0;
        break;
      case ShtpResult::Fragment:
        consecutiveBusErrors_ = 0;
        break;
      case ShtpResult::BusError:
        setError(ErrorCode::SpiRead, "shtp-read");
        intPending_ = false;
        if (++consecutiveBusErrors_ >=
                manager_.recovery.transientErrorThreshold &&
            stage_ == StageStream) {
          recoveryReason = ErrorCode::SpiRead;
        }
        break;
      case ShtpResult::Malformed:
        setError(ErrorCode::ShtpBadFrame, "shtp-frame");
        if (++consecutiveProtocolErrors_ >=
                manager_.recovery.transientErrorThreshold &&
            stage_ == StageStream) {
          recoveryReason = ErrorCode::ShtpBadFrame;
        }
        break;
      case ShtpResult::Oversize:
        // A legal but unbufferable transfer (the boot advertisement) is
        // drained by the transport. Only repeated oversize while streaming
        // suggests a desynchronised link.
        setError(ErrorCode::ShtpBadFrame, "shtp-oversize");
        if (++consecutiveProtocolErrors_ >=
                manager_.recovery.transientErrorThreshold &&
            stage_ == StageStream) {
          recoveryReason = ErrorCode::ShtpBadFrame;
        }
        break;
    }

    if (recoveryReason != ErrorCode::None) {
      consecutiveBusErrors_ = 0;
      consecutiveProtocolErrors_ = 0;
      failRecoverable(recoveryReason);
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
      // Reset-complete, not the advertisement, is the readiness signal: it is
      // the hub telling the host that its own initialization has finished.
      if (sh2_.epoch() != bootEpochBaseline_) {
        stage_ = StageIdentify;
        setState(State::Identifying);
        diag_.initStage = static_cast<uint8_t>(stage_);
        stageDeadlineUs_ = nowUs + commandTimeoutUs();
        nextAttemptUs_ = nowUs;
        if (log_.enabled(LogLevel::Info, LogInit)) {
          log_.log(LogLevel::Info, LogInit, "s%u device reset complete, identifying",
                   index_);
        }
      } else if (reached(nowUs, stageDeadlineUs_)) {
        failRecoverable(ErrorCode::BootTimeout);
      }
      break;

    case StageIdentify:
      if (sh2_.productInfo().valid) {
        diag_.product = sh2_.productInfo();
        stage_ = StageSetFeature;
        setState(State::Configuring);
        diag_.initStage = static_cast<uint8_t>(stage_);
        stageDeadlineUs_ = nowUs + commandTimeoutUs();
        if (log_.enabled(LogLevel::Info, LogInit)) {
          log_.log(LogLevel::Info, LogInit, "s%u configuring rotation vector",
                   index_);
        }
      } else if (reached(nowUs, stageDeadlineUs_)) {
        failRecoverable(ErrorCode::Sh2Timeout);
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
        } else {
          // The hub answers a Set Feature with a Get Feature Response once the
          // new period is applied (datasheet 1.3.5). An explicit request sent
          // immediately after the Set Feature is answered from the old config
          // (a zero interval), so wait for the hub's own response instead.
          sh2_.expectFeatureResponse(static_cast<uint8_t>(Report::RotationVector),
                                     nowUs, kFeatureApplyTimeoutUs);
        }
        break;
      }
      if (sh2_.cmdState() == Sh2Client::CmdState::Ok) {
        stage_ = StageGetFeature;
        diag_.initStage = static_cast<uint8_t>(stage_);
        stageDeadlineUs_ = nowUs + kFeatureApplyTimeoutUs;
        nextAttemptUs_ = nowUs;
      } else if (sh2_.cmdState() == Sh2Client::CmdState::HubError) {
        failRecoverable(ErrorCode::Sh2BadResponse);
      } else if (sh2_.cmdState() == Sh2Client::CmdState::Timeout) {
        failRecoverable(ErrorCode::Sh2Timeout);
      } else if (!sh2_.busy() && reached(nowUs, nextAttemptUs_)) {
        setFeatureSent_ = sh2_.setFeature(
            static_cast<uint8_t>(Report::RotationVector), cfg_.intervalUs, nowUs);
        if (setFeatureSent_) {
          sh2_.expectFeatureResponse(static_cast<uint8_t>(Report::RotationVector),
                                     nowUs, kFeatureApplyTimeoutUs);
        }
        nextAttemptUs_ = nowUs + kRetryIntervalUs;
      } else if (reached(nowUs, stageDeadlineUs_)) {
        failRecoverable(ErrorCode::Sh2Timeout);
      }
      break;

    case StageGetFeature:
      if (sh2_.featureState() == Sh2Client::CmdState::Ok) {
        enterStreaming();
      } else if (sh2_.featureState() == Sh2Client::CmdState::HubError) {
        failRecoverable(ErrorCode::Sh2BadResponse);
      } else if (sh2_.featureState() == Sh2Client::CmdState::Timeout) {
        failRecoverable(ErrorCode::Sh2Timeout);
      } else if (reached(nowUs, stageDeadlineUs_)) {
        failRecoverable(ErrorCode::Sh2Timeout);
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
  if (diag_.recoveryAttempts > 0) {
    if (diag_.recoverySuccesses < 0xFFFFu) ++diag_.recoverySuccesses;
  }
  recoveryAttempts_ = 0;
  recovering_ = false;
  diag_.recoveryAttempts = 0;
  diag_.recoveryTerminal = false;
  consecutiveBusErrors_ = 0;
  consecutiveProtocolErrors_ = 0;
  silenceSinceUs_ = 0;
  lastReportUs_ = hal_.timeUs();
  lastStatusUs_ = lastReportUs_;

  if (outputEnabled_) {
    observeOutput(encoder_.writeInitResult(index_, 0,
                                           static_cast<uint8_t>(StageStream),
                                           sh2_.productInfo(),
                                           diag_.grantedIntervalUs),
                  "init-result");
  }
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
  if (static_cast<uint32_t>(nowUs - lastReportUs_) <= limit) {
    silenceSinceUs_ = 0;
    return;
  }

  if (state_ == State::Streaming) {
    setState(State::Degraded);
    setError(ErrorCode::NoReports, "silence");
    silenceSinceUs_ = nowUs;
    if (outputEnabled_) {
      observeOutput(encoder_.writeError(index_, ErrorCode::NoReports,
                                        static_cast<uint8_t>(stage_), nowUs),
                    "silence");
    }
    if (log_.enabled(LogLevel::Warn, LogData)) {
      log_.log(LogLevel::Warn, LogData, "s%u no reports for %luus", index_,
               static_cast<unsigned long>(nowUs - lastReportUs_));
    }
    return;
  }

  if (state_ == State::Degraded && manager_.recovery.silenceRecoveryMs > 0) {
    const uint32_t recoverAfter =
        static_cast<uint32_t>(manager_.recovery.silenceRecoveryMs) * 1000u;
    if (static_cast<uint32_t>(nowUs - silenceSinceUs_) >= recoverAfter) {
      failRecoverable(ErrorCode::NoReports);
    }
  }
}

void Sensor::emitStatusIfDue(uint32_t nowUs) {
  if (manager_.statusIntervalMs == 0) return;
  const uint32_t interval = static_cast<uint32_t>(manager_.statusIntervalMs) * 1000u;
  if (static_cast<uint32_t>(nowUs - lastStatusUs_) < interval) return;
  lastStatusUs_ = nowUs;
  if (outputEnabled_) {
    observeOutput(encoder_.writeStatus(index_, diag_), "status");
  }
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
  if (!started_ || state_ == State::Disabled) {
    return Status{Status::InvalidConfig, index_};
  }

  const uint32_t now = hal_.timeUs();
  const uint32_t limit =
      static_cast<uint32_t>(manager_.recovery.manualRecoverIntervalMs) * 1000u;
  if (manualRecoverUsed_ && limit > 0 &&
      static_cast<uint32_t>(now - lastManualRecoverUs_) < limit) {
    if (log_.enabled(LogLevel::Warn, LogReset)) {
      log_.log(LogLevel::Warn, LogReset,
               "s%u manual recover ignored (rate limited)", index_);
    }
    return Status{Status::RecoveryRateLimited, index_};
  }
  lastManualRecoverUs_ = now;
  manualRecoverUsed_ = true;

  // A manual request is an operator action: give it a fresh bounded budget and
  // let it work even when automatic recovery is disabled.
  recoveryAttempts_ = 0;
  diag_.recoveryAttempts = 0;
  diag_.recoveryTerminal = false;
  scheduleRecovery(ErrorCode::None, true);
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
    silenceSinceUs_ = 0;
    if (log_.enabled(LogLevel::Info, LogData)) {
      log_.log(LogLevel::Info, LogData, "s%u reports resumed", index_);
    }
  }

  if (outputEnabled_) {
    observeOutput(encoder_.writeQuaternion(index_, q), "quaternion");
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
  // The device restarted its SHTP sequence counters. Keeping ours would count
  // a gap on every packet after the reset even though the link is healthy.
  shtp_.reset();
  intPending_ = false;
  stage_ = StageSetFeature;
  setState(State::Configuring);
  diag_.initStage = static_cast<uint8_t>(stage_);
  setFeatureSent_ = false;
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
  mirrorOutputStats();
}

void Sensor::mirrorOutputStats() {
  const BinaryEncoder::OutputStats& o = encoder_.outputStats();
  diag_.framesWritten = o.written;
  diag_.framesSuppressed = o.suppressed;
  diag_.framesTooLarge = o.tooLarge;
  diag_.framesNoCapacity = o.noCapacity;
  diag_.framesPartial = o.partial;
  diag_.framesFailed = o.failed;
  diag_.framesDropped = o.dropped();
}

void Sensor::observeOutput(bool ok, const char* where) {
  mirrorOutputStats();
  if (ok) return;

  diag_.lastOutputError = ErrorCode::OutputOverflow;
  diag_.lastOutputErrorUs = hal_.timeUs();

  // One log per second: an unavailable sink would otherwise flood the log from
  // inside the report path and make a slow serial link worse.
  const uint32_t now = hal_.timeUs();
  if (lastOutputLogUs_ == 0 ||
      static_cast<uint32_t>(now - lastOutputLogUs_) >= 1000000u) {
    lastOutputLogUs_ = now;
    if (log_.enabled(LogLevel::Warn, LogData)) {
      log_.log(LogLevel::Warn, LogData,
               "s%u output drop (%s): total=%lu cap=%lu partial=%lu failed=%lu",
               index_, where, static_cast<unsigned long>(diag_.framesDropped),
               static_cast<unsigned long>(diag_.framesNoCapacity),
               static_cast<unsigned long>(diag_.framesPartial),
               static_cast<unsigned long>(diag_.framesFailed));
    }
  }
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

void Sensor::scheduleRecovery(ErrorCode reason, bool manual) {
  if (!started_ || state_ == State::Disabled) return;
  if (!manual && !manager_.recovery.autoRecover) {
    enterTerminalFailed(reason);
    return;
  }
  if (recoveryAttempts_ >= manager_.recovery.maxAttempts) {
    enterTerminalFailed(reason);
    return;
  }

  ++recoveryAttempts_;
  diag_.recoveryAttempts = recoveryAttempts_;
  diag_.lastRecoveryUs = hal_.timeUs();
  if (reason != ErrorCode::None) {
    diag_.lastRecoveryReason = reason;
    setError(reason, manual ? "manual-recover" : "recover");
  }

  recovering_ = true;
  diag_.recoveryTerminal = false;
  stage_ = StageBoot;
  setState(State::Recovering);
  diag_.initStage = static_cast<uint8_t>(stage_);

  const uint8_t attemptIndex =
      (recoveryAttempts_ > 0xFFu) ? 0xFFu
                                  : static_cast<uint8_t>(recoveryAttempts_);
  const uint16_t backoffMs =
      manager_.recovery.backoffForAttempt(attemptIndex);
  retryAtUs_ = hal_.timeUs() + static_cast<uint32_t>(backoffMs) * 1000u;

  if (log_.enabled(LogLevel::Warn, LogReset)) {
    log_.log(LogLevel::Warn, LogReset, "s%u recover attempt %u/%u in %ums%s",
             index_, static_cast<unsigned>(recoveryAttempts_),
             static_cast<unsigned>(manager_.recovery.maxAttempts),
             static_cast<unsigned>(backoffMs), manual ? " (manual)" : "");
  }
}

void Sensor::failRecoverable(ErrorCode code) {
  if (diag_.recoveryFailures < 0xFFFFu) ++diag_.recoveryFailures;
  if (log_.enabled(LogLevel::Info, LogInit)) {
    log_.log(LogLevel::Info, LogInit,
             "s%u recoverable error stage=%u cmd=%u feat=%u setSent=%u busy=%u",
             index_, static_cast<unsigned>(stage_),
             static_cast<unsigned>(sh2_.cmdState()),
             static_cast<unsigned>(sh2_.featureState()),
             setFeatureSent_ ? 1u : 0u, sh2_.busy() ? 1u : 0u);
  }
  if (!manager_.recovery.autoRecover) {
    enterTerminalFailed(code);
    return;
  }
  scheduleRecovery(code, false);
}

void Sensor::enterTerminalFailed(ErrorCode code) {
  recovering_ = false;
  setState(State::Failed);
  setError(code, "terminal");
  diag_.recoveryTerminal = true;
  diag_.lastRecoveryReason = code;
  diag_.lastRecoveryUs = hal_.timeUs();

  if (outputEnabled_) {
    observeOutput(encoder_.writeInitResult(
                      index_, static_cast<uint8_t>(code),
                      static_cast<uint8_t>(stage_), sh2_.productInfo(),
                      diag_.grantedIntervalUs),
                  "init-result");
    observeOutput(encoder_.writeError(index_, code,
                                      static_cast<uint8_t>(stage_),
                                      hal_.timeUs()),
                  "error");
  }

  if (log_.enabled(LogLevel::Error, LogInit)) {
    log_.log(LogLevel::Error, LogInit, "s%u terminal failure: %s", index_,
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
