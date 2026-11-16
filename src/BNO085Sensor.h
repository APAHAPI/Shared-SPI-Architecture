#ifndef BNO085_SENSOR_H
#define BNO085_SENSOR_H

#include <cstdint>

#include "BNO085Config.h"
#include "BNO085Types.h"
#include "diag/Logger.h"
#include "hal/IHal.h"
#include "output/BinaryEncoder.h"
#include "sh2/Sh2Client.h"
#include "shtp/ShtpTransport.h"

namespace bno085 {

// One BNO085. Owns every piece of mutable state for that device: SHTP context,
// SH-2 context, buffers, latest values, diagnostics and the init/recovery state
// machine. Nothing here is static, so instances cannot interfere.
class Sensor {
 public:
  explicit Sensor(IHal& hal);

  // Validates the configuration, opens the HAL and starts the non-blocking
  // initialisation sequence. Returns promptly; completion is asynchronous.
  Status begin(const ManagerConfig& managerConfig,
               const SensorConfig& sensorConfig, uint8_t index = 0);

  void end();

  // Advances this sensor by at most `serviceBudgetUs`. Call frequently.
  void update();

  // Latest-arrival access: true only once per new report.
  bool latest(Quaternion& out);
  const Quaternion& quaternion() const { return latest_; }

  // Optional reports are a Phase 2+ feature; Phase 1 reports unsupported.
  bool get(Report report, ImuSample& out) const;

  const Diagnostics& diagnostics() const { return diag_; }
  const ProductInfo& productInfo() const { return sh2_.productInfo(); }
  State state() const { return state_; }
  bool streaming() const {
    return state_ == State::Streaming || state_ == State::Degraded;
  }
  uint8_t index() const { return index_; }
  const SensorConfig& config() const { return cfg_; }
  ErrorCode lastError() const { return diag_.lastError; }

  // Requests a reset + re-initialisation of this sensor only. Backoff and the
  // attempt limit come from ManagerConfig::recovery.
  Status recover();

  // Sends a new report interval for the primary report. Does not block.
  bool configure(Report report, uint32_t intervalUs);
  void disable();

 private:
  enum InitStage : uint8_t {
    StageIdle = 0,
    StageBoot,
    StageIdentify,
    StageSetFeature,
    StageGetFeature,
    StageStream,
  };

  static void channelThunk(void* context, uint8_t channel,
                           const uint8_t* payload, uint16_t len, uint32_t tUs);
  static void quaternionThunk(void* context, const Quaternion& q);
  static void resetThunk(void* context);

  void onChannel(uint8_t channel, const uint8_t* payload, uint16_t len,
                 uint32_t tUs);
  void onQuaternion(const Quaternion& q);
  void onResetComplete();

  void startBoot();
  void serviceBus(uint32_t nowUs);
  void advanceInit(uint32_t nowUs);
  void checkSilence(uint32_t nowUs);
  void emitStatusIfDue(uint32_t nowUs);
  void mirrorStats();
  void setState(State state);
  void setError(ErrorCode code, const char* where);
  void enterFailed(ErrorCode code);
  void enterStreaming();

  static bool reached(uint32_t nowUs, uint32_t deadlineUs) {
    return static_cast<int32_t>(nowUs - deadlineUs) >= 0;
  }
  uint32_t commandTimeoutUs() const {
    return static_cast<uint32_t>(manager_.recovery.commandTimeoutMs) * 1000u;
  }

  IHal& hal_;
  Logger log_;
  ShtpTransport shtp_;
  Sh2Client sh2_;
  BinaryEncoder encoder_;
  IOutputSink* output_;
  ManagerConfig manager_;
  SensorConfig cfg_;
  Diagnostics diag_;
  Quaternion latest_;
  bool newData_;
  bool started_;
  State state_;
  InitStage stage_;
  uint8_t index_;

  uint32_t stageDeadlineUs_;
  uint32_t nextAttemptUs_;
  uint32_t bootPacketBaseline_;
  bool setFeatureSent_;
  bool getFeatureSent_;

  bool intPending_;
  uint32_t intEdgeUs_;

  uint32_t lastStatusUs_;
  uint32_t lastReportUs_;
  uint32_t retryAtUs_;
  bool recovering_;
  uint8_t recoveryAttempts_;
};

} // namespace bno085

#endif // BNO085_SENSOR_H
