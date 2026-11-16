#ifndef BNO085_SH2_CLIENT_H
#define BNO085_SH2_CLIENT_H

#include <cstdint>

#include "BNO085Types.h"
#include "sh2/Sh2Reports.h"
#include "shtp/ShtpTransport.h"

namespace bno085 {

class Logger;

// Minimal SH-2 client for one sensor: product identification, feature
// configuration, Rotation Vector decoding and hub timestamps.
//
// Requests are asynchronous. Each request sets a pending flag and a deadline;
// the response is matched by report ID (and, where echoed, command sequence).
// Nothing here blocks, and no request can leave the client stuck forever.
class Sh2Client {
 public:
  enum class CmdState : uint8_t {
    Idle = 0,
    Pending,
    Ok,
    HubError,
    Timeout,
  };

  struct Handlers {
    void* context = nullptr;
    void (*resetComplete)(void* context) = nullptr;
    void (*quaternion)(void* context, const Quaternion& q) = nullptr;
    void (*sensorReport)(void* context, uint8_t reportId, const uint8_t* report,
                         uint8_t length, uint32_t tUs) = nullptr;
  };

  struct Stats {
    uint16_t timeouts = 0;
    uint16_t badResponses = 0;
    uint16_t unexpectedResponses = 0;
    uint16_t unknownReports = 0;
    uint16_t duplicates = 0;
    uint16_t baseTimestampRefs = 0;
    uint32_t reports = 0;
    uint32_t reportGaps = 0;
  };

  Sh2Client();

  void attach(ShtpTransport* transport, Logger* log, uint8_t sensorId);
  void setHandlers(const Handlers& handlers) { handlers_ = handlers; }
  void setTimeoutMs(uint16_t timeoutMs) { timeoutMs_ = timeoutMs; }

  // Clears pending requests and decoded state. Statistics are kept.
  void reset();

  // Entry point from the SHTP transport.
  void handlePayload(uint8_t channel, const uint8_t* payload, uint16_t len,
                     uint32_t tUs);

  bool requestProductIds(uint32_t nowUs);
  bool setFeature(uint8_t reportId, uint32_t intervalUs, uint32_t nowUs);
  bool getFeature(uint8_t reportId, uint32_t nowUs);
  bool sendReset();

  void pollTimeouts(uint32_t nowUs);

  bool busy() const;
  CmdState cmdState() const { return cmdState_; }
  CmdState featureState() const { return featureState_; }
  const ProductInfo& productInfo() const { return product_; }
  uint32_t grantedIntervalUs() const { return grantedIntervalUs_; }
  uint32_t epoch() const { return epoch_; }
  ErrorCode lastError() const { return lastError_; }
  const Stats& stats() const { return stats_; }

 private:
  uint32_t timeoutUs() const {
    return static_cast<uint32_t>(timeoutMs_) * 1000u;
  }
  static bool expired(uint32_t nowUs, uint32_t deadlineUs) {
    return static_cast<int32_t>(nowUs - deadlineUs) >= 0;
  }

  void handleExecutable(const uint8_t* payload, uint16_t len);
  void handleControl(const uint8_t* payload, uint16_t len, uint32_t tUs);
  void handleReports(const uint8_t* payload, uint16_t len, uint32_t tUs);
  void handleRotationVector(const uint8_t* report, uint8_t length, uint32_t tUs);
  void handleProdId(const uint8_t* report);
  void handleCommandResponse(const uint8_t* report);
  void handleGetFeatureResponse(const uint8_t* report);
  void handleTimestampRebase(const uint8_t* report);

  uint32_t computeMeasTime(uint32_t tEdgeUs, uint16_t delayTicks) const;

  ShtpTransport* transport_;
  Logger* log_;
  uint8_t sensorId_;
  Handlers handlers_;
  uint16_t timeoutMs_;

  ProductInfo product_;
  uint8_t prodEntries_;
  uint8_t prodExpected_;
  bool prodPending_;
  uint32_t prodDeadlineUs_;

  bool cmdPending_;
  uint8_t cmdId_;
  uint8_t cmdSeq_;
  uint8_t nextCmdSeq_;
  uint32_t cmdDeadlineUs_;
  CmdState cmdState_;

  bool featurePending_;
  uint8_t featureId_;
  uint32_t featureDeadlineUs_;
  CmdState featureState_;
  uint32_t grantedIntervalUs_;

  int32_t rebaseOffsetUs_;
  uint32_t epoch_;

  bool haveQuaternion_;
  uint8_t lastQuatSeq_;
  uint32_t lastQuatTMeasUs_;

  Stats stats_;
  ErrorCode lastError_;
};

} // namespace bno085

#endif // BNO085_SH2_CLIENT_H
