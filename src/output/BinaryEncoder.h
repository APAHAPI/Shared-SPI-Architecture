#ifndef BNO085_OUTPUT_BINARYENCODER_H
#define BNO085_OUTPUT_BINARYENCODER_H

#include <cstdint>

#include "BNO085Types.h"
#include "output/IOutputSink.h"
#include "output/OutputProtocol.h"

namespace bno085 {

class Logger;

// Encodes records into the versioned binary protocol. Holds no sensor state:
// every field is passed in, so the same encoder can serve any IMU driver.
class BinaryEncoder {
 public:
  BinaryEncoder();

  void attach(IOutputSink* sink, Logger* log);
  void reset();

  uint16_t sequence() const { return sequence_; }
  uint32_t framesWritten() const { return frames_; }
  uint32_t framesDropped() const { return dropped_; }

  bool writeHello(uint8_t sensorCount, uint16_t libraryVersion, uint32_t tBootUs);
  bool writeQuaternion(uint8_t sensorId, const Quaternion& q);
  bool writeInitResult(uint8_t sensorId, uint8_t result, uint8_t stage,
                       const ProductInfo& product, uint32_t grantedIntervalUs);
  bool writeStatus(uint8_t sensorId, const Diagnostics& diagnostics);
  bool writeError(uint8_t sensorId, ErrorCode code, uint8_t context, uint32_t tUs);
  bool writeManagerDiagnostics(uint8_t sensorCount, uint8_t streamingCount,
                               uint16_t busBusy, uint32_t queueDrops,
                               uint32_t uptimeMs, uint32_t maxUpdateUs);

 private:
  bool emit(protocol::PacketType type, const uint8_t* payload, uint16_t len);

  IOutputSink* sink_;
  Logger* log_;
  uint16_t sequence_;
  uint32_t frames_;
  uint32_t dropped_;
  uint8_t frame_[protocol::kMaxFrameBytes];
};

} // namespace bno085

#endif // BNO085_OUTPUT_BINARYENCODER_H
