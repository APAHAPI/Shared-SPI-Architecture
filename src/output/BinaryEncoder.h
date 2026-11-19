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
//
// Output outcomes are classified so the application can tell an intentional
// non-emission from real loss:
//   Ok          - the whole frame was accepted
//   Suppressed  - no sink attached; nothing was expected to go out
//   TooLarge    - payload exceeded the protocol limit
//   NoCapacity  - the sink reported less free space than the frame needs;
//                 nothing was written and no partial frame exists
//   PartialWrite- the sink accepted a prefix of the frame
//   FailedWrite - the sink accepted zero bytes
//
// A real attempt (anything after Suppressed/TooLarge) consumes a frame
// sequence number, so a consumer sees a gap after loss. Suppressed frames do
// not consume a sequence number.
class BinaryEncoder {
 public:
  enum class EmitResult : uint8_t {
    Ok = 0,
    Suppressed,
    TooLarge,
    NoCapacity,
    PartialWrite,
    FailedWrite,
  };

  struct OutputStats {
    uint32_t written = 0;
    uint32_t suppressed = 0;
    uint32_t tooLarge = 0;
    uint32_t noCapacity = 0;
    uint32_t partial = 0;
    uint32_t failed = 0;

    uint32_t dropped() const {
      return tooLarge + noCapacity + partial + failed;
    }
  };

  BinaryEncoder();

  void attach(IOutputSink* sink, Logger* log);
  void reset();

  uint16_t sequence() const { return sequence_; }
  uint32_t framesWritten() const { return stats_.written; }
  uint32_t framesDropped() const { return stats_.dropped(); }
  uint32_t framesSuppressed() const { return stats_.suppressed; }
  uint32_t framesTooLarge() const { return stats_.tooLarge; }
  uint32_t framesNoCapacity() const { return stats_.noCapacity; }
  uint32_t framesPartial() const { return stats_.partial; }
  uint32_t framesFailed() const { return stats_.failed; }
  const OutputStats& outputStats() const { return stats_; }
  EmitResult lastResult() const { return lastResult_; }

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
  OutputStats stats_;
  EmitResult lastResult_;
  bool loggedDrop_;
  uint8_t frame_[protocol::kMaxFrameBytes];
};

} // namespace bno085

#endif // BNO085_OUTPUT_BINARYENCODER_H
