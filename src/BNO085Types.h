#ifndef BNO085_TYPES_H
#define BNO085_TYPES_H

#include <cstddef>
#include <cstdint>

#include "BNO085Config.h"
#include "diag/LogTypes.h"

namespace bno085 {

class IOutputSink;

// ---------------------------------------------------------------------------
// Reports. Enumerator values are the SH-2 report IDs, so the same number
// identifies a report in configuration, on the wire and in diagnostics.
// ---------------------------------------------------------------------------
enum class Report : uint8_t {
  Accelerometer             = 0x01,
  GyroscopeCalibrated       = 0x02,
  MagneticFieldCalibrated   = 0x03,
  LinearAcceleration        = 0x04,
  RotationVector            = 0x05,
  Gravity                   = 0x06,
  GyroscopeUncalibrated     = 0x07,
  GameRotationVector        = 0x08,
  GeomagneticRotationVector = 0x09,
  Pressure                  = 0x0A,
  AmbientLight              = 0x0B,
  Humidity                  = 0x0C,
  Temperature               = 0x0E,
  MagneticFieldUncalibrated = 0x0F,
  StepCounter               = 0x11,
  StabilityClassifier       = 0x13,
  RawAccelerometer          = 0x14,
  RawGyroscope              = 0x15,
  RawMagnetometer           = 0x16,
};

const char* reportName(Report report);

enum class Accuracy : uint8_t {
  Unreliable = 0,
  Low        = 1,
  Medium     = 2,
  High       = 3,
};

inline Accuracy accuracyFromStatus(uint8_t status) {
  switch (status & 0x03u) {
    case 0:  return Accuracy::Unreliable;
    case 1:  return Accuracy::Low;
    case 2:  return Accuracy::Medium;
    default: return Accuracy::High;
  }
}

const char* accuracyName(Accuracy accuracy);

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
enum class State : uint8_t {
  Unconfigured = 0,
  Booting,
  Identifying,
  Configuring,
  Streaming,
  Degraded,
  Recovering,
  Failed,
  Disabled,
};

const char* stateName(State state);

enum class ErrorCode : uint8_t {
  None               = 0,
  InvalidConfig      = 1,
  DuplicatePin       = 2,
  SpiRead            = 3,
  SpiWrite           = 4,
  ShtpBadFrame       = 5,
  ShtpBadChannel     = 6,
  ShtpSequenceGap    = 7,
  Sh2Timeout         = 8,
  Sh2BadResponse     = 9,
  ProductIdMismatch  = 10,
  BootTimeout        = 11,
  NoReports          = 12,
  BusTimeout         = 13,
  QueueOverflow      = 14,
  OutputOverflow     = 15,
  GrvFallbackUsed    = 16,
  UnsupportedReport  = 17,
};

const char* errorName(ErrorCode code);

struct Status {
  enum Code : uint8_t {
    Ok = 0,
    InvalidConfig,
    DuplicatePin,
    NoSensors,
    NoMemory,
    BusUnavailable,
    OutputUnavailable,
    AlreadyStarted,
    SensorFailed,
  };

  Code code = Ok;
  uint8_t sensorId = 0xFF;

  bool ok() const { return code == Ok; }
  const char* str() const;
};

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------

// Primary orientation output. Device values are preserved exactly; the float
// members are the same values divided by 16384 and are provided for
// convenience only.
struct Quaternion {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  int16_t rawW = 0;          // Q14, as sent by the device
  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;
  int16_t rawAccuracy = 0;   // Q12, radians

  Accuracy accuracy = Accuracy::Unreliable;
  uint32_t tMeasUs = 0;      // device measurement time
  uint32_t tHostUs = 0;      // host time at INT assertion
  uint8_t seq = 0;
  uint8_t epoch = 0;
  Report source = Report::RotationVector;
  uint8_t status = 0;        // raw status byte
  uint16_t delayTicks = 0;   // raw 14-bit delay field, 100 us units
  bool valid = false;
};

struct ImuSample {
  uint8_t reportId = 0;
  Accuracy accuracy = Accuracy::Unreliable;
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  int8_t scaleExponent = 0;
  uint32_t tMeasUs = 0;
  uint32_t tHostUs = 0;
  uint8_t seq = 0;
  uint8_t epoch = 0;
  bool valid = false;
};

enum class RecordKind : uint8_t {
  Quaternion = 0,
  Imu = 1,
  SensorStatus = 2,
};

// Aggregated record. Reserved for the Phase 3 aggregator; Phase 1 exposes the
// per-sensor accessors directly.
struct Record {
  uint8_t sensorId = 0;
  RecordKind kind = RecordKind::Quaternion;
  uint32_t tHostUs = 0;
  Quaternion rv;
  ImuSample imu;
};

struct ProductInfo {
  uint32_t partNumber = 0;
  uint32_t buildNumber = 0;
  uint16_t patch = 0;
  uint8_t major = 0;
  uint8_t minor = 0;
  uint8_t resetCause = 0;
  uint8_t entries = 0;
  bool valid = false;
};

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
struct Diagnostics {
  State state = State::Unconfigured;
  ErrorCode lastError = ErrorCode::None;
  Accuracy lastAccuracy = Accuracy::Unreliable;
  uint8_t initStage = 0;

  ProductInfo product;
  uint32_t grantedIntervalUs = 0;

  // Reset / interrupt / bus
  uint32_t resets = 0;
  uint32_t resetPulses = 0;
  uint32_t intAssertions = 0;
  uint32_t intSpurious = 0;
  uint32_t csTransactions = 0;
  uint32_t spiTransfers = 0;
  uint32_t spiErrors = 0;
  uint32_t busTimeouts = 0;
  uint32_t transferBytes = 0;

  // SHTP
  uint32_t shtpPackets = 0;
  uint32_t shtpBytes = 0;
  uint32_t shtpTxPackets = 0;
  uint32_t shtpErrors = 0;
  uint16_t shortFragments = 0;
  uint16_t badChannels = 0;
  uint16_t badFragments = 0;
  uint16_t oversizePayloads = 0;
  uint16_t sequenceGaps = 0;
  uint16_t spuriousInterrupts = 0;

  // SH-2
  uint16_t sh2Timeouts = 0;
  uint16_t sh2BadResponses = 0;
  uint16_t unknownReports = 0;
  uint16_t duplicateReports = 0;
  uint32_t reportsDecoded = 0;
  uint32_t missedReports = 0;

  // Timing (microseconds, host clock)
  uint32_t lastIntUs = 0;
  uint32_t lastReportUs = 0;
  uint32_t lastErrorUs = 0;
  uint32_t maxServiceUs = 0;
  uint32_t serviceCalls = 0;
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
struct RecoveryPolicy {
  uint8_t maxAttempts = 5;
  uint16_t backoffMs[5] = {250, 500, 1000, 2000, 4000};
  uint16_t bootTimeoutMs = 300;     // datasheet: 94 ms typical
  uint16_t commandTimeoutMs = 100;  // per SH-2 request
  bool resetOnBootTimeout = true;

  uint16_t backoffForAttempt(uint8_t attempt) const {
    if (attempt == 0) return backoffMs[0];
    uint8_t index = static_cast<uint8_t>(attempt - 1);
    if (index >= sizeof(backoffMs) / sizeof(backoffMs[0])) {
      index = static_cast<uint8_t>(sizeof(backoffMs) / sizeof(backoffMs[0]) - 1);
    }
    return backoffMs[index];
  }
};

struct SensorConfig {
  int8_t csPin = -1;     // required, distinct per sensor
  int8_t intPin = -1;    // optional; -1 = polled
  int8_t resetPin = -1;  // optional; -1 = software reset only
  int8_t wakePin = -1;   // optional PS0/WAKE; -1 = not wired
  uint8_t resetGroup = 0;

  Report report = Report::RotationVector;
  uint32_t intervalUs = kDefaultIntervalUs;
  bool allowGrvFallback = false;

  Report extra[kMaxExtraReports];
  uint32_t extraIntervalUs[kMaxExtraReports];
  uint8_t extraCount = 0;

  SensorConfig();
  SensorConfig(int8_t cs, int8_t intr = -1, int8_t rst = -1);

  static SensorConfig spi(int8_t cs, int8_t intr = -1, int8_t rst = -1) {
    return SensorConfig(cs, intr, rst);
  }

  SensorConfig& enable(Report report, uint32_t intervalUs);
  SensorConfig& disable(Report report);
  bool reportsValid() const;
};

struct ManagerConfig {
  // Bus
  uint32_t spiHz = 1000000;  // bring-up default; kSpiMaxHz is the limit
  int8_t sckPin = -1;        // -1 = core default for the board
  int8_t misoPin = -1;
  int8_t mosiPin = -1;

  // Output / logging
  IOutputSink* output = nullptr;
  ILogSink* log = nullptr;
  LogLevel logLevel = LogLevel::Info;
  uint16_t logCategories = logAllCategories;
  bool emitBinaryFrames = true;

  // Timing
  uint32_t statusIntervalMs = 1000;   // 0 disables periodic status
  uint32_t serviceBudgetUs = 2000;
  uint8_t maxTransfersPerVisit = 3;
  uint32_t busTimeoutUs = 2000;

  RecoveryPolicy recovery;
};

} // namespace bno085

#endif // BNO085_TYPES_H
