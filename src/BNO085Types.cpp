#include "BNO085Types.h"

namespace bno085 {

const char* reportName(Report report) {
  switch (report) {
    case Report::Accelerometer:             return "Accelerometer";
    case Report::GyroscopeCalibrated:       return "GyroscopeCalibrated";
    case Report::MagneticFieldCalibrated:   return "MagneticFieldCalibrated";
    case Report::LinearAcceleration:        return "LinearAcceleration";
    case Report::RotationVector:            return "RotationVector";
    case Report::Gravity:                   return "Gravity";
    case Report::GyroscopeUncalibrated:     return "GyroscopeUncalibrated";
    case Report::GameRotationVector:        return "GameRotationVector";
    case Report::GeomagneticRotationVector: return "GeomagneticRotationVector";
    case Report::Pressure:                  return "Pressure";
    case Report::AmbientLight:              return "AmbientLight";
    case Report::Humidity:                  return "Humidity";
    case Report::Temperature:               return "Temperature";
    case Report::MagneticFieldUncalibrated: return "MagneticFieldUncalibrated";
    case Report::StepCounter:               return "StepCounter";
    case Report::StabilityClassifier:       return "StabilityClassifier";
    case Report::RawAccelerometer:          return "RawAccelerometer";
    case Report::RawGyroscope:              return "RawGyroscope";
    case Report::RawMagnetometer:           return "RawMagnetometer";
  }
  return "Unknown";
}

const char* accuracyName(Accuracy accuracy) {
  switch (accuracy) {
    case Accuracy::Unreliable: return "unreliable";
    case Accuracy::Low:        return "low";
    case Accuracy::Medium:     return "medium";
    case Accuracy::High:       return "high";
  }
  return "unknown";
}

const char* stateName(State state) {
  switch (state) {
    case State::Unconfigured: return "unconfigured";
    case State::Booting:      return "booting";
    case State::Identifying:  return "identifying";
    case State::Configuring:  return "configuring";
    case State::Streaming:    return "streaming";
    case State::Degraded:     return "degraded";
    case State::Recovering:   return "recovering";
    case State::Failed:       return "failed";
    case State::Disabled:     return "disabled";
  }
  return "unknown";
}

const char* errorName(ErrorCode code) {
  switch (code) {
    case ErrorCode::None:              return "none";
    case ErrorCode::InvalidConfig:     return "invalid-config";
    case ErrorCode::DuplicatePin:      return "duplicate-pin";
    case ErrorCode::SpiRead:           return "spi-read";
    case ErrorCode::SpiWrite:          return "spi-write";
    case ErrorCode::ShtpBadFrame:      return "shtp-bad-frame";
    case ErrorCode::ShtpBadChannel:    return "shtp-bad-channel";
    case ErrorCode::ShtpSequenceGap:   return "shtp-sequence-gap";
    case ErrorCode::Sh2Timeout:        return "sh2-timeout";
    case ErrorCode::Sh2BadResponse:    return "sh2-bad-response";
    case ErrorCode::ProductIdMismatch: return "product-id-mismatch";
    case ErrorCode::BootTimeout:       return "boot-timeout";
    case ErrorCode::NoReports:         return "no-reports";
    case ErrorCode::BusTimeout:        return "bus-timeout";
    case ErrorCode::QueueOverflow:     return "queue-overflow";
    case ErrorCode::OutputOverflow:    return "output-overflow";
    case ErrorCode::GrvFallbackUsed:   return "grv-fallback-used";
    case ErrorCode::UnsupportedReport: return "unsupported-report";
  }
  return "unknown";
}

const char* Status::str() const {
  switch (code) {
    case Ok:                 return "ok";
    case InvalidConfig:      return "invalid-config";
    case DuplicatePin:       return "duplicate-pin";
    case NoSensors:          return "no-sensors";
    case NoMemory:           return "no-memory";
    case BusUnavailable:     return "bus-unavailable";
    case OutputUnavailable:  return "output-unavailable";
    case AlreadyStarted:     return "already-started";
    case SensorFailed:       return "sensor-failed";
  }
  return "unknown";
}

SensorConfig::SensorConfig()
    : csPin(-1), intPin(-1), resetPin(-1), wakePin(-1), resetGroup(0),
      report(Report::RotationVector), intervalUs(kDefaultIntervalUs),
      allowGrvFallback(false), extra(), extraIntervalUs(), extraCount(0) {}

SensorConfig::SensorConfig(int8_t cs, int8_t intr, int8_t rst)
    : csPin(cs), intPin(intr), resetPin(rst), wakePin(-1), resetGroup(0),
      report(Report::RotationVector), intervalUs(kDefaultIntervalUs),
      allowGrvFallback(false), extra(), extraIntervalUs(), extraCount(0) {}

SensorConfig& SensorConfig::enable(Report reportId, uint32_t interval) {
  if (reportId == report) {
    intervalUs = interval;
    return *this;
  }
  for (uint8_t i = 0; i < extraCount; ++i) {
    if (extra[i] == reportId) {
      extraIntervalUs[i] = interval;
      return *this;
    }
  }
  if (extraCount < kMaxExtraReports) {
    extra[extraCount] = reportId;
    extraIntervalUs[extraCount] = interval;
    ++extraCount;
  }
  return *this;
}

SensorConfig& SensorConfig::disable(Report reportId) {
  if (reportId == report) {
    return *this;  // the primary report is always enabled
  }
  for (uint8_t i = 0; i < extraCount; ++i) {
    if (extra[i] == reportId) {
      for (uint8_t j = static_cast<uint8_t>(i + 1); j < extraCount; ++j) {
        extra[j - 1] = extra[j];
        extraIntervalUs[j - 1] = extraIntervalUs[j];
      }
      --extraCount;
      return *this;
    }
  }
  return *this;
}

bool SensorConfig::reportsValid() const {
  if (intervalUs < kMinIntervalUs) return false;
  if (extraCount > kMaxExtraReports) return false;
  for (uint8_t i = 0; i < extraCount; ++i) {
    if (extraIntervalUs[i] < kMinIntervalUs) return false;
    if (extra[i] == report) return false;
  }
  return true;
}

} // namespace bno085
