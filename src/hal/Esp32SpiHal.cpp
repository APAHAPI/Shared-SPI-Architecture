#include "hal/Esp32SpiHal.h"

namespace bno085 {

namespace {
constexpr uint32_t kWakePulseUs = 200;   // datasheet: twk typ 150 us
constexpr uint32_t kWakeWaitUs = 2000;   // bounded wait for INT after wake
constexpr uint16_t kResetLowUs = 10;     // datasheet minimum is 10 ns
constexpr uint16_t kResetHoldMs = 1;
} // namespace

Esp32SpiHal::Esp32SpiHal(SpiBusManager& bus, SPIClass* spi,
                         const ManagerConfig& managerConfig,
                         const SensorConfig& sensorConfig)
    : bus_(bus), spi_(spi), manager_(&managerConfig), config_(&sensorConfig),
      log_(), transfers_(0), begun_(false) {
  log_.attach(managerConfig.log);
  log_.setLevel(managerConfig.logLevel);
  log_.setCategories(managerConfig.logCategories);
}

Status Esp32SpiHal::begin() {
  const Status busStatus =
      bus_.begin(spi_, manager_->sckPin, manager_->misoPin, manager_->mosiPin,
                 manager_->spiHz, manager_->busTimeoutUs);
  if (!busStatus.ok()) {
    if (log_.enabled(LogLevel::Error, LogInit)) {
      log_.log(LogLevel::Error, LogInit, "bus begin failed: %s", busStatus.str());
    }
    return busStatus;
  }

  const SensorConfig& cfg = *config_;

  if (cfg.csPin < 0) {
    return Status{Status::InvalidConfig, 0};
  }
  pinMode(cfg.csPin, OUTPUT);
  digitalWrite(cfg.csPin, HIGH);  // deassert before any device is reset

  if (cfg.intPin >= 0) {
    pinMode(cfg.intPin, INPUT_PULLUP);
  }
  if (cfg.resetPin >= 0) {
    pinMode(cfg.resetPin, OUTPUT);
    digitalWrite(cfg.resetPin, HIGH);
  }
  if (cfg.wakePin >= 0) {
    pinMode(cfg.wakePin, OUTPUT);
    digitalWrite(cfg.wakePin, HIGH);
  }

  begun_ = true;
  return Status{Status::Ok, 0};
}

void Esp32SpiHal::end() {
  if (config_->csPin >= 0) {
    digitalWrite(config_->csPin, HIGH);
  }
  begun_ = false;
}

bool Esp32SpiHal::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
  if (!begun_) return false;

  if (log_.enabled(LogLevel::Trace, LogSpi) && tx != nullptr) {
    log_.hex(LogLevel::Trace, LogSpi, "spi-tx", tx, len);
  }

  const bool ok = bus_.transfer(config_->csPin, tx, rx, len);
  if (!ok) {
    if (log_.enabled(LogLevel::Error, LogSpi)) {
      log_.log(LogLevel::Error, LogSpi, "transfer failed (cs=%d, len=%u)",
               static_cast<int>(config_->csPin), static_cast<unsigned>(len));
    }
    return false;
  }

  ++transfers_;
  if (log_.enabled(LogLevel::Trace, LogCs)) {
    log_.log(LogLevel::Trace, LogCs, "cs=%d %u bytes",
             static_cast<int>(config_->csPin), static_cast<unsigned>(len));
  }
  if (log_.enabled(LogLevel::Trace, LogSpi) && rx != nullptr) {
    log_.hex(LogLevel::Trace, LogSpi, "spi-rx", rx, len);
  }
  return true;
}

bool Esp32SpiHal::intAsserted() {
  // With no INT pin the sensor is polled on every update.
  if (config_->intPin < 0) return true;
  return digitalRead(config_->intPin) == LOW;
}

bool Esp32SpiHal::prepareWrite() {
  if (config_->wakePin < 0) return true;
  if (intAsserted()) return true;

  digitalWrite(config_->wakePin, LOW);
  delayMicroseconds(kWakePulseUs);
  digitalWrite(config_->wakePin, HIGH);

  const uint32_t start = micros();
  while (static_cast<uint32_t>(micros() - start) < kWakeWaitUs) {
    if (intAsserted()) return true;
  }

  // The write is still attempted: several hosts succeed without WAKE.
  if (log_.enabled(LogLevel::Warn, LogSpi)) {
    log_.log(LogLevel::Warn, LogSpi, "wake pulse did not assert INT within %uus",
             static_cast<unsigned>(kWakeWaitUs));
  }
  return true;
}

void Esp32SpiHal::pulseReset() {
  if (config_->resetPin < 0) return;
  digitalWrite(config_->resetPin, LOW);
  delayMicroseconds(kResetLowUs);
  delay(kResetHoldMs);
  digitalWrite(config_->resetPin, HIGH);
  if (log_.enabled(LogLevel::Debug, LogReset)) {
    log_.log(LogLevel::Debug, LogReset, "reset pulse on pin %d",
             static_cast<int>(config_->resetPin));
  }
}

} // namespace bno085
