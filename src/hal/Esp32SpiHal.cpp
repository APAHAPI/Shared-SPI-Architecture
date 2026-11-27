#include "hal/Esp32SpiHal.h"

namespace bno085 {

namespace {
// Datasheet twk is 150 us typical. Hold WAKE low while waiting for INT instead
// of a fixed-length pulse: the hub asserts INT when it is awake, and a pulse
// that ends before that point is simply ignored.
constexpr uint32_t kWakeWaitUs = 2000;   // bounded wait for INT after wake
constexpr uint16_t kResetLowUs = 10;     // datasheet minimum is 10 ns
constexpr uint16_t kResetHoldMs = 1;
} // namespace

Esp32SpiHal::Esp32SpiHal(SpiBusManager& bus, SPIClass* spi,
                         const ManagerConfig& managerConfig,
                         const SensorConfig& sensorConfig)
    : bus_(bus), spi_(spi), manager_(&managerConfig), config_(&sensorConfig),
      log_(), transfers_(0), begun_(false), readActive_(false), readBytes_(0) {
  log_.attach(managerConfig.log);
  log_.setLevel(managerConfig.logLevel);
  log_.setCategories(managerConfig.logCategories);
}

Status Esp32SpiHal::begin() {
  // The HAL object is normally constructed before setup() fills the manager
  // config, so the constructor saw a null sink. Bind the logger here, once the
  // caller's configuration (and its sink) is final.
  log_.attach(manager_->log);
  log_.setLevel(manager_->logLevel);
  log_.setCategories(manager_->logCategories);

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
  endRead();
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

// Continuous-CS read. The header and every payload byte must be clocked inside
// one CS assertion: releasing CS mid-transfer makes the device drop the pending
// transfer, so the next read would return a header instead of the payload.
bool Esp32SpiHal::beginRead() {
  if (!begun_ || readActive_) return false;

  if (!bus_.beginTransaction(config_->csPin)) {
    if (log_.enabled(LogLevel::Error, LogSpi)) {
      log_.log(LogLevel::Error, LogSpi, "read start failed (cs=%d)",
               static_cast<int>(config_->csPin));
    }
    return false;
  }

  readActive_ = true;
  readBytes_ = 0;
  return true;
}

bool Esp32SpiHal::read(uint8_t* dst, size_t len) {
  if (!readActive_ || dst == nullptr || len == 0) return false;

  // Only SPI byte transfers happen here: no logging while CS is asserted.
  if (!bus_.transferInTransaction(nullptr, dst, len)) {
    return false;
  }

  readBytes_ += static_cast<uint32_t>(len);
  return true;
}

void Esp32SpiHal::endRead() {
  if (!readActive_) return;  // idempotent, so every exit path may call it

  bus_.endTransaction(config_->csPin);
  readActive_ = false;
  ++transfers_;

  if (log_.enabled(LogLevel::Trace, LogCs)) {
    log_.log(LogLevel::Trace, LogCs, "cs=%d %u bytes in one assertion",
             static_cast<int>(config_->csPin), static_cast<unsigned>(readBytes_));
  }
}

bool Esp32SpiHal::intAsserted() {
  // With no INT pin the sensor is polled on every update.
  if (config_->intPin < 0) return true;
  return digitalRead(config_->intPin) == LOW;
}

bool Esp32SpiHal::prepareWrite() {
  if (config_->wakePin < 0) return true;
  if (intAsserted()) {
    if (log_.enabled(LogLevel::Info, LogSpi)) {
      log_.log(LogLevel::Info, LogSpi, "write: INT already low, no wake pulse");
    }
    return true;
  }

  digitalWrite(config_->wakePin, LOW);
  // Read the pad back while it is driven low. A pad still reading high is tied
  // to 3V3 somewhere else and the wake request cannot reach the device.
  const int readback = digitalRead(config_->wakePin);

  const uint32_t start = micros();
  while (static_cast<uint32_t>(micros() - start) < kWakeWaitUs) {
    if (intAsserted()) {
      const uint32_t waited = micros() - start;
      digitalWrite(config_->wakePin, HIGH);
      // One line per host write: the pad readback proves WAKE was actually
      // driven low (no 3V3 tie), and the interval is the observed wake time.
      if (log_.enabled(LogLevel::Info, LogSpi)) {
        log_.log(LogLevel::Info, LogSpi, "wake: INT after %luus (pad=%d)",
                 static_cast<unsigned long>(waited), readback);
      }
      return true;
    }
  }

  digitalWrite(config_->wakePin, HIGH);
  // The write is still attempted: several hosts succeed without WAKE.
  if (log_.enabled(LogLevel::Warn, LogSpi)) {
    // pad readback = 1 while driven low means something else still holds the
    // line at 3V3, so the wake request never reaches the device.
    log_.log(LogLevel::Warn, LogSpi,
             "wake: no INT within %uus (pad readback while low=%d, pin %d)",
             static_cast<unsigned>(kWakeWaitUs), readback,
             static_cast<int>(config_->wakePin));
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
