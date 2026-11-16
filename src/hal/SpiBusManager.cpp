#include "hal/SpiBusManager.h"

#include <cstring>

namespace bno085 {

SpiBusManager::SpiBusManager()
    : spi_(nullptr), settings_(1000000, MSBFIRST, SPI_MODE3), begun_(false),
      lockTimeoutUs_(2000), transactions_(0), lockTimeouts_(0), busErrors_(0),
      zeros_(), discard_()
#if defined(ESP32)
      , mutex_(nullptr)
#endif
{
}

Status SpiBusManager::begin(SPIClass* spi, int8_t sckPin, int8_t misoPin,
                            int8_t mosiPin, uint32_t clockHz,
                            uint32_t lockTimeoutUs) {
  if (begun_) return Status{Status::Ok, 0xFF};
  if (spi == nullptr || clockHz == 0 || clockHz > kSpiMaxHz) {
    return Status{Status::InvalidConfig, 0xFF};
  }

  spi_ = spi;
  lockTimeoutUs_ = lockTimeoutUs;
  settings_ = SPISettings(clockHz, MSBFIRST, SPI_MODE3);

  if (sckPin >= 0) {
    spi_->begin(static_cast<int8_t>(sckPin), static_cast<int8_t>(misoPin),
                static_cast<int8_t>(mosiPin), -1);
  } else {
    spi_->begin();
  }

#if defined(ESP32)
  mutex_ = xSemaphoreCreateMutex();
  if (mutex_ == nullptr) {
    return Status{Status::NoMemory, 0xFF};
  }
#endif

  begun_ = true;
  return Status{Status::Ok, 0xFF};
}

void SpiBusManager::end() {
#if defined(ESP32)
  if (mutex_ != nullptr) {
    vSemaphoreDelete(mutex_);
    mutex_ = nullptr;
  }
#endif
  begun_ = false;
  spi_ = nullptr;
}

void SpiBusManager::setClock(uint32_t clockHz) {
  if (clockHz == 0 || clockHz > kSpiMaxHz) return;
  settings_ = SPISettings(clockHz, MSBFIRST, SPI_MODE3);
}

bool SpiBusManager::lock(uint32_t timeoutUs) {
#if defined(ESP32)
  if (mutex_ == nullptr) return false;
  TickType_t ticks = pdMS_TO_TICKS((timeoutUs + 999u) / 1000u);
  if (ticks == 0) ticks = 1;
  return xSemaphoreTake(mutex_, ticks) == pdTRUE;
#else
  (void)timeoutUs;
  return true;
#endif
}

void SpiBusManager::unlock() {
#if defined(ESP32)
  if (mutex_ != nullptr) {
    xSemaphoreGive(mutex_);
  }
#endif
}

bool SpiBusManager::transfer(int8_t csPin, const uint8_t* tx, uint8_t* rx,
                             size_t len) {
  if (!begun_ || spi_ == nullptr || csPin < 0 || len == 0) {
    ++busErrors_;
    return false;
  }

  const uint8_t* out = (tx != nullptr) ? tx : zeros_;
  uint8_t* in = (rx != nullptr) ? rx : discard_;
  if ((tx == nullptr) && (len > sizeof(zeros_))) {
    ++busErrors_;
    return false;
  }
  if ((rx == nullptr) && (len > sizeof(discard_))) {
    ++busErrors_;
    return false;
  }

  // One sensor on the bus at a time, and no unbounded wait for the lock.
  if (!lock(lockTimeoutUs_)) {
    ++lockTimeouts_;
    return false;
  }

  digitalWrite(csPin, LOW);
  spi_->beginTransaction(settings_);
  spi_->transferBytes(const_cast<uint8_t*>(out), in,
                      static_cast<uint32_t>(len));
  spi_->endTransaction();
  digitalWrite(csPin, HIGH);

  ++transactions_;
  unlock();
  return true;
}

} // namespace bno085
