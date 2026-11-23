#include "hal/SpiBusManager.h"

#include <cstring>

namespace bno085 {

SpiBusManager::SpiBusManager()
    : spi_(nullptr), settings_(1000000, MSBFIRST, SPI_MODE3), begun_(false),
      transactionHeld_(false), activeCsPin_(-1), lockTimeoutUs_(2000),
      transactions_(0), lockTimeouts_(0), busErrors_(0), zeros_(), discard_()
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
  // Never leave a continuous read open: release the SPI transaction and the lock.
  endTransaction(-1);
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
  if (transactionHeld_) {
    // A one-shot transfer cannot be nested inside a continuous-CS read: it would
    // release CS in the middle of the other transfer.
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

  // Configure the bus BEFORE asserting CS. beginTransaction applies SPI_MODE3,
  // and on a cold boot the clock switches from the core's default idle-low to
  // mode-3 idle-high. Doing that with CS already low makes the device see a
  // spurious clock edge, which shifts the first header by one bit (0x0114
  // reads as 0x0228 = 552). The clock must be at its idle level first.
  spi_->beginTransaction(settings_);
  digitalWrite(csPin, LOW);
  spi_->transferBytes(const_cast<uint8_t*>(out), in,
                      static_cast<uint32_t>(len));
  digitalWrite(csPin, HIGH);
  spi_->endTransaction();

  ++transactions_;
  unlock();
  return true;
}

bool SpiBusManager::beginTransaction(int8_t csPin) {
  if (!begun_ || spi_ == nullptr || csPin < 0) {
    ++busErrors_;
    return false;
  }
  if (transactionHeld_) {
    ++busErrors_;
    return false;
  }

  if (!lock(lockTimeoutUs_)) {
    ++lockTimeouts_;
    return false;
  }

  // Same ordering rule as transfer(): settle the bus into SPI_MODE3 while CS is
  // still high, then assert CS. CS then stays low across every transferInTransaction
  // call until endTransaction().
  spi_->beginTransaction(settings_);
  digitalWrite(csPin, LOW);
  transactionHeld_ = true;
  activeCsPin_ = csPin;
  return true;
}

bool SpiBusManager::transferInTransaction(const uint8_t* tx, uint8_t* rx,
                                          size_t len) {
  if (!transactionHeld_ || spi_ == nullptr || len == 0) {
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

  spi_->transferBytes(const_cast<uint8_t*>(out), in,
                      static_cast<uint32_t>(len));
  return true;
}

void SpiBusManager::endTransaction(int8_t csPin) {
  if (!transactionHeld_) return;  // idempotent: safe on every exit path

  // Callers normally pass their CS pin; if the bus is torn down without one the
  // pin remembered by beginTransaction is used, so CS can never stay asserted.
  const int8_t pin = (csPin >= 0) ? csPin : activeCsPin_;

  // CS high first, so the clock cannot be reconfigured while the device is
  // selected, then release the peripheral and the lock.
  if (pin >= 0) {
    digitalWrite(pin, HIGH);
  }
  spi_->endTransaction();
  transactionHeld_ = false;
  activeCsPin_ = -1;
  ++transactions_;
  unlock();
}

} // namespace bno085
