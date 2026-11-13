#ifndef BNO085_HAL_SPIBUSMANAGER_H
#define BNO085_HAL_SPIBUSMANAGER_H

#ifndef ARDUINO
#error "SpiBusManager requires an Arduino build (ESP32). Host builds use a mock IHal."
#endif

#include <Arduino.h>
#include <SPI.h>

#if defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

#include "BNO085Config.h"
#include "BNO085Types.h"

namespace bno085 {

// Owns the SPI peripheral, the transaction settings and the lock. Per-sensor
// state lives in the HAL, never here.
class SpiBusManager {
 public:
  SpiBusManager();

  // Idempotent: a second call from another sensor is a no-op.
  Status begin(SPIClass* spi, int8_t sckPin, int8_t misoPin, int8_t mosiPin,
               uint32_t clockHz, uint32_t lockTimeoutUs);
  void end();

  bool begun() const { return begun_; }
  void setClock(uint32_t clockHz);

  // One CS-asserted transaction. Always releases the lock and deasserts CS.
  bool transfer(int8_t csPin, const uint8_t* tx, uint8_t* rx, size_t len);

  bool lock(uint32_t timeoutUs);
  void unlock();

  uint32_t transactions() const { return transactions_; }
  uint32_t lockTimeouts() const { return lockTimeouts_; }
  uint32_t busErrors() const { return busErrors_; }

 private:
  SPIClass* spi_;
  SPISettings settings_;
  bool begun_;
  uint32_t lockTimeoutUs_;
  uint32_t transactions_;
  uint32_t lockTimeouts_;
  uint32_t busErrors_;

  uint8_t zeros_[BNO085_RX_TRANSFER_BYTES];   // MOSI filler for reads
  uint8_t discard_[BNO085_TX_TRANSFER_BYTES]; // MISO sink for writes

#if defined(ESP32)
  SemaphoreHandle_t mutex_;
#endif
};

} // namespace bno085

#endif // BNO085_HAL_SPIBUSMANAGER_H
