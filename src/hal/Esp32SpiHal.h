#ifndef BNO085_HAL_ESP32SPIHAL_H
#define BNO085_HAL_ESP32SPIHAL_H

#ifndef ARDUINO
#error "Esp32SpiHal requires an Arduino build (ESP32)."
#endif

#include <Arduino.h>
#include <SPI.h>

#include "BNO085Config.h"
#include "BNO085Types.h"
#include "diag/Logger.h"
#include "hal/IHal.h"
#include "hal/SpiBusManager.h"

namespace bno085 {

// ESP32-S3 SPI implementation of one sensor's hardware access. Pins come from
// SensorConfig; nothing is hardcoded here.
class Esp32SpiHal : public IHal {
 public:
  Esp32SpiHal(SpiBusManager& bus, SPIClass* spi, const ManagerConfig& managerConfig,
              const SensorConfig& sensorConfig);

  Status begin() override;
  void end() override;
  bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) override;
  bool intAsserted() override;
  bool prepareWrite() override;
  void pulseReset() override;
  uint32_t timeUs() override { return micros(); }
  uint32_t transferCount() const override { return transfers_; }

 private:
  SpiBusManager& bus_;
  SPIClass* spi_;
  const ManagerConfig* manager_;
  const SensorConfig* config_;
  Logger log_;
  uint32_t transfers_;
  bool begun_;
};

} // namespace bno085

#endif // BNO085_HAL_ESP32SPIHAL_H
