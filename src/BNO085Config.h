// BNO085Multi — compile-time limits and defaults.
// Every macro can be overridden with -D<NAME>=<value>.
#ifndef BNO085_CONFIG_H
#define BNO085_CONFIG_H

#include <cstdint>

#ifndef BNO085_MAX_SENSORS
#define BNO085_MAX_SENSORS 20
#endif

#ifndef BNO085_RX_TRANSFER_BYTES
#define BNO085_RX_TRANSFER_BYTES 256
#endif

#ifndef BNO085_RX_PAYLOAD_BYTES
#define BNO085_RX_PAYLOAD_BYTES 192
#endif

#ifndef BNO085_TX_TRANSFER_BYTES
#define BNO085_TX_TRANSFER_BYTES 64
#endif

#ifndef BNO085_MAX_EXTRA_REPORTS
#define BNO085_MAX_EXTRA_REPORTS 6
#endif

#ifndef BNO085_ENABLE_DIAGNOSTICS
#define BNO085_ENABLE_DIAGNOSTICS 1
#endif

#ifndef BNO085_ENABLE_LOGGING
#define BNO085_ENABLE_LOGGING 1
#endif

namespace bno085 {

constexpr uint8_t kMaxSensors          = BNO085_MAX_SENSORS;
constexpr uint16_t kRxTransferBytes    = BNO085_RX_TRANSFER_BYTES;
constexpr uint16_t kRxPayloadBytes     = BNO085_RX_PAYLOAD_BYTES;
constexpr uint16_t kTxTransferBytes    = BNO085_TX_TRANSFER_BYTES;
constexpr uint8_t kMaxExtraReports     = BNO085_MAX_EXTRA_REPORTS;

constexpr uint32_t kDefaultIntervalUs  = 10000;   // 100 Hz
constexpr uint32_t kMinIntervalUs      = 1000;    // 1 kHz upper bound guard
constexpr uint32_t kSpiMaxHz           = 3000000; // BNO08X datasheet maximum

// One SHTP transfer carries a 4-byte header plus cargo; a full transfer must fit
// in the transfer buffer, and a payload alone must fit in the payload buffer.
static_assert(BNO085_RX_PAYLOAD_BYTES + 4 <= BNO085_RX_TRANSFER_BYTES,
              "RX payload buffer must leave room for the SHTP header");
static_assert(BNO085_TX_TRANSFER_BYTES >= 8,
              "TX transfer buffer must hold a header plus at least one byte");
static_assert(BNO085_MAX_SENSORS >= 1 && BNO085_MAX_SENSORS <= 32,
              "sensor count must fit the 32-bit scheduler mask used from Phase 2");

} // namespace bno085

#endif // BNO085_CONFIG_H
