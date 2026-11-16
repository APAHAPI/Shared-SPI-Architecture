#ifndef BNO085_MULTI_H
#define BNO085_MULTI_H

// BNO085Multi — Phase 1 single-sensor baseline.
//
// Portable core: types, SHTP transport, SH-2 client, sensor state machine and
// the binary output encoder. None of it depends on Arduino, which is what makes
// the host tests possible.
#include "BNO085Config.h"
#include "BNO085Types.h"
#include "BNO085Sensor.h"
#include "diag/Logger.h"
#include "output/IOutputSink.h"

#if defined(ARDUINO)
#include "hal/Esp32SpiHal.h"
#include "hal/SpiBusManager.h"
#include "output/ArduinoSink.h"
#endif

namespace bno085 {
constexpr uint16_t kLibraryVersion = 0x0001;  // 0.1
}

#endif // BNO085_MULTI_H
