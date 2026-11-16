#include "sh2/Sh2Codes.h"

namespace bno085 {
namespace sh2 {

namespace {
struct ReportLength {
  uint8_t id;
  uint8_t len;
};

// Sensor input reports plus the control-channel responses we parse. Unknown
// IDs terminate parsing of the current payload, because SH-2 reports are
// concatenated and an unknown length cannot be skipped safely.
const ReportLength kReportLengths[] = {
    // Sensor reports
    {0x01, 10},  // Accelerometer
    {0x02, 10},  // Gyroscope calibrated
    {0x03, 10},  // Magnetic field calibrated
    {0x04, 10},  // Linear acceleration
    {0x05, 14},  // Rotation vector
    {0x06, 10},  // Gravity
    {0x07, 16},  // Gyroscope uncalibrated
    {0x08, 12},  // Game rotation vector
    {0x09, 14},  // Geomagnetic rotation vector
    {0x0A, 8},   // Pressure
    {0x0B, 8},   // Ambient light
    {0x0C, 6},   // Humidity
    {0x0D, 6},   // Proximity
    {0x0E, 6},   // Temperature
    {0x0F, 16},  // Magnetic field uncalibrated
    {0x10, 5},   // Tap detector
    {0x11, 12},  // Step counter
    {0x12, 6},   // Significant motion
    {0x13, 6},   // Stability classifier
    {0x14, 16},  // Raw accelerometer
    {0x15, 16},  // Raw gyroscope
    {0x16, 16},  // Raw magnetometer
    {0x18, 8},   // Step detector
    {0x19, 6},   // Shake detector
    {0x1A, 6},   // Flip detector
    {0x1B, 8},   // Pickup detector
    {0x1C, 6},   // Stability detector
    {0x1E, 16},  // Personal activity classifier
    {0x1F, 6},   // Sleep detector
    {0x20, 6},   // Tilt detector
    {0x21, 6},   // Pocket detector
    {0x22, 6},   // Circle detector
    {0x23, 6},   // Heart rate monitor
    {0x28, 14},  // ARVR stabilized RV
    {0x29, 12},  // ARVR stabilized GRV
    {0x2A, 14},  // Gyro-integrated RV
    {0x2B, 6},   // iZRO motion request
    {0x2C, 24},  // Raw optical flow
    {0x2D, 60},  // Dead reckoning pose
    {0x2E, 12},  // Wheel encoder
    // Control channel responses
    {kCommandResp, kCommandRespLen},
    {0xF3, 16},  // FRS read response
    {0xF5, 4},   // FRS write response
    {kProdIdResp, kProdIdRespLen},
    {kBaseTimestampRef, kTimestampRefLen},
    {kTimestampRebase, kTimestampRefLen},
    {kGetFeatureResp, kGetFeatureRespLen},
};
} // namespace

uint8_t reportLength(uint8_t reportId) {
  for (const ReportLength& entry : kReportLengths) {
    if (entry.id == reportId) return entry.len;
  }
  return 0;
}

bool isSensorReport(uint8_t reportId) {
  return reportId >= 0x01 && reportId <= 0x2E;
}

} // namespace sh2
} // namespace bno085
