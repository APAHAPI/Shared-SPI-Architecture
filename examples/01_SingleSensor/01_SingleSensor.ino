// BNO085Multi — Phase 1: one BNO085 on SPI.
//
// Wiring (change these three pins to match your board):
//   CS   -> BNO085_CS_PIN
//   INT  -> BNO085_INT_PIN   (H_INTN, active low)
//   RST  -> BNO085_RESET_PIN (NRST, active low; -1 if not wired)
//   SCK/MISO/MOSI use the board's default SPI pins.
//
// Set BNO085_BINARY_OUTPUT to 1 to stream the compact binary protocol on
// Serial instead of human-readable lines. The two must not be mixed on one
// port: text would corrupt the frames.
//
// This sketch is the Phase 1 acceptance sketch. It has been compiled but not
// run against hardware in this repository.

#include <BNO085Multi.h>

// ---------------------------------------------------------------------------
#define BNO085_CS_PIN 10
#define BNO085_INT_PIN 9
#define BNO085_RESET_PIN 8
#define BNO085_WAKE_PIN -1  // PS0/WAKE, only needed if commands time out

#define BNO085_SPI_HZ 1000000  // raise to 3000000 once the wiring is proven

#define BNO085_BINARY_OUTPUT 0
#define BNO085_LOG_LEVEL bno085::LogLevel::Info
// ---------------------------------------------------------------------------

bno085::ManagerConfig managerConfig;
bno085::SensorConfig sensorConfig(BNO085_CS_PIN, BNO085_INT_PIN, BNO085_RESET_PIN);
bno085::SpiBusManager bus;
bno085::Esp32SpiHal hal(bus, &SPI, managerConfig, sensorConfig);
bno085::Sensor imu(hal);

bno085::StreamLogSink logSink(Serial);
#if BNO085_BINARY_OUTPUT
bno085::StreamSink outputSink(Serial);
#endif

uint32_t lastStatusMs = 0;
uint32_t reportCount = 0;

void printHelp() {
  Serial.println();
  Serial.println("commands: r = recover sensor, d = diagnostics, q = quaternion, h = help");
}

void printDiagnostics() {
  const bno085::Diagnostics& d = imu.diagnostics();
  Serial.printf("state=%s lastError=%s stage=%u\n", bno085::stateName(d.state),
                bno085::errorName(d.lastError), static_cast<unsigned>(d.initStage));
  Serial.printf("product=0x%08lX fw=%u.%u.%u  granted=%luus\n",
                static_cast<unsigned long>(d.product.partNumber),
                static_cast<unsigned>(d.product.major),
                static_cast<unsigned>(d.product.minor),
                static_cast<unsigned>(d.product.patch),
                static_cast<unsigned long>(d.grantedIntervalUs));
  Serial.printf("packets=%lu tx=%lu bytes=%lu spiErrors=%lu busTimeouts=%lu\n",
                static_cast<unsigned long>(d.shtpPackets),
                static_cast<unsigned long>(d.shtpTxPackets),
                static_cast<unsigned long>(d.shtpBytes),
                static_cast<unsigned long>(d.spiErrors),
                static_cast<unsigned long>(d.busTimeouts));
  Serial.printf("short=%u badChan=%u badFrag=%u oversize=%u seqGaps=%u spurious=%u\n",
                static_cast<unsigned>(d.shortFragments),
                static_cast<unsigned>(d.badChannels),
                static_cast<unsigned>(d.badFragments),
                static_cast<unsigned>(d.oversizePayloads),
                static_cast<unsigned>(d.sequenceGaps),
                static_cast<unsigned>(d.spuriousInterrupts));
  Serial.printf("sh2Timeouts=%u badResp=%u unknown=%u dups=%u gaps=%lu\n",
                static_cast<unsigned>(d.sh2Timeouts),
                static_cast<unsigned>(d.sh2BadResponses),
                static_cast<unsigned>(d.unknownReports),
                static_cast<unsigned>(d.duplicateReports),
                static_cast<unsigned long>(d.missedReports));
  Serial.printf("resets=%lu ints=%lu reports=%lu lastReport=%luus maxService=%luus\n",
                static_cast<unsigned long>(d.resetPulses),
                static_cast<unsigned long>(d.intAssertions),
                static_cast<unsigned long>(d.reportsDecoded),
                static_cast<unsigned long>(d.lastReportUs),
                static_cast<unsigned long>(d.maxServiceUs));
}

void handleSerial() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    switch (c) {
      case 'r': {
        const bno085::Status st = imu.recover();
        Serial.printf("recover -> %s\n", st.str());
        break;
      }
      case 'd':
        printDiagnostics();
        break;
      case 'q':
        if (!imu.quaternion().valid) {
          Serial.println("no quaternion yet");
        } else {
          const bno085::Quaternion& q = imu.quaternion();
          Serial.printf("q w=%f x=%f y=%f z=%f acc=%s\n", q.w, q.x, q.y, q.z,
                        bno085::accuracyName(q.accuracy));
        }
        break;
      case 'h':
        printHelp();
        break;
      default:
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  managerConfig.spiHz = BNO085_SPI_HZ;
  managerConfig.log = &logSink;
  managerConfig.logLevel = BNO085_LOG_LEVEL;
  managerConfig.logCategories = static_cast<uint16_t>(
      bno085::LogReset | bno085::LogInit | bno085::LogInt | bno085::LogSh2 |
      bno085::LogData | bno085::LogShtp | bno085::LogSpi);
  managerConfig.statusIntervalMs = 0;  // diagnostics printed on demand

#if BNO085_BINARY_OUTPUT
  managerConfig.output = &outputSink;
  managerConfig.emitBinaryFrames = true;
#else
  managerConfig.output = nullptr;
  managerConfig.emitBinaryFrames = false;
#endif

  sensorConfig.wakePin = BNO085_WAKE_PIN;

  const bno085::Status st = imu.begin(managerConfig, sensorConfig, 0);
  if (!st.ok()) {
    Serial.printf("begin failed: %s (sensor %u)\n", st.str(), st.sensorId);
    return;
  }

  Serial.println("BNO085Multi phase-1 baseline started");
  printHelp();
}

void loop() {
  imu.update();

#if !BNO085_BINARY_OUTPUT
  bno085::Quaternion q;
  if (imu.latest(q)) {
    ++reportCount;
    Serial.printf("RV #%lu seq=%u acc=%-10s q=%.5f %.5f %.5f %.5f tMeas=%lu dt=%lu\n",
                  static_cast<unsigned long>(reportCount),
                  static_cast<unsigned>(q.seq), bno085::accuracyName(q.accuracy),
                  static_cast<double>(q.w), static_cast<double>(q.x),
                  static_cast<double>(q.y), static_cast<double>(q.z),
                  static_cast<unsigned long>(q.tMeasUs),
                  static_cast<unsigned long>(q.tHostUs - q.tMeasUs));
  }

  handleSerial();

  if (millis() - lastStatusMs >= 2000) {
    lastStatusMs = millis();
    Serial.printf("[%s] reports=%lu errors=%lu maxService=%luus\n",
                  bno085::stateName(imu.state()),
                  static_cast<unsigned long>(imu.diagnostics().reportsDecoded),
                  static_cast<unsigned long>(imu.diagnostics().shtpErrors),
                  static_cast<unsigned long>(imu.diagnostics().maxServiceUs));
  }
#endif
}
