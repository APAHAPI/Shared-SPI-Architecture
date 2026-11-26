// BNO085Multi — Phase 1: one BNO085 on SPI.
//
// Wiring (Phase 1 baseline; change to match your board):
//   SCK  -> BNO085_SCK_PIN   (12)
//   MISO -> BNO085_MISO_PIN  (13)
//   MOSI -> BNO085_MOSI_PIN  (11)
//   CS   -> BNO085_CS_PIN    (10)
//   INT  -> BNO085_INT_PIN   (9,  H_INTN, active low)
//   RST  -> BNO085_RESET_PIN (8,  NRST, active low; -1 if not wired)
//   PS0 and PS1 must be HIGH at reset for SPI mode. After reset PS0 is the WAKE
//   input: connect it to a GPIO (BNO085_WAKE_PIN). With PS0 tied HIGH the hub
//   stops accepting host writes as soon as it is idle, so Product ID and feature
//   configuration time out even though reads work.
//
// Set BNO085_BINARY_OUTPUT to 1 to stream the compact binary protocol on
// Serial instead of human-readable lines. The two must not be mixed on one
// port: text would corrupt the frames.
//
// This sketch is the Phase 1 acceptance sketch. On the Stage 2 bench
// (ESP32-S3 + BNO085 on COM11, PS0/WAKE on GPIO14) it reads the 276-byte
// advertisement, identifies the part, configures Rotation Vector at 100 Hz and
// streams quaternions. See hardware_test_report.md.
//
// Timestamp diagnostic (`dt=` on each RV line):
//   dt = tHost - tMeas, printed as a signed value. It is the interval between
//   the hub's estimated measurement instant and the moment the host first
//   observed the interrupt. It is therefore a coarse transport/scheduling
//   figure, not a sensor latency: it includes the hub delay-field
//   compensation, and tHost is captured by the polling loop (there is no ISR
//   in Phase 1), so it can lag the physical INT edge by up to one loop period.
//   A negative value means tMeas is later than tHost. That is not a huge
//   latency; it indicates clock-reference skew or an unaligned 0xFB base.
//   The value is valid across one 32-bit micros() rollover while the two
//   readings are less than 2^31 us (~35.8 min) apart.

#include <BNO085Multi.h>

// ---------------------------------------------------------------------------
#define BNO085_SCK_PIN 12
#define BNO085_MISO_PIN 13
#define BNO085_MOSI_PIN 11
#define BNO085_CS_PIN 10
#define BNO085_INT_PIN 9
#define BNO085_RESET_PIN 8
#define BNO085_WAKE_PIN 14  // PS0/WAKE on GPIO14; -1 if not wired

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
  Serial.printf("recovery attempts=%u successes=%u failures=%u terminal=%u reason=%s\n",
                static_cast<unsigned>(d.recoveryAttempts),
                static_cast<unsigned>(d.recoverySuccesses),
                static_cast<unsigned>(d.recoveryFailures),
                d.recoveryTerminal ? 1u : 0u,
                bno085::errorName(d.lastRecoveryReason));
  Serial.printf("output enabled=%u written=%lu suppressed=%lu noCap=%lu partial=%lu failed=%lu dropped=%lu lastError=%s\n",
                d.outputEnabled ? 1u : 0u,
                static_cast<unsigned long>(d.framesWritten),
                static_cast<unsigned long>(d.framesSuppressed),
                static_cast<unsigned long>(d.framesNoCapacity),
                static_cast<unsigned long>(d.framesPartial),
                static_cast<unsigned long>(d.framesFailed),
                static_cast<unsigned long>(d.framesDropped),
                bno085::errorName(d.lastOutputError));
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

  managerConfig.sckPin = BNO085_SCK_PIN;
  managerConfig.misoPin = BNO085_MISO_PIN;
  managerConfig.mosiPin = BNO085_MOSI_PIN;
  managerConfig.spiHz = BNO085_SPI_HZ;
#if BNO085_BINARY_OUTPUT
  // Log text and binary frames would share Serial and corrupt each other, so
  // logging stays off on this port. Use diagnostics over the binary protocol
  // (SensorStatus) or a second UART for logs.
  managerConfig.log = nullptr;
  managerConfig.logLevel = bno085::LogLevel::Off;
#else
  managerConfig.log = &logSink;
  managerConfig.logLevel = BNO085_LOG_LEVEL;
  managerConfig.logCategories = static_cast<uint16_t>(
      bno085::LogReset | bno085::LogInit | bno085::LogInt | bno085::LogSh2 |
      bno085::LogData | bno085::LogShtp | bno085::LogSpi);
#endif
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
    const int32_t dtUs = bno085::timeDeltaUs(q.tHostUs, q.tMeasUs);
    Serial.printf("RV #%lu seq=%u acc=%-10s q=%.5f %.5f %.5f %.5f tMeas=%lu dt=%ldus%s\n",
                  static_cast<unsigned long>(reportCount),
                  static_cast<unsigned>(q.seq), bno085::accuracyName(q.accuracy),
                  static_cast<double>(q.w), static_cast<double>(q.x),
                  static_cast<double>(q.y), static_cast<double>(q.z),
                  static_cast<unsigned long>(q.tMeasUs),
                  static_cast<long>(dtUs),
                  (dtUs < 0) ? " (tMeas ahead of tHost)" : "");
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
