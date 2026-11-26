## Result

**Stage 2 hardware validation PASSED.** With PS0/WAKE on GPIO14, the board cold-boots, reads the Product ID, initializes SH-2, configures Rotation Vector at 100 Hz, and streams finite quaternions. Tests A–D all pass. No Phase 2, soak, or fault-injection work was started.

Two firmware defects were exposed only by the wired hardware and fixed before the final run: the fixed-length 200 µs wake pulse never asserted INT, and an immediate get-feature returned the stale 0 µs configuration.

## Final checkpoint

```
HEAD    2e9a2d81078c0c31ad3eff3c9e08fe1ec1d36c46  Update output.md
Branch  main
Status  8 modified files, nothing staged or committed
```

```
 examples/01_SingleSensor/01_SingleSensor.ino |  10 +-
 hardware_test_report.md                      | 367 +++++++++++++++++++++++----
 src/BNO085Sensor.cpp                         |  42 +--
 src/BNO085Sensor.h                           |   1 -
 src/hal/Esp32SpiHal.cpp                      |  42 ++-
 src/sh2/Sh2Client.cpp                        |  30 ++-
 src/sh2/Sh2Client.h                          |   3 +
 test/native/test_main.cpp                    |  38 ++-
 8 files changed, 441 insertions(+), 92 deletions(-)
```

The complete 8-file diff (825 lines) is also saved at `build/stage2/final_checkpoint.diff`. The functional diff is inline below; `hardware_test_report.md` is the documentation rewrite described at the end.

## Functional diff

```diff
diff --git a/examples/01_SingleSensor/01_SingleSensor.ino b/examples/01_SingleSensor/01_SingleSensor.ino
@@
-// This sketch is the Phase 1 acceptance sketch. It has been run on the Stage 1
-// bench (ESP32-S3 + BNO085 on COM11): the 276-byte advertisement and reset-complete
-// read correctly, but Product ID is blocked until PS0/WAKE is wired to a GPIO.
-// See hardware_test_report.md.
+// This sketch is the Phase 1 acceptance sketch. On the Stage 2 bench
+// (ESP32-S3 + BNO085 on COM11, PS0/WAKE on GPIO14) it reads the 276-byte
+// advertisement, identifies the part, configures Rotation Vector at 100 Hz and
+// streams quaternions. See hardware_test_report.md.
@@
-#define BNO085_WAKE_PIN -1  // PS0/WAKE: wire to a GPIO for SPI; -1 skips wake
+#define BNO085_WAKE_PIN 14  // PS0/WAKE on GPIO14; -1 if not wired
```

```diff
diff --git a/src/BNO085Sensor.cpp b/src/BNO085Sensor.cpp
@@ namespace {
 constexpr uint32_t kRetryIntervalUs = 10000;
+// The hub emits its Get Feature Response only after the new report period has
+// been applied. Measured on the bench at ~120 ms when the RV starts at 100 Hz,
+// so the command timeout (100 ms) is not enough for this exchange.
+constexpr uint32_t kFeatureApplyTimeoutUs = 250000;
 constexpr uint32_t kSilenceGraceUs = 50000;
@@
-      setFeatureSent_(false), getFeatureSent_(false), intPending_(false),
+      setFeatureSent_(false), intPending_(false),
@@
   bootEpochBaseline_ = sh2_.epoch();
   setFeatureSent_ = false;
-  getFeatureSent_ = false;
   intPending_ = false;
@@
     case StageIdentify:
       if (sh2_.productInfo().valid) {
+        diag_.product = sh2_.productInfo();
         stage_ = StageSetFeature;
@@
       if (!setFeatureSent_) {
         setFeatureSent_ = sh2_.setFeature(...);
         if (!setFeatureSent_) {
           nextAttemptUs_ = nowUs + kRetryIntervalUs;
+        } else {
+          // Datasheet 1.3.5: the hub answers a Set Feature with a Get Feature
+          // Response once the new period is applied. An explicit request sent
+          // immediately is answered from the old config (a zero interval).
+          sh2_.expectFeatureResponse(..., nowUs, kFeatureApplyTimeoutUs);
         }
         break;
       }
       if (sh2_.cmdState() == Sh2Client::CmdState::Ok) {
         stage_ = StageGetFeature;
         diag_.initStage = static_cast<uint8_t>(stage_);
-        stageDeadlineUs_ = nowUs + commandTimeoutUs();
+        stageDeadlineUs_ = nowUs + kFeatureApplyTimeoutUs;
         nextAttemptUs_ = nowUs;
-        getFeatureSent_ = false;
@@
       } else if (!sh2_.busy() && reached(nowUs, nextAttemptUs_)) {
         setFeatureSent_ = sh2_.setFeature(...);
+        if (setFeatureSent_) {
+          sh2_.expectFeatureResponse(..., nowUs, kFeatureApplyTimeoutUs);
+        }
         nextAttemptUs_ = nowUs + kRetryIntervalUs;
@@
     case StageGetFeature:
-      if (!getFeatureSent_) {
-        if (reached(nowUs, nextAttemptUs_)) {
-          getFeatureSent_ = sh2_.getFeature(...);
-          nextAttemptUs_ = nowUs + kRetryIntervalUs;
-        }
-        if (!getFeatureSent_ && reached(nowUs, stageDeadlineUs_)) {
-          failRecoverable(ErrorCode::Sh2Timeout);
-        }
-        break;
-      }
       if (sh2_.featureState() == Sh2Client::CmdState::Ok) {
@@
   diag_.initStage = static_cast<uint8_t>(stage_);
   setFeatureSent_ = false;
-  getFeatureSent_ = false;
   stageDeadlineUs_ = hal_.timeUs() + commandTimeoutUs();
@@
 void Sensor::failRecoverable(ErrorCode code) {
   if (diag_.recoveryFailures < 0xFFFFu) ++diag_.recoveryFailures;
+  if (log_.enabled(LogLevel::Info, LogInit)) {
+    log_.log(LogLevel::Info, LogInit,
+             "s%u recoverable error stage=%u cmd=%u feat=%u setSent=%u busy=%u",
+             index_, static_cast<unsigned>(stage_),
+             static_cast<unsigned>(sh2_.cmdState()),
+             static_cast<unsigned>(sh2_.featureState()),
+             setFeatureSent_ ? 1u : 0u, sh2_.busy() ? 1u : 0u);
+  }
```

```diff
diff --git a/src/BNO085Sensor.h b/src/BNO085Sensor.h
@@
   uint32_t bootEpochBaseline_;
   bool setFeatureSent_;
-  bool getFeatureSent_;
```

```diff
diff --git a/src/hal/Esp32SpiHal.cpp b/src/hal/Esp32SpiHal.cpp
@@ namespace {
-constexpr uint32_t kWakePulseUs = 200;   // datasheet: twk typ 150 us
+// Datasheet twk is 150 us typical. Hold WAKE low while waiting for INT instead
+// of a fixed-length pulse: the hub asserts INT when it is awake, and a pulse
+// that ends before that point is simply ignored.
 constexpr uint32_t kWakeWaitUs = 2000;   // bounded wait for INT after wake
@@
 Status Esp32SpiHal::begin() {
+  // The HAL object is normally constructed before setup() fills the manager
+  // config, so the constructor saw a null sink. Bind the logger here, once the
+  // caller's configuration (and its sink) is final.
+  log_.attach(manager_->log);
+  log_.setLevel(manager_->logLevel);
+  log_.setCategories(manager_->logCategories);
+
   const Status busStatus = ...
@@ bool Esp32SpiHal::prepareWrite() {
   if (config_->wakePin < 0) return true;
-  if (intAsserted()) return true;
+  if (intAsserted()) {
+    if (log_.enabled(LogLevel::Info, LogSpi)) {
+      log_.log(LogLevel::Info, LogSpi, "write: INT already low, no wake pulse");
+    }
+    return true;
+  }
 
   digitalWrite(config_->wakePin, LOW);
-  delayMicroseconds(kWakePulseUs);
-  digitalWrite(config_->wakePin, HIGH);
+  // A pad still reading high is tied to 3V3 somewhere else and the wake
+  // request cannot reach the device.
+  const int readback = digitalRead(config_->wakePin);
 
   const uint32_t start = micros();
   while (static_cast<uint32_t>(micros() - start) < kWakeWaitUs) {
-    if (intAsserted()) return true;
+    if (intAsserted()) {
+      const uint32_t waited = micros() - start;
+      digitalWrite(config_->wakePin, HIGH);
+      if (log_.enabled(LogLevel::Debug, LogSpi)) {
+        log_.log(LogLevel::Debug, LogSpi, "wake: INT after %luus (pad=%d)",
+                 static_cast<unsigned long>(waited), readback);
+      }
+      return true;
+    }
   }
 
+  digitalWrite(config_->wakePin, HIGH);
   if (log_.enabled(LogLevel::Warn, LogSpi)) {
-    log_.log(LogLevel::Warn, LogSpi, "wake pulse did not assert INT within %uus",
-             static_cast<unsigned>(kWakeWaitUs));
+    log_.log(LogLevel::Warn, LogSpi,
+             "wake: no INT within %uus (pad readback while low=%d, pin %d)",
+             static_cast<unsigned>(kWakeWaitUs), readback,
+             static_cast<int>(config_->wakePin));
   }
   return true;
 }
```

```diff
diff --git a/src/sh2/Sh2Client.cpp b/src/sh2/Sh2Client.cpp
@@ void Sh2Client::handleGetFeatureResponse(const uint8_t* report) {
   if (log_) {
-    log_->log(LogLevel::Info, LogSh2, "s%u report 0x%02X granted interval %lu us",
+    log_->hex(LogLevel::Info, LogSh2, "get-feature", report,
+              sh2::kGetFeatureRespLen);
+    log_->log(LogLevel::Info, LogSh2,
+              "s%u report 0x%02X granted interval %lu us (flags=0x%02X)",
               sensorId_, static_cast<unsigned>(featureId_),
-              static_cast<unsigned long>(grantedIntervalUs_));
+              static_cast<unsigned long>(grantedIntervalUs_),
+              static_cast<unsigned>(report[2]));
   }
 }
@@ bool Sh2Client::setFeature(...) {
   if (log_) {
+    log_->hex(LogLevel::Info, LogSh2, "set-feature-tx", payload, sizeof(payload));
     log_->log(LogLevel::Info, LogSh2, "s%u -> set feature 0x%02X @ %lu us", ...);
   }
@@
+bool Sh2Client::expectFeatureResponse(uint8_t reportId, uint32_t nowUs,
+                                      uint32_t windowUs) {
+  if (transport_ == nullptr) return false;
+  featurePending_ = true;
+  featureId_ = reportId;
+  featureDeadlineUs_ = nowUs + windowUs;
+  featureState_ = CmdState::Pending;
+  if (log_) {
+    log_->log(LogLevel::Info, LogSh2, "s%u waiting for get-feature response 0x%02X",
+              sensorId_, static_cast<unsigned>(reportId));
+  }
+  return true;
+}
+
 bool Sh2Client::getFeature(uint8_t reportId, uint32_t nowUs) {
   ...
   if (log_) {
-    log_->log(LogLevel::Debug, LogSh2, "s%u -> get feature 0x%02X", ...);
+    log_->log(LogLevel::Info, LogSh2, "s%u -> get feature 0x%02X", ...);
   }
@@ void Sh2Client::pollTimeouts(uint32_t nowUs) {
     if (log_) {
       log_->log(LogLevel::Error, LogSh2,
-                "s%u get-feature 0x%02X timed out after %u ms", ...);
+                "s%u get-feature 0x%02X response timed out", ...);
     }
```

```diff
diff --git a/src/sh2/Sh2Client.h b/src/sh2/Sh2Client.h
@@
   bool requestProductIds(uint32_t nowUs);
   bool setFeature(uint8_t reportId, uint32_t intervalUs, uint32_t nowUs);
+  // Arm the wait for the Get Feature Response the hub emits once it has applied
+  // a Set Feature (datasheet 1.3.5). No request is sent.
+  bool expectFeatureResponse(uint8_t reportId, uint32_t nowUs, uint32_t windowUs);
   bool getFeature(uint8_t reportId, uint32_t nowUs);
```

```diff
diff --git a/test/native/test_main.cpp b/test/native/test_main.cpp
@@ void driveSensorToStreaming(SensorFixture& fx) {
   fx.sensor.update();  // set-feature (completes on send)
   fx.sensor.update();  // -> StageGetFeature
-  fx.sensor.update();  // get-feature request
+
+  // The hub emits the Get Feature Response itself once the new period is
+  // applied; the host does not request it.
   fx.hal.queuePacket(makePacket(2, 1, makeGetFeatureResponse(0x05, 10000)));
@@ void finishBootToStreaming(SensorFixture& fx) {
-  fx.sensor.update();
   fx.sensor.update();
   fx.sensor.update();
@@
+void testSh2ExpectFeatureResponse() {
+  Sh2Fixture fx;
+  CHECK(fx.sh2.expectFeatureResponse(0x05, 1000, 250000));
+  CHECK(fx.hal.sent.empty());  // waiting, not requesting
+  CHECK(fx.sh2.busy());
+  CHECK(fx.sh2.featureState() == bno085::Sh2Client::CmdState::Pending);
+  std::vector<uint8_t> response = makeGetFeatureResponse(0x05, 10000);
+  fx.sh2.handlePayload(2, response.data(),
+                       static_cast<uint16_t>(response.size()), 2000);
+  CHECK(fx.sh2.featureState() == bno085::Sh2Client::CmdState::Ok);
+  CHECK(fx.sh2.grantedIntervalUs() == 10000);
+  CHECK(!fx.sh2.busy());
+}
+
+void testSh2ExpectFeatureResponseTimeout() {
+  Sh2Fixture fx;
+  CHECK(fx.sh2.expectFeatureResponse(0x05, 1000, 5000));
+  fx.sh2.pollTimeouts(5999);
+  CHECK(fx.sh2.featureState() == bno085::Sh2Client::CmdState::Pending);
+  fx.sh2.pollTimeouts(6000);
+  CHECK(fx.sh2.featureState() == bno085::Sh2Client::CmdState::Timeout);
+  CHECK(!fx.sh2.busy());
+  CHECK(fx.sh2.stats().timeouts == 1);
+}
@@ void testSensorInitializationSequence() {
-  fx.sensor.update();  // sends get-feature
-  CHECK(fx.hal.sent.size() == 3);
-  CHECK(fx.hal.sent[2][4] == bno085::sh2::kGetFeatureReq);
+  // The host waits for the hub's own Get Feature Response; it must not send a
+  // second request whose answer would still describe the old configuration.
+  CHECK(fx.hal.sent.size() == 2);
@@ int main() {
       {"sh2_set_feature_bytes", testSh2SetFeatureBytes},
+      {"sh2_expect_feature_response", testSh2ExpectFeatureResponse},
+      {"sh2_expect_feature_timeout", testSh2ExpectFeatureResponseTimeout},
       {"sh2_rotation_vector_decode", testSh2RotationVectorDecode},
```

The report diff is prose: it updates the verdict, §9.4 resolution, §11–12 pass/fail, and replaces §13–14 with the Stage 2 wiring, four fixes, raw capture, Test A–D table, rate analysis, remaining issues, and artifacts.

## Commands and results

```
> cmd /c test\native\build_msvc.bat
BNO085Multi host tests (63 cases)
449 checks, 0 failures
RESULT: PASS

> cmd /c build\analyze_msvc.bat
exit 0, 0 warnings

> arduino-cli compile --fqbn esp32:esp32:esp32s3 --library . ^
    --build-path build\stage2-final --warnings all examples\01_SingleSensor
Sketch uses 327206 bytes (24%) of program storage space.
Global variables use 24640 bytes (7%) of dynamic memory.

> arduino-cli upload -p COM11 --fqbn esp32:esp32:esp32s3 ^
    --input-dir build\stage2-final examples\01_SingleSensor
Verifying written data...
Hash of data verified.
Hard resetting via RTS pin...
New upload port: COM11 (serial)

> powershell -ExecutionPolicy Bypass -File ^
    %TEMP%\opencode\serial_capture.ps1 -Port COM11 -Baud 115200 -Seconds 18 ^
    -Out build\stage2\serial_final_clean.log ^
    -SendSchedule "12.0|d;13.0|q" -Reset
captured 1780 lines in 18.04s
```

## Raw COM11 output (cold boot, excerpt)

```
   0.006  Build:Mar 27 2021
   0.009  rst:0x1 (POWERON),boot:0x2a (SPI_FAST_FLASH_BOOT)
   0.281  [I] s0 begin cs=10 int=9 rst=8 wake=14 rate=10000us
   0.286  [I] s0 reset pulse on pin 8 (attempt 0)
   0.286  [I] s0 boot wait, budget 300ms
   0.444  [I] s0 reset complete (epoch 1)
   0.449  [I] s0 device reset complete, identifying
   0.454  [I] s0 product id part=10004148 build=6 fw=3.2.13 resetCause=4
   0.454  [I] s0 configuring rotation vector
   0.465  [I] s0 -> set feature 0x05 @ 10000 us
   0.467  [I] s0 waiting for get-feature response 0x05
   0.590  [I] get-feature [17] FC 05 00 00 00 10 27 00 00 00 00 00 00 00 00 00 00
   0.596  [I] s0 report 0x05 granted interval 10000 us (flags=0x00)
   0.596  [I] s0 streaming at 10000us (requested 10000us)
   0.607  RV #1 seq=0 acc=unreliable q=0.96704 0.01099 -0.00537 -0.25439 tMeas=507801 dt=3600us
----- HOST SENT 'd' at t=12.011s -----
  12.025  state=streaming lastError=none stage=5
  12.026  product=0x0098A6B4 fw=3.2.13  granted=10000us
  12.031  packets=1148 tx=2 bytes=26660 spiErrors=0 busTimeouts=0
  12.036  short=0 badChan=0 badFrag=0 oversize=0 seqGaps=0 spurious=0
  12.042  sh2Timeouts=0 badResp=0 unknown=0 dups=0 gaps=0
  12.047  resets=1 ints=1147 reports=1142 lastReport=11940885us maxService=12578us
  12.053  recovery attempts=0 successes=0 failures=0 terminal=0 reason=none
  12.058  output enabled=0 written=0 suppressed=0 noCap=0 partial=0 failed=0 dropped=0 lastError=none
----- HOST SENT 'q' at t=13.009s -----
  13.015  q w=0.967041 x=0.011475 y=-0.005920 z=-0.254395 acc=unreliable
  18.007  RV #1737 seq=201 acc=unreliable q=0.96704 0.01166 -0.00580 -0.25433 tMeas=17929436 dt=2300us
```

Full log: `build/stage2/serial_final_clean.log`.

## Hardware tests

| Test | Requirement | Result |
|---|---|---|
| **A. Boot and WAKE** | Reset, `wake=14`, hub accepts host writes | **PASS** — pad read back 0 while WAKE low; INT asserted 548–876 µs after WAKE went low in the diagnostic runs |
| **B. SHTP, Product ID, SH-2 init** | Advertisement, Product ID, reset-complete, streaming | **PASS** — `reset complete (epoch 1)`; `part=10004148 build=6 fw=3.2.13`; begin→streaming 315 ms; `resets=1`, no recovery |
| **C. RV configuration** | Set 10 000 µs and confirm the applied period | **PASS** — set-feature at 10 000 µs; automatic `FC ... 10 27 ...` = 10 000 µs; `granted interval 10000 us`; `streaming at 10000us` |
| **D. Live reports** | 100 Hz quaternions, plausible timestamps | **PASS** — 1737 reports / 17.422 s = 99.65 Hz; 99.82 Hz steady; all finite; `dt` 2.0–4.3 ms steady |

## Stream measurements

- Steady window 0.7–11.9 s: 1118 reports over 11.19 s = **99.82 Hz**; `tMeas` deltas 8.25–11.74 ms, average 10.02 ms, no sequence gaps.
- Overall: **99.65 Hz**. Startup jitter settles by RV #9.
- Exactly one report was lost at the deliberate 50 ms `d` print (`seq 118→120` at 12.075 s). This is host back-pressure, not a link error; all SHTP error counters remained zero.
- `tx=2` confirms only the Product ID and Set Feature requests were sent; the applied rate came from the hub's automatic Get Feature Response.
- All counters clean: `spiErrors=0 busTimeouts=0 badFrag=0 oversize=0 seqGaps=0 spurious=0 sh2Timeouts=0 badResp=0 unknown=0 dups=0 gaps=0`.

## Remaining issues

1. **RV accuracy stayed `unreliable`** for the whole stationary capture. Quaternions are finite and stable; motion/calibration behavior was not tested.
2. **No explicit get-feature fallback** if the hub's automatic 0xFC is lost; the 250 ms window expires and recovery starts.
3. **A ~50 ms blocking host print drops one 100 Hz sample.** The text log runs near 70% of 115 200 baud; production should use the binary output and a non-blocking sink.
4. **Not tested:** soak, fault injection, output loss under load, recovery under load, multi-sensor, RV accuracy after motion.
5. Stage 1 caveats remain: no continuation assembly, `/WX` not enabled, ASan unavailable.

## Report and artifacts

- `hardware_test_report.md` now documents the GPIO14 wiring, electrical checks, firmware identity, build/flash, raw output, Product ID, SH-2 initialization, RV configuration, stream rate, and remaining issues.
- `build/stage2-final/` — flashed final build (`01_SingleSensor.ino.bin`, 327 360 B)
- `build/stage2/serial_final_clean.log` — final 18 s capture
- `build/stage2/serial_diag*.log` — diagnostic iterations
- `build/stage2/final_checkpoint.diff` — complete 8-file diff
- Nothing was staged, committed, pushed, reset, or rewritten.
