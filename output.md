# BNO085Multi — implementation trace and documentation-vs-code audit

**Purpose of this file.** Handoff document for another AI agent. It records what the code in this
repository actually does, traced from `setup()` through to a decoded Rotation Vector leaving the
binary output, and lists every place where the implementation differs from the documents in
`docs/`. It is analysis only: no source file was modified.

| Item | Value |
| --- | --- |
| Repository root | `C:\Users\Apahapi\Desktop\Source codes\Shared SPI Architecture` |
| Library name / version | `BNO085Multi` 0.1.0 (`library.properties`) |
| Analysed commit | `d040c8e` — "Drain oversized SHTP transfers completely" (`git log --oneline`) |
| Stage | Phase 1 — single-sensor baseline, ESP32-S3, Arduino core 3.3.12 |
| Analysis date | 2026-11-17 |
| Authoritative sources for this file | The source files themselves. Protocol comparisons were checked against the Hillcrest/CEVA reference `sh2.c` (the same upstream that `docs/UPSTREAM-AUDIT.md` analyses). |
| Files read | `README.md`; `docs/{ARCHITECTURE,API,PHASE1-NOTES,UPSTREAM-AUDIT}.md`; `src/BNO085Multi.h`, `BNO085Config.h`, `BNO085Types.{h,cpp}`, `BNO085Sensor.{h,cpp}`; `src/hal/{IHal.h,SpibusManager.*,Esp32SpiHal.*}`; `src/shtp/{ShtpProtocol.h,ShtpTransport.*}`; `src/sh2/{Sh2Codes.*,Sh2Client.*,Sh2Reports.*}`; `src/output/{OutputProtocol.h,IOutputSink.h,BinaryEncoder.*,ArduinoSink.h}`; `src/util/Crc16.*`; `src/diag/Logger.h`; `examples/01_SingleSensor/01_SingleSensor.ino`; `test/native/{test_main.cpp,mock_hal.h}` |

> Note for the next agent: the claims below are anchored to file + line numbers at commit
> `d040c8e`. Re-read the cited functions before editing; line numbers will drift after changes.

---

## 0. What this project is (one paragraph)

An Arduino library for an ESP32-S3 that drives 1–20 BNO085 IMUs on one shared SPI bus. The
BNO085 does its own sensor fusion; the firmware moves bytes, keeps per-device state, timestamps
reports and emits a CRC-framed binary stream. Phase 1 implements one sensor only, Rotation Vector
only, and has no `Manager`, no aggregation, no interrupt handler and no automatic recovery.

---

## 1. Object graph as actually built

There is no `Manager` class in `src/`. One `Sensor` owns one HAL reference and all per-device state:

```
StreamSink / NullOutputSink ──► BinaryEncoder (member of Sensor)
Sensor ──owns── ShtpTransport ──owns── rx/tx buffers, per-channel seq, reassembly
       └─owns── Sh2Client    ──owns── product, pending requests, deadlines, rebase, epoch
       └─ref─── IHal (Esp32SpiHal) ──ref── SpiBusManager (the only shared object)
```

Wiring happens in `Sensor::begin()` (`src/BNO085Sensor.cpp:22`):

- `shtp_.attach(&hal_, &log_, index_)` and `shtp_.setHandler(&Sensor::channelThunk, this)` — lines 58–59
- `sh2_.attach(&shtp_, &log_, index_)`, `setTimeoutMs(manager_.recovery.commandTimeoutMs)`,
  `setHandlers(...)` — lines 61–68
- `encoder_.attach(manager_.emitBinaryFrames ? output_ : nullptr, &log_)` — line 56
- `hal_.begin()` — line 70
- `encoder_.writeHello(1, 0x0001, hal_.timeUs())` — line 85 (sensor count is hardcoded to 1)
- `startBoot()` — line 94

---

## 2. Initialization, step by step

| Step | File / function | Caller → callee | What happens next |
| --- | --- | --- | --- |
| 1 | `Sensor::begin` (`BNO085Sensor.cpp:22–96`) | sketch → `Sensor` | Validates only `csPin >= 0`, `report == RotationVector`, `intervalUs >= kMinIntervalUs && reportsValid()`, `0 < spiHz <= 3 MHz` (lines 32–49). No duplicate-pin, sensor-count or `resetGroup` validation. |
| 2 | `Esp32SpiHal::begin` (`hal/Esp32SpiHal.cpp:22–55`) | Sensor → HAL → `SpiBusManager::begin` | Bus/driver init once (`SpiBusManager.cpp:17–45`), CS driven high, INT as `INPUT_PULLUP`, RST/WAKE driven high. |
| 3 | `BinaryEncoder::writeHello` (`output/BinaryEncoder.cpp:67–77`) | Sensor | First bytes on the sink: `A5 5A 01 7F 0A 00 <seq> …`; flags `0x0009` (binary, reception timestamps). |
| 4 | `Sensor::startBoot` (`104–137`) | Sensor | `shtp_.reset()` + `sh2_.reset()`, `stage_=StageBoot`, `state_=Booting`, `bootPacketBaseline_ = shtp_.stats().packets`, deadline `now + bootTimeoutMs` (default 300 ms). `hal_.pulseReset()` if `resetPin >= 0` (NRST low 10 µs, `delay(1)`, high). If no reset pin **and** recovering, `sh2_.sendReset()` on executable channel 1 with byte `0x01`. |
| 5 | `Sensor::update` → `serviceBus` → `ShtpTransport::service` | sketch loop | Any **complete** SHTP packet increments `stats_.packets`; `advanceInit` sees `packets > baseline` and moves to `StageIdentify` (`225–237`). The 552-byte advertisement takes the `Oversize` path and does **not** increment `packets`, so it is the following reset-complete packet that advances the stage. |
| 6 | `StageIdentify` (`240–256`) | `advanceInit` → `Sh2Client::requestProductIds` | Sends `F9 00` on control channel every 10 ms while not busy; advances as soon as `productInfo().valid` is set by the **first** `0xF8`. |
| 7 | `StageSetFeature` (`258–282`) | → `Sh2Client::setFeature` | Sends one 17-byte `0xFD` payload; waits for `cmdState() == Ok` (from an `0xF1` response). On Ok → `StageGetFeature`. See §4 for the wire bytes and a serious defect. |
| 8 | `StageGetFeature` (`284–298`) | → `Sh2Client::getFeature` | Sends `FE 05`; a `0xFC` response stores `grantedIntervalUs_` and sets `Ok`; `enterStreaming()` (`309–327`) sets `State::Streaming`, emits `INIT_RESULT` (result 0). |

`Sensor::update()` order (`139–163`): `serviceBus`, then `Sh2Client::pollTimeouts`, then
`advanceInit`, then `checkSilence`, then `emitStatusIfDue`. Responses therefore win ties against
deadlines in the same update.

---

## 3. INT handling and SHTP reading (as implemented)

### 3.1 INT: polling, no ISR

`Esp32SpiHal::intAsserted()` (`Esp32SpiHal.cpp:91–95`) is a `digitalRead(intPin) == LOW`. With
`intPin < 0` it returns `true` so the sensor is polled every `update()`. There is **no**
`attachInterrupt`, no ISR, no pending mask.

`Sensor::serviceBus` (`165–220`):

1. Poll `intAsserted()`. On first observation while `!intPending_`, latch `intPending_ = true`
   and `intEdgeUs_ = nowUs`; `++diag_.intAssertions` (166–174).
2. The latch is **not** re-stamped while it stays pending, so every packet drained in that visit
   carries the time of first observation.
3. Loop while `intPending_`, `visits < manager_.maxTransfersPerVisit` (default 3) and
   `timeUs() - nowUs <= serviceBudgetUs` (default 2 ms), calling `shtp_.service(intEdgeUs_)`
   (188–217).
4. After each call the latch is cleared if INT reads high. `NoData`/`Spurious`/`BusError` also
   clear it; `Malformed`/`Oversize` leave it set so the loop can continue draining.
5. `mirrorStats()` copies transport/SH-2 counters into `Diagnostics`.

Consequence: `Quaternion::tHostUs` is the **poll-observation** time, not the hardware H_INTN
assertion time. Under a busy `loop()`, it can be late by up to one loop period.

### 3.2 SHTP read sequence

`ShtpTransport::service` (`shtp/ShtpTransport.cpp:77–279`):

1. 4-byte peek read: `readRaw(peek, 4)` (81–86). The peek is non-consuming; the device
   re-presents the whole transfer.
2. Parse `rawLength` (bit 15 = continuation), `channel = peek[2]`, `sequence = peek[3]` (88–94).
3. Reject/consume:
   - `length == 0` → `Spurious` (104–108)
   - `0xFFFF` reserved → count `badFragments`, `discard(length)` (110–116)
   - `length < 4` → `shortFragments` + discard (118–124)
   - `channel >= 8` → `badChannels` + discard (126–132)
4. Sequence tracking is diagnostic only (134–144).
5. `length > 256` (transfer buffer) → `oversizePayloads++`, drain exactly `length` bytes in
   ≤256-byte chunks following continuation headers, bounded to 8 fragments, return `Oversize`
   (146–205). This is the fix validated on hardware.
6. Otherwise read the full `length` bytes (header repeated + payload) into `rxTransfer_` (207),
   increment `packets`/`bytes`, record `lastChannel/lastLength/lastTimestampUs` (213–217).
7. Fragment assembly into `rxPayload_` (192 B): a continuation must match an in-progress payload;
   a new non-continuation transfer aborts a partial assembly and counts `badFragments` (223–263).
8. When `inRemaining_` reaches 0, call
   `handler_(context, channel, rxPayload_, inCursor_, inTimestampUs_)` (269–278).

Dispatch: `channelThunk → Sensor::onChannel → Sh2Client::handlePayload`
(`BNO085Sensor.cpp:421–424`, `Sh2Client.cpp:51–73`):

| Channel | Handler | Result |
| --- | --- | --- |
| 0 command/advertisement | logged only | `handlePayload:53–56` |
| 1 executable | `handleExecutable` → `resetComplete` | `75–96` |
| 2 SH-2 control | `handleControl` | `98–145` |
| 3/4 reports | `handleReports` | `274–298` |
| 5 gyro-RV | logged "ignored" | `67–69` |

---

## 4. SH-2: product ID, set/get feature

### 4.1 Product ID (`Sh2Client.cpp:385–397`, `147–184`)

Request payload `{0xF9, 0x00}` on channel 2. Each `0xF8` response is 16 bytes:

```
report[1] resetCause, [2] major, [3] minor, [4..7] partNumber LE,
[8..11] buildNumber LE, [12..13] patch LE
```

The first response sets `product_.valid = true`. `prodExpected_` becomes 5 for part numbers
10004095 / 10004818 / 10005028, otherwise 4. There is **no** known-good part-number gate;
`ErrorCode::ProductIdMismatch` is never produced. Because `valid` is set on the first response,
configuration may begin before all product-ID entries have arrived.

### 4.2 Set feature — wire bytes and a defect

`Sh2Client::setFeature` (`399–433`) builds a 17-byte payload:

```
[0]=0xFD  [1]=nextCmdSeq_++  [2]=0x00  [3..4]=0
[5..8]=intervalUs LE  [9..12]=0  [13..16]=0
```

The `reportId` argument is discarded: `(void)reportId;` at line 431.

Canonical SH-2 `SetFeatureReport_t` (Hillcrest/CEVA `sh2.c`, packed, no sequence, no command byte):

```
reportId(0xFD), featureReportId, flags, changeSensitivity(u16),
reportInterval(u32), batchInterval(u32), sensorSpecific(u32)   // 17 bytes
```

So byte 1 must be the feature report ID (`0x05` for Rotation Vector); the code puts a command
sequence counter there. The host test enshrines the same wrong bytes
(`test/native/test_main.cpp:445–464`: 21-byte frame total, interval at `frame[9]`).

Additionally, `StageSetFeature` waits for an `0xF1` command response for `0xFD` (`267–274`). The
reference SH-2 client treats `0xFD` as fire-and-forget (`setSensorConfigStart` completes
immediately after send) and does not wait for an ack. Combined with the byte-1 defect, this stage
is very likely never to complete against real hardware — consistent with README/PHASE1-NOTES
stating stage 3+ is unverified.

### 4.3 Get feature (correct for the reference layout)

`getFeature` sends `{0xFE, 0x05}` (`435–450`). `handleGetFeatureResponse` (`237–256`) matches on
`featureId_`, reads `reportInterval_uS` at report offset 5 (1 id + 1 featureId + 1 flags + 2
sensitivity = 5), stores `grantedIntervalUs_`, state `Ok`.

### 4.4 Command response parsing

`handleCommandResponse` (`186–235`) treats `report[2]` as command, `report[3]` as command sequence
and `report[5]` as hub status. `report[5]` is `r[0]` in the canonical `CommandResp_t`, so the
status offset is right.

### 4.5 Timestamp reports — transposed IDs and wrong channel

`Sh2Codes.h:17–18` defines `kBaseTimestampRef = 0xFA` and `kTimestampRebase = 0xFB`. The reference
driver defines the opposite: **0xFB = base timestamp reference (u32), 0xFA = timestamp rebase
(i32)**. The implementation handles both only in `handleControl` (channel 2), while the reference
processes them inside the input-report stream (channels 3/4). `handleReports` decodes only `0x05`;
other reports go to `handlers_.sensorReport`, which `Sensor::begin` sets to `nullptr` (line 67),
so `0xFA/0xFB` on channels 3/4 are silently skipped. Net effect: `rebaseOffsetUs_` is either never
updated or updated with the wrong quantity.

---

## 5. Decode → timestamp → encode

### 5.1 Decode

`Sh2Client::handleReports` (`274–298`) walks concatenated reports via the fixed `kReportLengths`
table (`Sh2Codes.cpp:15–73`). Unknown IDs stop parsing of that payload.

`decodeRotationVector` (`sh2/Sh2Reports.cpp:8–26`) requires `length >= 14` and `report[0] == 0x05`:

```
seq = report[1]
status = report[2]
delayTicks = ((report[2] & 0xFC) << 6) | report[3]
i = readI16(4), j = readI16(6), k = readI16(8), real = readI16(10)
accuracyQ12 = readI16(12)
```

### 5.2 Fill `Quaternion` (`Sh2Client.cpp:300–375`)

- `rawX..rawW = i, j, k, real`; floats = raw × `1/16384` (`kQ14Scale`).
- `accuracy = accuracyFromStatus(status)` (status bits 1:0), `rawAccuracy = accuracyQ12`.
- `tMeasUs = computeMeasTime(tEdgeUs, delayTicks)` = `clamp0(tEdge − 12000 + delayTicks×100 +
  rebaseOffsetUs_)` (`377–383`).
- `tHostUs = tEdgeUs` (poll time, see §3.1).
- `seq`, `epoch = (uint8_t)epoch_`, `status`, `delayTicks`, `valid = true`.
- Duplicate suppression: identical `(seq, tMeasUs)` as the last accepted report → `duplicates++`,
  dropped (335–343). Otherwise an 8-bit gap from the expected next sequence adds to `reportGaps`
  (345–355).
- Calls `handlers_.quaternion` → `Sensor::quaternionThunk` → `Sensor::onQuaternion`.

### 5.3 Sensor-level handling

`Sensor::onQuaternion` (`426–444`): stores `latest_`, sets `newData_` (consumed by `latest()`),
updates `diag_.lastReportUs/lastAccuracy/reportsDecoded`, restores `Streaming` from `Degraded`,
and if `manager_.emitBinaryFrames` calls `encoder_.writeQuaternion(index_, q)`.

### 5.4 Encoded frame (34 bytes)

`BinaryEncoder::writeQuaternion` (`79–98`) + `emit` (`25–65`):

```
A5 5A | ver=01 | type=01 | len=0x0018 | seq(u16 LE) | payload(24) | crc16(u16 LE)
payload: id, flags, accuracy, seq, epoch, qFormat=14,
         rawX i16, rawY i16, rawZ i16, rawW i16, rawAccuracy i16,
         tMeasUs u32, tHostUs u32
```

CRC-16/CCITT-FALSE (init `0xFFFF`, poly `0x1021`), computed over bytes `[2, 8+N)` (version through
last payload byte), appended little-endian (`emit:48–50`, `util/Crc16.cpp`). Written through
`IOutputSink::write`; the example uses `StreamSink` over `Serial`. A short write increments
`dropped_` and returns false; `onQuaternion` ignores the return, so no `ERROR` frame is emitted
for output loss.

Other frames: `HELLO` at begin (`writeHello`), `INIT_RESULT` at `enterStreaming`/`enterFailed`,
`SENSOR_STATUS` periodically (`emitStatusIfDue`), `ERROR` on failure and on silence.
`MANAGER_DIAGNOSTICS` is implemented but **never called**.

---

## 6. State ownership as implemented

| State | Owner | Notes |
| --- | --- | --- |
| SPI peripheral, `SPISettings` (MODE3, clock), mutex, `lockTimeoutUs_` | `SpiBusManager` | Single instance; `zeros_[256]`/`discard_[64]` scratch, `transactions_`, `lockTimeouts_`, `busErrors_` |
| CS/INT/RST/WAKE pin numbers, `begun_`, `transfers_` | `Esp32SpiHal` (per sensor) | Pin numbers copied from `SensorConfig` |
| RX/TX buffers, `inRemaining_/inCursor_/inChannel_/inTimestampUs_`, `nextInSeq_[8]`, `nextOutSeq_[8]`, `channelSeen_[8]`, `ShtpStats` | `ShtpTransport` (per sensor) | Sequence numbers are per sensor **and** per channel |
| `product_`, `prodEntries_`, `prodPending_`, `cmd*`, `feature*`, `grantedIntervalUs_`, `rebaseOffsetUs_`, `epoch_`, `haveQuaternion_`, `lastQuat*`, `Sh2Client::Stats` | `Sh2Client` (per sensor) | Timestamp rebase/epoch are per device |
| `state_`, `stage_`, deadlines, `intPending_/intEdgeUs_`, `latest_`, `newData_`, `Diagnostics`, `encoder_` + frame `sequence_` | `Sensor` (per device) | Encoder is a per-sensor member, so the 16-bit frame sequence is per sensor |

Shared read-only globals: `kZeroFill[256]` (`ShtpTransport.cpp:12`), the report-length table, CRC
constants. All `const`, so the "no mutable file-scope state" design rule is honoured.

---

## 7. Timeouts, retries and error paths (exhaustive as implemented)

| Condition | Constant / source | Action |
| --- | --- | --- |
| Config invalid | `BNO085Sensor.cpp:32–49` | Return `Status::InvalidConfig`; no state machine |
| Bus/HAL begin fails | `70–79` | `State::Failed`, lastError `InvalidConfig`, return HAL status |
| No complete SHTP packet in boot window | `bootTimeoutMs` 300 ms, `235–237` | `enterFailed(BootTimeout)` → `Failed`, `INIT_RESULT` + `ERROR`. **No automatic retry** |
| Product ID not valid in stage window | 100 ms from stage entry, `250–251` | `enterFailed(Sh2Timeout)` |
| Product ID response missing | 100 ms in `Sh2Client::pollTimeouts`, `464–473` | Clears pending; counts a timeout only if no product is known |
| Product ID request retry | every 10 ms while `!busy`, `252–255` | Re-sends `0xF9` |
| Set-feature transport send fails | 10 ms retry, `259–266`, `275–278` | Re-sends |
| Set-feature response `HubError` | `271–273` | `enterFailed(Sh2BadResponse)` |
| Set-feature response missing | 100 ms, `273–274` | `enterFailed(Sh2Timeout)` |
| Set-feature stage window | 100 ms, `279–281` | `enterFailed(Sh2Timeout)` |
| Get-feature send rejected | retried every update, `285–289` | `getFeatureSent_` stays false |
| Get-feature `HubError` or `Timeout` | `290–294` | `enterFailed(Sh2BadResponse)` (both cases) |
| Get-feature stage window | 100 ms, `295–297` | `enterFailed(Sh2Timeout)` |
| No reports while streaming | `(interval×5)/2 + 50 ms`, `329–346` | `Degraded`, `NoReports`, one `ERROR` frame; back to `Streaming` on next report. No recovery |
| SPI bus lock | 2 ms, `SpiBusManager.cpp:101–105` | `transfer` false, `lockTimeouts_++`; transport returns `BusError`; `Sensor` sets `SpiRead`, clears INT latch (`203–206`) |
| Header/body transfer fails | `ShtpTransport.cpp:82–86, 207–211` | `rxErrors++`, `BusError` |
| Oversize transfer | `146–205` | Drained in ≤256-byte chunks, ≤8 continuation fragments; error only on read failure |
| Reserved/short/bad-channel | `110–132` | `discard(length)` capped at 1024 bytes, counted, `Malformed`; `Sensor` sets `ShtpBadFrame` and keeps servicing |
| Length 0 | `104–108` | `Spurious`, counted, visit ends |
| Per-visit work | 3 transfers; 2 ms budget checked **after** each transfer, `188–217` | Stops servicing; can overshoot by one transfer |
| Wake before write | 200 µs pulse, 2 ms wait, then proceeds anyway (`Esp32SpiHal.cpp:97–116`) | `prepareWrite` never fails |
| Reset pulse | 10 µs low + 1 ms delay, `118–128` | Blocking 1 ms |
| `recover()` | attempts ≥ 5 → `enterFailed(BootTimeout)`; else backoff 250/500/1000/2000/4000 ms, `369–394` | `State::Recovering`; `update()` waits for `retryAtUs_`, then `startBoot()` (`148–151`) |
| Device reset-complete | `Sh2Client.cpp:75–96` → `Sensor::onResetComplete` `446–463` | `epoch_++`; pending cmd/feature forced to `Timeout`; sensor returns to `StageSetFeature` (ignored during Boot/Identify) |
| Output short write | `BinaryEncoder.cpp:52–61` | `dropped_++`, returns false silently |
| `sendReset()` failure | `Sh2Client.cpp:452–461` | Returns false; caller in `startBoot` ignores it |

**Not present:** automatic fault-triggered recovery (nothing calls `recover()` on boot/SH-2/silence
failure), "3 consecutive invalid headers → Degraded", stuck-INT counter, per-index recovery phase
offset, shared reset groups, and `SH2_TIMEOUT → Degraded` (it goes to `Failed`).

---

## 8. Call-flow diagram

```
setup()
 └─ Sensor::begin                       BNO085Sensor.cpp:22
     ├─ validate config
     ├─ ShtpTransport::attach/setHandler
     ├─ Sh2Client::attach/setHandlers
     ├─ BinaryEncoder::attach
     ├─ Esp32SpiHal::begin ──► SpiBusManager::begin   [SPI.begin, MODE3, mutex]
     ├─ BinaryEncoder::writeHello ──► IOutputSink::write
     └─ startBoot
         ├─ ShtpTransport::reset / Sh2Client::reset
         ├─ state=Booting, stage=StageBoot, deadline=now+300ms
         └─ Esp32SpiHal::pulseReset (NRST low 10us, delay 1ms, high)

loop()
 └─ Sensor::update                       BNO085Sensor.cpp:139
     ├─ Recovering && now>=retryAtUs ──► startBoot()
     ├─ serviceBus(now)                  165
     │   ├─ Esp32SpiHal::intAsserted()   digitalRead (no ISR)
     │   ├─ latch intPending_/intEdgeUs_, ++intAssertions
     │   └─ while pending && visits<3 && elapsed<=2ms
     │       └─ ShtpTransport::service(intEdgeUs_)              77
     │           ├─ readRaw(peek,4) ──► Esp32SpiHal::transfer
     │           │                       └─ SpiBusManager::transfer   [lock, CS low, xfer, CS high, unlock]
     │           ├─ validate header/seq; oversize drain or full read
     │           ├─ assemble fragments in rxPayload_
     │           └─ channelThunk ──► Sensor::onChannel ──► Sh2Client::handlePayload   51
     │               ├─ ch0: log advertisement
     │               ├─ ch1: handleExecutable ──► Sensor::onResetComplete
     │               ├─ ch2: handleControl
     │               │     ├─ 0xF8 handleProdId
     │               │     ├─ 0xF1 handleCommandResponse
     │               │     ├─ 0xFC handleGetFeatureResponse
     │               │     └─ 0xFB handleTimestampRebase
     │               └─ ch3/4: handleReports                    274
     │                     └─ 0x05 handleRotationVector         300
     │                         ├─ sh2::decodeRotationVector      Sh2Reports.cpp:8
     │                         ├─ computeMeasTime (tMeas)
     │                         ├─ duplicate/gap accounting
     │                         └─ quaternionThunk ──► Sensor::onQuaternion   426
     │                             ├─ latest_, newData_, diag
     │                             └─ if emitBinaryFrames
     │                                 └─ BinaryEncoder::writeQuaternion ──► emit ──► CRC16 ──► sink->write(34 bytes)
     ├─ Sh2Client::pollTimeouts(now)    → Timeout states
     ├─ advanceInit(now)                → Boot→Identify→SetFeature→GetFeature→Stream
     ├─ checkSilence(now)               → Degraded + ERROR
     └─ emitStatusIfDue(now)            → SENSOR_STATUS frame
```

---

## 9. Differences between implementation and documented architecture

### 9.1 Already acknowledged in `docs/PHASE1-NOTES.md`

- `Sensor(IHal&)` instead of the sensor creating its own HAL.
- Own minimal SH-2 instead of vendoring CEVA's.
- 16-bit frame sequence at offset 6 instead of 32-bit.
- `tMeas` uses a fixed −12 000 µs instead of applying the `0xFA` base-timestamp reference.
- Oversize transfers drained, not assembled.
- `Sensor::get()` returns false.

### 9.2 Not acknowledged, materially different from `docs/ARCHITECTURE.md`

1. **No ISR.** §6.4 specifies `attachInterrupt(..., FALLING)`, an atomic pending mask and a level
   re-check. The code polls `digitalRead` every `update()`. `tHostUs` is therefore poll-observation
   time, not the H_INTN assertion time.
2. **No `Manager`, no cross-sensor round-robin.** §3/§5.1 specify a sensor table, cursor, fairness
   and starvation handling. There is none; `serviceBudgetUs`/`maxTransfersPerVisit` apply inside
   one sensor only.
3. **No aggregation layer.** `aggregate/ReportQueue.h`, `ReportAggregator`, `next(Record&)` and the
   record queue do not exist. `Record` is declared but never constructed. Emission is synchronous
   from the report callback; the `BinaryEncoder` and its sequence are per sensor, not the single
   output-writer/drain described in §4.3/§8. `writeManagerDiagnostics` exists but is never called.
4. **No automatic recovery orchestration.** §7.4's `Streaming → Recovering → Reconfiguring →
   Streaming` with attempt limits, backoff and `(index × 37 ms)` phase offset is not implemented.
   Faults call `enterFailed()` and `update()` returns early forever; the application must call
   `recover()` (the example binds this to `r`).
5. **`resetGroup` is dead.** Declared in `SensorConfig` and initialised to 0 in both constructors;
   never read. §7.1/§7.5 shared-reset behaviour does not exist.
6. **Validation is much thinner than §4.1 / API §2.3.** No sensor-count check, no distinct-CS
   check, no `csPin != intPin/resetPin` check, no duplicate-pin status. `Status::DuplicatePin`,
   `NoSensors`, `OutputUnavailable` are never returned. `allowGrvFallback` is unused.
7. **Optional reports are accepted but never used.** `enable()` stores extras and `reportsValid()`
   validates them, but `advanceInit` sends only the primary report, `handlers_.sensorReport` is
   `nullptr`, and `get()` returns false. Extras are silently ignored rather than rejected.
8. **Failure detection differs.** §7.3's "3 consecutive invalid headers", "malformed counter
   increase", product-ID mismatch gate and `Degraded` on SH-2 timeout are absent. SH-2 failures go
   straight to `Failed`. `ErrorCode::ProductIdMismatch`, `BusTimeout`, `SpiWrite`,
   `ShtpBadChannel`, `QueueOverflow`, `OutputOverflow` are never produced.
9. **Fragments are handled better than documented.** Ordinary transfers ≤256 B **are** reassembled
   (192-byte payload buffer, `223–267`); only oversized transfers are drained.
10. **Bus-lock scope.** The lock is taken per SPI transaction (`SpiBusManager::transfer`); there is
    no separate output-drain phase, and the output write happens inside `update()` between SPI
    transfers. The bus is not held across the write, but the §4.3 "drain outside the bus lock"
    structure is not present.
11. **`update()` cannot guarantee `serviceBudgetUs`.** The budget is checked *after* each transfer,
    and a 256-byte read at 1 MHz is already ~2 ms. `maxServiceUs` records the overrun rather than
    preventing it.
12. **Boot completion criterion.** §7.2 says ready when H_INTN asserts; the code waits for any
    complete SHTP packet (`225`), and the 552-byte advertisement does not count as one because the
    oversize path returns before `++stats_.packets`.
13. **Diagnostics gaps.** `diag_.product`, `csTransactions`, `spiTransfers`, `busTimeouts`,
    `transferBytes` are never written (grep confirms). The example's `printDiagnostics()` prints
    `d.product.partNumber`, which is therefore always `0`. `SpiBusManager::lockTimeouts_` is never
    surfaced.
14. **Status frame semantics.** `SENSOR_STATUS` is emitted only periodically, not "periodically and
    on state change"; payload offset 8 is `reportsDecoded`, while `API.md` documents `rxPackets`.

### 9.3 Protocol/code defects found in the implementation (not documented anywhere)

15. **Set-feature payload byte 1 is a sequence counter, not the feature report ID**
    (`Sh2Client.cpp:405–413`, `(void)reportId` at 431). Canonical SH-2 is
    `0xFD, featureReportId, flags, …`; this sends `0xFD, seq, flags, …`, so the hub is asked to
    configure report `0x00/0x01/…` instead of `0x05`. The host test enshrines the same bytes
    (`test_main.cpp:445–464`).
16. **Set-feature waits for an `0xF1` ack** that the reference SH-2 client does not expect. Combined
    with (15), `StageSetFeature` is likely to time out on real hardware. This is exactly the
    unverified stage 3 in `PHASE1-NOTES.md`.
17. **`0xFA`/`0xFB` are transposed and handled on the wrong channel.** `Sh2Codes.h:17–18` labels
    `0xFA` base timestamp ref and `0xFB` timestamp rebase; the reference defines the opposite
    (`0xFB` = base timestamp reference, `0xFA` = timestamp rebase). The reference also processes
    them inside the input-report stream; this code parses them only on channel 2 and skips them on
    channels 3/4. Net effect: `rebaseOffsetUs_` is never updated, or updated with the wrong
    sign/quantity.

### 9.4 Files promised by the architecture but absent

`src/aggregate/`, `src/util/RingBuffer.h`, `src/util/Time.h`, `src/diag/Diagnostics.h`,
`src/diag/RecoveryPolicy.h` (the struct lives in `BNO085Types.h`),
`src/output/JsonDebugEncoder.*`, `src/sh2/third_party/ceva/` (the audit's "vendored stateless
decoder"), `extras/host/bno085_stream.py`, `examples/02`/`03`, `BNO085Manager.*`, `LICENSE`,
`NOTICE`. Root contains only `library.properties`, `keywords.txt`, `README.md` plus `src/`,
`docs/`, `examples/`, `test/`.

### 9.5 Minor behavioural notes

- `disable()` only sets `State::Disabled`; it sends nothing to the device, which keeps streaming and
  asserting INT.
- `configure()` sends a new set-feature without waiting for or verifying the granted interval, and
  (because of item 15) does not actually target report `0x05`.
- `begin()` writes `HELLO` with `sensorCount = 1` hardcoded.
- `Sh2Client::reset()` clears pending state but keeps `rebaseOffsetUs_` and `epoch_`; after a device
  reset, timestamps keep the old offset until (never-occurring) rebase handling changes it.
- `Sensor::end()` only drives CS high and clears `started_`; it does not reset SHTP/SH-2 state or
  the bus manager.
- `SpiBusManager::transfer` has no RAII CS guard, but it has no early return between CS-low and
  CS-high, so the "CS deasserted on every path" invariant holds in practice.
- `ShtpTransport::discard()` caps at 1024 bytes, so a genuinely long malformed transfer can leave
  bytes pending and desynchronise the stream.

---

## 10. Verification status (from the repo's own evidence)

- Host tests: 30 cases / 168 checks, MSVC 2022, `test\native\build_msvc.bat`; GCC variant
  `sh test/native/build_gcc.sh`. They cover CRC, framing, SHTP header/oversize/fragment paths,
  Rotation Vector decode, timestamps, timeouts and the init state machine. They do **not** catch
  the set-feature byte-1 defect because the test asserts the same layout the code produces.
- ESP32-S3 build: `esp32:esp32:esp32s3`, core 3.3.12 — 324 262 B flash, 23 880 B static RAM.
- Hardware-confirmed (README/PHASE1-NOTES): bus/CS, reset pulse, boot in ~93.7 ms, 552-byte
  channel-0 advertisement drained in 4 bus transactions, stream returns to idle.
- **Not** hardware-confirmed: product-ID retrieval, set/get feature, quaternion streaming,
  timestamp reconstruction, soak, INT-loss behaviour, cold boots.

---

## 11. Suggested next steps for the next agent (no changes made yet)

> Do not assume the "intended" design from `docs/`; the code above is what runs.

1. **Fix set-feature first** (defect 15): byte 1 must be `reportId`, not `nextCmdSeq_`; remove the
   `(void)reportId`. Update `test/native/test_main.cpp:445–464` to assert `frame[5] == 0x05` and
   re-baseline the expected bytes. Decide, with hardware evidence, whether the `0xF1` ack must be
   awaited (defect 16); the reference SH-2 client does not.
2. **Correct `0xFA`/`0xFB`** (defect 17) and parse them in `handleReports` on channels 3/4, as the
   reference does, or explicitly document that timestamp rebase is unsupported.
3. **Populate `Diagnostics::product`** (and, if wanted, `csTransactions`/`busTimeouts`) from
   `Sh2Client::productInfo()`/`SpiBusManager`, or change the example so it does not print a
   permanently-zero field.
4. **Decide the INT story:** implement the documented ISR + pending mask (to get honest
   `tHostUs`), or amend `ARCHITECTURE.md` §6.4 to document polling. The current docs and code
   contradict each other.
5. **Decide the recovery story:** either add fault-triggered `recover()` with attempt limits and
   backoff, or document that recovery is application-driven in Phase 1.
6. **Decide the multi-sensor story:** `Manager`, aggregation and per-sensor encoder sequencing are
   Phase 2; today there is no way to use the library with more than one sensor without writing the
   scheduler yourself.
7. Re-run host tests and the ESP32-S3 compile after any change; do not claim hardware validation
   for stages 3+ until it is actually measured.

---

### Appendix A — quick symbol index

| Concern | File | Symbols |
| --- | --- | --- |
| State machine | `src/BNO085Sensor.cpp` | `begin`, `update`, `startBoot`, `serviceBus`, `advanceInit`, `enterStreaming`, `checkSilence`, `recover`, `onQuaternion`, `onResetComplete`, `mirrorStats` |
| SHTP | `src/shtp/ShtpTransport.cpp` | `service`, `send`, `readRaw`, `discard` |
| SH-2 | `src/sh2/Sh2Client.cpp` | `handlePayload`, `handleControl`, `handleReports`, `handleRotationVector`, `requestProductIds`, `setFeature`, `getFeature`, `sendReset`, `pollTimeouts` |
| RV decode | `src/sh2/Sh2Reports.cpp` | `decodeRotationVector` |
| Output | `src/output/BinaryEncoder.cpp` | `emit`, `writeHello`, `writeQuaternion`, `writeInitResult`, `writeStatus`, `writeError`, `writeManagerDiagnostics` |
| Bus | `src/hal/SpiBusManager.cpp` | `begin`, `transfer`, `lock`, `unlock` |
| HAL | `src/hal/Esp32SpiHal.cpp` | `begin`, `transfer`, `intAsserted`, `prepareWrite`, `pulseReset` |
| Frame protocol | `src/output/OutputProtocol.h` | sync `0xA5 0x5A`, version 1, 8-byte header, 64-byte max payload, CRC-16/CCITT-FALSE |

### Appendix B — key constants

| Constant | Value | Where |
| --- | --- | --- |
| `kDefaultIntervalUs` | 10 000 (100 Hz) | `BNO085Config.h:44` |
| `kMinIntervalUs` | 1 000 | `BNO085Config.h:45` |
| `kSpiMaxHz` | 3 000 000 | `BNO085Config.h:46` |
| RX transfer / payload / TX transfer | 256 / 192 / 64 bytes | `BNO085Config.h:12–22` |
| Boot timeout | 300 ms | `RecoveryPolicy::bootTimeoutMs` |
| Command timeout | 100 ms | `RecoveryPolicy::commandTimeoutMs` |
| Recovery backoff | 250/500/1000/2000/4000 ms | `RecoveryPolicy::backoffMs` |
| Max recovery attempts | 5 | `RecoveryPolicy::maxAttempts` |
| Bus lock timeout | 2 000 µs | `ManagerConfig::busTimeoutUs` |
| Service budget / max transfers | 2 000 µs / 3 | `ManagerConfig` |
| Silence limit | `(interval × 5)/2 + 50 ms` | `BNO085Sensor.cpp:332–333` |
| `tMeas` reference | −12 000 µs + `delayTicks × 100` + rebase | `Sh2Client.cpp:13–15, 377–383` |
