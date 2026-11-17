# BNO085Multi — implementation trace and documentation-vs-code audit

**Purpose of this file.** Handoff document for another AI agent. It records what the code in this
repository actually does, traced from `setup()` through to a decoded Rotation Vector leaving the
binary output, and lists every place where the implementation differs from the documents in
`docs/`.

The file has been through three passes. Pass 1 (§0–§11) traced the implementation and listed
documentation divergences; it changed no source. Pass 2 (§12) re-derived the five highest-priority
findings from the source and the reference **without relying on pass 1**, corrected two statements,
and added per-finding verdicts. Pass 3 (§13) implemented the two highest-priority protocol
corrections — set-feature payload/completion and timestamp reference/rebase — added regression
tests, re-ran the host suite and the ESP32-S3 compile, and committed the result. Where an earlier
section still describes the pre-fix behaviour, that section's pass-3 status note and **§13** are
authoritative.

| Item | Value |
| --- | --- |
| Repository root | `C:\Users\Apahapi\Desktop\Source codes\Shared SPI Architecture` |
| Library name / version | `BNO085Multi` 0.1.0 (`library.properties`) |
| Analysed commit | `d040c8e` — "Drain oversized SHTP transfers completely" (`git log --oneline`) |
| Pass-2 document commit | `5ce28fe` — "Update output.md"; `src/` unchanged since `d040c8e` |
| Pass-3 code/test commit | recorded in §13.6 (set-feature + timestamp fixes) |
| Stage | Phase 1 — single-sensor baseline, ESP32-S3, Arduino core 3.3.12 |
| Analysis date | 2026-11-17 |
| Authoritative sources for this file | The source files themselves. Protocol comparisons were checked against the Hillcrest/CEVA reference `sh2.c`, downloaded for pass 2 to `%TEMP%\sh2_ref.c` (Adafruit mirror; the same upstream that `docs/UPSTREAM-AUDIT.md` analyses). |
| Files read | `README.md`; `docs/{ARCHITECTURE,API,PHASE1-NOTES,UPSTREAM-AUDIT}.md`; `src/BNO085Multi.h`, `BNO085Config.h`, `BNO085Types.{h,cpp}`, `BNO085Sensor.{h,cpp}`; `src/hal/{IHal.h,SpibusManager.*,Esp32SpiHal.*}`; `src/shtp/{ShtpProtocol.h,ShtpTransport.*}`; `src/sh2/{Sh2Codes.*,Sh2Client.*,Sh2Reports.*}`; `src/output/{OutputProtocol.h,IOutputSink.h,BinaryEncoder.*,ArduinoSink.h}`; `src/util/Crc16.*`; `src/diag/Logger.h`; `examples/01_SingleSensor/01_SingleSensor.ino`; `test/native/{test_main.cpp,mock_hal.h}` |

> Note for the next agent: the claims below are anchored to file + line numbers at commit
> `d040c8e`. Re-read the cited functions before editing; line numbers will drift after changes.
>
> **Pass-2 corrections to keep in mind while reading §0–§11:**
> - `0xFA`/`0xFB` arriving on channels 3/4 do **not** abort parsing of the rest of the payload.
>   Both IDs have entries in `kReportLengths` (`Sh2Codes.cpp:62–63`, length 5), so they are
>   length-skipped and passed to the null `sensorReport` handler. They are silently ignored, not
>   parse-terminating. (§4.5 and §9.3 item 17 overstate this.)
> - “Set-feature is likely to time out on real hardware” is a **hypothesis**, not a measurement.
>   The code/reference divergence is certain; the hub's actual `0xF1` behaviour is unproven. See
>   §12.2.
> - Two defects missing from §9.3 are recorded there now as items 18–19 (recovery attempt counter;
>   `dropped_` counts writes that were never supposed to happen).

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
(`test/native/test_main.cpp:445–464`: 21-byte frame total, interval at `frame[9]`, and
`frame[5] == 0x00` at line 457). Every other field is at the canonical offset:

| Offset | Implementation | Reference | Verdict |
| --- | --- | --- | --- |
| 0 | `0xFD` | `reportId` = `0xFD` | correct |
| 1 | `nextCmdSeq_++` | `featureReportId` (`0x05`) | **wrong** |
| 2 | `0x00` | `flags` | correct (no flags requested) |
| 3–4 | 0 | `changeSensitivity` (u16) | correct |
| 5–8 | `intervalUs` LE | `reportInterval_uS` (u32) | correct offset/endianness |
| 9–12 | 0 | `batchInterval_uS` (u32) | correct |
| 13–16 | 0 | `sensorSpecific` (u32) | correct for RV with no options |

The reference's `sendCtrl` (`sh2.c:668–671`) adds only the 4-byte SHTP header; it does **not**
insert a command sequence into the payload. This library already has a per-channel SHTP sequence
(`ShtpTransport::nextOutSeq_`), so `nextCmdSeq_` is a field that does not exist in this report. The
argument is explicitly discarded at line 431 (`(void)reportId;`), and the log line at 427 prints the
correct ID while the wire carries something else.

Additionally, `StageSetFeature` waits for an `0xF1` command response for `0xFD` (`267–274`). The
reference SH-2 client treats `0xFD` as fire-and-forget: `setSensorConfigStart` calls
`opCompleted(pSh2, rc)` immediately after `sendCtrl`, and `setSensorConfigOp` declares no `.rx`
handler. `opRx` (`sh2.c:350–356`) only dispatches when an operation is in progress **and** has an
`rx` method, so the reference would ignore a `0xF1` for `0xFD` even if one arrived.

The implementation, by contrast, only leaves the stage on `cmdState() == Ok`, which requires that
ack. An absent ack is fatal; a present ack is harmless because `handleCommandResponse` only logs a
`commandSeq` mismatch and does not reject it (`Sh2Client.cpp:211–216`). Whether this hub emits
`0xF1` for `0xFD` is **unproven** — the “very likely never completes” statement is a hypothesis,
not evidence. See §12.2 for the verdict split (divergence confirmed, device behaviour uncertain).

> **Pass-3 status: fixed (§13.1).** Byte 1 is now `reportId`, `nextCmdSeq_` and `(void)reportId`
> are gone, and `setFeature` completes on a successful send. A stray `0xF1` is ignored. The
> paragraph above and the byte table describe the pre-fix code.

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
driver (`sh2.c:293–303`) defines the opposite:

```c
#define SENSORHUB_BASE_TIMESTAMP_REF (0xFB)   // uint32_t timebase
#define SENSORHUB_TIMESTAMP_REBASE   (0xFA)   // int32_t  timebase
```

The implementation handles both only in `handleControl` (channel 2). `handleReports` decodes only
`0x05`; other reports go to `handlers_.sensorReport`, which `Sensor::begin` sets to `nullptr`
(`BNO085Sensor.cpp:67`), so `0xFA/0xFB` on channels 3/4 are silently skipped. **They are not
parse-terminating:** both IDs are in `kReportLengths` (`Sh2Codes.cpp:62–63`, length 5), so the
cursor advances correctly and the rest of the payload is still decoded. (This corrects an earlier
draft.)

Reference processing is in `sensorhubInputHdlr` (`sh2.c:542–591`), i.e. the input-report stream, not
the control channel:

```c
referenceDelta = 0;                                  // per payload, line 549
...
if (reportId == SENSORHUB_BASE_TIMESTAMP_REF) {
    referenceDelta = -rpt->timebase;                 // line 567 (assignment)
} else if (reportId == SENSORHUB_TIMESTAMP_REBASE) {
    referenceDelta += rpt->timebase;                 // line 572 (accumulate)
}
...
event.timestamp_uS = touSTimestamp(timestamp, referenceDelta, delay);
```

and `touSTimestamp` (`sh2.c:524–540`):

```c
timestamp += hostInt + (referenceDelta + delay) * 100;
```

Consequences of the three divergences (transposed IDs, wrong scope, wrong channel):

- Both fields are in 100 µs units; the base reference **replaces** and negates the delta, the
  rebase **accumulates** onto it, and the delta lives only for the payload being parsed. The
  implementation instead accumulates into a persistent `rebaseOffsetUs_` and increments `epoch_`
  on the base-reference ID.
- `computeMeasTime` substitutes a fixed `-12 000 µs` for `-timebase*100` (`Sh2Client.cpp:13`,
  `377–383`). That is only correct if the hub's base timebase happens to be 120 ticks; otherwise
  every `tMeasUs` carries the difference as a systematic bias. `PHASE1-NOTES.md` acknowledges the
  approximation, but it is only safe because the base reference is being ignored.
- If a genuine `0xFB` ever did arrive on channel 2, `handleTimestampRebase` would parse an
  unsigned base timestamp as `int32_t`, multiply by 100 and clamp to ±`INT32_MAX` — one large
  value would permanently saturate the offset.

Net effect: on real hardware `rebaseOffsetUs_` is never updated (the input path is a no-op); the
`0xFA`/`0xFB` cases in `handleControl` are unreachable unless the device behaves unlike the
reference; and `epoch` never changes at a true device-time discontinuity.

> **Pass-3 status: fixed (§13.2).** The IDs are swapped to `0xFB` = base reference (u32) and
> `0xFA` = rebase (i32). Both are handled inside `handleReports` with a per-payload delta; the
> control-channel cases and `handleTimestampRebase` were deleted. `epoch_` increments on `0xFA`.
> Everything above this note describes the pre-fix code.

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
- `tMeasUs = computeMeasTime(tEdgeUs, delayTicks, referenceDeltaTicks)` =
  `clamp0(tEdge + (referenceDeltaTicks + delayTicks)×100)` (`376–382`); see §13.2 for the delta's
  per-payload scope. The pre-fix form added a persistent `rebaseOffsetUs_`.
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
| `product_`, `prodEntries_`, `prodPending_`, `cmd*`, `feature*`, `grantedIntervalUs_`, `epoch_`, `haveQuaternion_`, `lastQuat*`, `Sh2Client::Stats` | `Sh2Client` (per sensor) | No persistent timestamp offset since pass 3; `epoch_` marks rebases/reset |
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
| Set-feature `HubError` branch | `271–273` | Now unreachable: pass 3 completes on send (§13.1) |
| Set-feature ack wait | formerly 100 ms, `273–274` | Removed in pass 3; a late `0xF1` is counted unsolicited and ignored |
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
| Device reset-complete | `Sh2Client.cpp:75–96` → `Sensor::onResetComplete` `446–463` | `epoch_++`; any armed cmd/feature forced to `Timeout` (the command flag is no longer ever armed); sensor returns to `StageSetFeature` (ignored during Boot/Identify) |
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
    (**fixed in pass 3, §13.1**; text retained as the pre-fix record)
    (`Sh2Client.cpp:405–413`, `(void)reportId` at 431). Canonical SH-2 is
    `0xFD, featureReportId, flags, …`; this sends `0xFD, seq, flags, …`, so the hub is asked to
    configure report `0x00/0x01/…` instead of `0x05`. The host test enshrines the same bytes
    (`test_main.cpp:445–464`).
16. **Set-feature waits for an `0xF1` ack** (**fixed in pass 3, §13.1**: completion is now send
    success and a stray ack is ignored)
    that the reference SH-2 client does not expect (it
    completes immediately after `sendCtrl` and declares no `rx` handler). The divergence is
    certain; whether this hub acks `0xFD` is unproven. If it does not, `StageSetFeature` times out.
    See §12.2.
17. **`0xFA`/`0xFB` are transposed and handled on the wrong channel**
    (**fixed in pass 3, §13.2**: IDs corrected and both handled per payload in `handleReports`).
    `Sh2Codes.h:17–18` labels
    `0xFA` base timestamp ref and `0xFB` timestamp rebase; the reference defines the opposite
    (`0xFB` = base timestamp reference, u32; `0xFA` = timestamp rebase, i32). The reference also
    processes them inside the input-report stream; this code parses them only on channel 2 and
    skips them on channels 3/4. The input-path skip is silent but does not abort the payload,
    because both IDs have lengths in `kReportLengths`. See §4.5 and §12.3.
18. **`recover()` does not reset the attempt counter**, contradicting `ARCHITECTURE.md:410`
    (“it resets the attempt counter”). Attempts are reset only in `begin()` and `enterStreaming()`,
    so five manual retries leave the sensor permanently `Failed` until `begin()` is called again.
    See §12.4.
19. **`dropped_` is not a loss counter, and loss is never surfaced.** `emit()` increments it when
    `sink_ == nullptr` (`BinaryEncoder.cpp:27–30`), which is the normal state when
    `emitBinaryFrames == false` — so `HELLO`, `INIT_RESULT` and `ERROR` writes inflate it even
    though nothing was meant to be written. Real short writes are counted too, but `framesDropped()`
    is never read, every caller ignores the `bool`, and `ErrorCode::OutputOverflow` is never
    produced. See §12.5.

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
- `configure()` sends a new set-feature without waiting for or verifying the granted interval. Since
  pass 3 it targets report `0x05` correctly, but the granted interval is still only observed via the
  separate get-feature exchange in the init sequence.
- `begin()` writes `HELLO` with `sensorCount = 1` hardcoded.
- `Sh2Client::reset()` clears pending state but keeps `epoch_`; timestamp offsets are per payload
  since pass 3, so a payload after reset uses the fallback delta until the next `0xFB`.
- `Sensor::end()` only drives CS high and clears `started_`; it does not reset SHTP/SH-2 state or
  the bus manager.
- `SpiBusManager::transfer` has no RAII CS guard, but it has no early return between CS-low and
  CS-high, so the "CS deasserted on every path" invariant holds in practice.
- `ShtpTransport::discard()` caps at 1024 bytes, so a genuinely long malformed transfer can leave
  bytes pending and desynchronise the stream.

---

## 10. Verification status (from the repo's own evidence)

- Host tests: **35 cases / 194 checks**, MSVC 2022, `test\native\build_msvc.bat`; GCC variant
  `sh test/native/build_gcc.sh` (not run here — no `sh`/`g++` in this environment). They cover CRC,
  framing, SHTP header/oversize/fragment paths, Rotation Vector decode, set-feature bytes and
  completion, timestamp reference/rebase, timeouts and the init state machine. Before pass 3 the
  suite asserted the same wrong set-feature byte 1 the code produced; that test was re-baselined.
- ESP32-S3 build: `esp32:esp32:esp32s3`, core 3.3.12 — **324 210 B** flash (24%), 23 880 B static
  RAM (7%), 0 warnings, 0 errors with `--warnings all` (`build\esp32s3_final.log`).
- Hardware-confirmed (README/PHASE1-NOTES): bus/CS, reset pulse, boot in ~93.7 ms, 552-byte
  channel-0 advertisement drained in 4 bus transactions, stream returns to idle.
- **Not** hardware-confirmed: product-ID retrieval, set/get feature, quaternion streaming,
  timestamp reconstruction, soak, INT-loss behaviour, cold boots.
- Pass 2 (2026-11-17) added **no** new hardware evidence. The set-feature ack question, the real
  magnitude of base timestamps, and actual INT-to-poll latency remain unmeasured; see §12.9.
- Pass 3 implemented P1–P3 from §12.7 and re-ran the host suite and ESP32-S3 compile (above). It
  added **no** hardware evidence either; the same three questions remain open.

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

> The sequencing in §12.8 supersedes this list. The list is retained because it records the
> pass-1 plan; §12 re-derived the findings and re-ordered the repairs.

---

## 12. Independent verification of the five priority findings (pass 2)

**Method.** Each cited function was re-read from the source rather than trusted from pass 1, and
compared with the Hillcrest/CEVA reference `sh2.c` (Adafruit mirror, downloaded to
`%TEMP%\sh2_ref.c`; line numbers below are from that snapshot). No source file was modified.

### 12.0 Verdict summary

| # | Finding | Verdict |
| --- | --- | --- |
| 1a | Set-feature byte 1 carries a command counter instead of the feature report ID | **Confirmed defect** |
| 1b | Set-feature waits for a `0xF1` acknowledgement | **Divergence confirmed; device behaviour uncertain** |
| 2a | `0xFA`/`0xFB` are transposed | **Confirmed defect** |
| 2b | Signedness/units of the two reports | **Confirmed for `0xFA`; `0xFB` is mishandled** |
| 2c | Both parsed only on the control channel | **Confirmed defect** |
| 3 | `tHostUs` is poll time, docs claim INT-assertion time | **Confirmed mismatch (code vs documentation)** |
| 4 | Nothing calls `recover()`; `recover()` does not reset attempts | **Confirmed defect** |
| 5 | Output write failures are never surfaced | **Confirmed defect** |

### 12.1 Set-feature payload construction — confirmed

**Implementation.** `Sh2Client::setFeature` (`src/sh2/Sh2Client.cpp:399–433`), payload built at
405–413. Byte-by-byte comparison with `SetFeatureReport_t` is in §4.2; summary: byte 0 `0xFD`,
byte 1 `nextCmdSeq_++`, byte 2 flags, bytes 3–4 zero, bytes 5–8 interval LE, bytes 9–16 zero.
Total 17 payload bytes / 21-byte SHTP frame, which is correct.

**Reference.** `sh2.c`, `SENSORHUB_SET_FEATURE_CMD`:

```c
uint8_t reportId;          // 0xFD
uint8_t featureReportId;   // sensor id
uint8_t flags;
uint16_t changeSensitivity;
uint32_t reportInterval_uS;
uint32_t batchInterval_uS;
uint32_t sensorSpecific;
```

`setSensorConfigStart` memsets, then fills `req.featureReportId = ...sensorId`, `req.flags` from
the config, `req.changeSensitivity`, `req.reportInterval_uS`, `req.batchInterval_uS`,
`req.sensorSpecific`, and calls `sendCtrl` with `sizeof(req)`. `sendCtrl` (`sh2.c:668–671`) adds
only the SHTP header.

**Only byte 1 is wrong.** The argument is explicitly discarded at line 431 (`(void)reportId;`),
and the log line at 427 prints the correct ID while the wire carries something else.
`Sensor::configure` passes `Report::RotationVector` = `0x05` (`BNO085Sensor.cpp:401–402`), so the
correct value is available. The host test asserts the wrong byte:
`test/native/test_main.cpp:457` checks `frame[5] == 0x00`.

**Functional consequence.** The hub is asked to configure feature `0x00`, `0x01`, … instead of
`0x05`. At best a hub error; at worst nothing is configured and no Rotation Vector ever streams.
This is the most plausible reason stage 3 is the first unverified stage in `PHASE1-NOTES.md`.

**Regression test.** In `testSh2SetFeatureBytes` (`test/native/test_main.cpp:445–464`):

```cpp
CHECK(frame.size() == 21);
CHECK(frame[4] == 0xFD);
CHECK(frame[5] == 0x05);   // featureReportId, not a counter
CHECK(frame[6] == 0x00);   // flags
CHECK(frame[9] == 0x60 && frame[10] == 0xEA);  // 60000 LE
CHECK(frame[17] == 0x00 && frame[18] == 0x00 &&
      frame[19] == 0x00 && frame[20] == 0x00); // sensorSpecific
```

Call `setFeature` twice and assert byte 1 stays `0x05` (no counter drift).

### 12.2 Set-feature acknowledgement — divergence confirmed, device behaviour uncertain

**Implementation.** `setFeature` sets `cmdPending_ = true`, `cmdId_ = 0xFD`,
`cmdDeadlineUs_ = now + 100 ms` (`Sh2Client.cpp:420–424`). `StageSetFeature`
(`BNO085Sensor.cpp:258–281`) only leaves the stage when `cmdState() == Ok`, which requires an
`0xF1` whose `command == 0xFD` (`handleCommandResponse`, `Sh2Client.cpp:186–234`). Otherwise the
100 ms stage window fails the sensor.

**Reference.** `setSensorConfigStart` ends with:

```c
rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
opCompleted(pSh2, rc);          // completes immediately on send result
return rc;
```

and `const sh2_Op_t setSensorConfigOp = { .start = setSensorConfigStart };` declares no `.rx`.
`opRx` (`sh2.c:350–356`) only dispatches when an operation is in progress **and** has an `rx`
method — so the reference would ignore a `0xF1` for `0xFD` even if one arrived. The subsequent
`0xFE`/`0xFC` exchange is the real validation.

**Functional consequence and confidence.** The code↔reference divergence is certain. Whether this
hub firmware emits `0xF1` for `0xFD` is **not proven**; there is no capture in the repo. An absent
ack is fatal; a present ack is harmless because `handleCommandResponse` only logs a `commandSeq`
mismatch (211–216) and does not reject it. Pass 1's “very likely never to complete” is therefore a
hypothesis, not evidence.

**Regression test.** A state-machine test that feeds **no** `0xF1`: after `setFeature` returns
true and the stage deadline passes, feed an `0xFC` get-feature response and assert the sensor
reaches `Streaming`. Today that test fails (the stage goes `Failed` first); it is the proof needed
once the ack policy changes.

### 12.3 Timestamp reference/rebase reports — confirmed

**IDs and signedness.** `Sh2Codes.h:17–18` swaps `SENSORHUB_BASE_TIMESTAMP_REF (0xFB, u32)` and
`SENSORHUB_TIMESTAMP_REBASE (0xFA, i32)` from `sh2.c:293–303`. Because
`handleTimestampRebase` (`Sh2Client.cpp:258–272`) reads `int32_t`, the report it treats as a
rebase is the **unsigned** base reference.

**Scope and sign.** Reference `sensorhubInputHdlr` (`sh2.c:542–591`) resets
`referenceDelta = 0` per payload (549), assigns `referenceDelta = -rpt->timebase` for a base
reference (567), accumulates `referenceDelta += rpt->timebase` for a rebase (572), and computes
`timestamp += hostInt + (referenceDelta + delay) * 100` (`touSTimestamp`, 524–540). Both fields
are 100 µs units.

**Channel.** Reference processes both in the input handler (registered on `inputNormal`/`inputWake`,
`sh2.c:1736–1737`). Implementation handles `kTimestampRebase` (i.e. `0xFB` today) only in
`handleControl` (`Sh2Client.cpp:114–133`) and only logs `kBaseTimestampRef` (`0xFA`).
`handleReports` (`274–298`) decodes only `0x05`; everything else goes to
`handlers_.sensorReport`, which is `nullptr` (`BNO085Sensor.cpp:67`).

**Correction to pass 1:** `0xFA`/`0xFB` on channels 3/4 do **not** abort parsing. Both have
length 5 in `kReportLengths` (`Sh2Codes.cpp:62–63`), so the cursor advances and the rest of the
payload still decodes. They are silently skipped.

**Functional consequence.** With real hardware `rebaseOffsetUs_` is never updated at all; the
control-channel cases are unreachable in normal operation. `computeMeasTime`
(`Sh2Client.cpp:377–383`) substitutes `-12 000 µs` (`kTimebaseReferenceUs`, line 13) for
`-timebase*100`, which is only correct when the hub's base timebase is 120 ticks — otherwise every
`tMeasUs` has a systematic bias. `epoch_` increments on the base-reference ID instead of the real
rebase. If a genuine `0xFB` ever arrived on channel 2, an unsigned base timestamp parsed as
`int32_t` × 100 would saturate the offset at ±`INT32_MAX`.

**Regression test.** Feed a channel-3 payload `{0xFB, timebase u32 LE, 0x05, RV(14)}` at a fixed
`tEdge`; assert `tMeasUs == tEdge - timebase*100 + delay*100`. Then
`{0xFB, tb, 0xFA, +N i32, 0x05, RV, 0x05, RV}` and assert the delta `-tb + N` applies to the
reports in **that payload only**, with the next payload falling back to the documented default.
Also assert `0xFB` on channel 2 no longer mutates the offset after the fix.

### 12.4 INT timestamp semantics — confirmed mismatch

**Implementation.** `Esp32SpiHal::intAsserted()` (`Esp32SpiHal.cpp:91–95`) is
`digitalRead(intPin) == LOW`; there is no `attachInterrupt` in `src/`. `Sensor::serviceBus`
(`BNO085Sensor.cpp:165–180`) latches `intEdgeUs_ = nowUs` where `nowUs` is `startUs` from
`Sensor::update` (139–146). `shtp_.service(intEdgeUs_)` (189) stamps every packet drained in that
visit with the same first-observation time. `Sh2Client` copies it through unchanged:
`q.tHostUs = tUs` (`Sh2Client.cpp:325`) and `q.tMeasUs = computeMeasTime(tUs, …)` (324), so
`tMeasUs` inherits the lateness.

**Documentation.** `README.md:121`; `docs/API.md:179` and `:378`; `docs/ARCHITECTURE.md:169`,
`:306–312` (explicit `attachInterrupt(..., FALLING)` and `edgeUs[i] = micros()`), and `:436` — all
state that `tHostUs` is the value at H_INTN assertion.

**Functional consequence.** Both exported timestamps are late by the time between the assertion
and the next `update()`, worst case one application loop period plus any blocking work earlier in
that iteration. `diag_.lastIntUs` and `diag_.lastReportUs`, and therefore the silence check
(`BNO085Sensor.cpp:334`), use the same value. There is no data loss from polling — H_INTN is level
asserted until CS is observed — so this is a fidelity problem, not a missed-interrupt problem.

**Regression test.** With `MockHal`, set `intState = true` and advance `clockUs` by 7 000 µs
before `update()`; assert `q.tHostUs` equals the poll time, 7 000 µs later than the simulated
assertion. After an ISR implementation, add an edge-injection hook and assert `q.tHostUs` equals
the recorded edge. A docs-only resolution is “fixed” by amending the three documents.

### 12.5 Recovery call sites — confirmed

**Nothing calls `recover()`.** Exhaustive `enterFailed` sites in `BNO085Sensor.cpp`: 236 (boot
timeout), 251 (identify timeout), 272/274 (set-feature hub error/timeout), 280 (set-feature
window), 294/296 (get-feature hub error/timeout), plus 374 inside `recover()` itself. All set
`State::Failed` and emit `INIT_RESULT` + `ERROR` (505–516). The only call to `recover()` in the
repository is `examples/01_SingleSensor/01_SingleSensor.ino:97` (the `r` key).

**`update()` makes `Failed` terminal.** Lines 139–144 return immediately for `Failed`, so a failed
sensor stays failed until the application intervenes.

**`recover()` does not reset attempts.** `recover()` (369–394) increments
`recoveryAttempts_` and never resets it, contradicting `ARCHITECTURE.md:410` (“it resets the
attempt counter”). Attempts are reset only in `begin()` (82–83) and `enterStreaming()` (314–315),
so a device that never reaches streaming exhausts the budget after five manual calls and the sixth
goes to `enterFailed(BootTimeout)` permanently.

**Silence does not recover either.** `checkSilence` (329–346) only sets `Degraded` and emits one
`ERROR`; `Degraded` self-clears on the next report (434–439), so a device that stops forever stays
`Degraded` forever.

**Functional consequence.** Any transient boot or SH-2 timeout is a permanent outage in an
unattended system. The documented `Streaming → Recovering → Reconfiguring → Streaming` loop with
backoff and phase offset (`ARCHITECTURE.md:400–414`) does not exist.

**Regression test.** Empty `MockHal` queue, advance past `bootTimeoutMs`, assert `State::Failed`;
then queue a valid boot packet and call `update()` 1 000 times, asserting the state stays `Failed`
(locks in today's contract; invert it when auto-recovery is added). Second test: call `recover()`
six times, assert the sixth returns `SensorFailed` and `State::Failed`; after the fix assert the
counter resets and the sensor retries.

### 12.6 Output loss — confirmed

**Trace.** `BinaryEncoder::emit` (`BinaryEncoder.cpp:25–65`):

```cpp
if (sink_ == nullptr) { ++dropped_; return false; }             // 27–30
if (len > kMaxPayloadBytes) { ++dropped_; ... return false; }    // 31–36
...
protocol::putU16(frame_ + kSequenceOffset, sequence_++);         // 43, before the write
...
const size_t written = sink_->write(frame_, total);
if (written != total) { ++dropped_; log(...); return false; }     // 53–61
```

Every caller ignores the return: `BNO085Sensor.cpp:85` (`writeHello`), `319` (`writeInitResult`),
`339` (`writeError`), `353` (`writeStatus`), `442` (`writeQuaternion`), `508`
(`writeInitResult`), `511` (`writeError`). `framesDropped()` (`BinaryEncoder.h:25`) is never
referenced. `ErrorCode::OutputOverflow` (`BNO085Types.h:93`) is produced nowhere; it appears only
in `errorName` (`BNO085Types.cpp:72`).

**Two subtleties.**

- `sequence_++` happens **before** the write, so a dropped frame still consumes a sequence number.
  A host that survives sees a gap, but the wire format does not otherwise announce the loss and no
  `ERROR` frame is emitted.
- When `emitBinaryFrames` is false, `begin()` attaches `nullptr` (`BNO085Sensor.cpp:56`), so
  `writeHello` and every `enterFailed` `INIT_RESULT`/`ERROR` increment `dropped_` even though
  nothing was meant to be written. As it stands `dropped_` means “write not performed”, not
  “frame lost”, so exposing it without this fix would be misleading.

**Functional consequence.** Silent data loss; the consumer sees a sequence gap indistinguishable
from a sensor dropout, with no counter, status bit or error frame to disambiguate.

**Regression test.** A sink that accepts a configurable number of bytes and then short-writes.
Assert `writeQuaternion` returns false, `framesDropped() == 1`, `sequence_` advanced by 1, and no
`ERROR` frame (today's behaviour). After the fix, assert an `ERROR(OutputOverflow)` or a surfaced
`Sensor::framesDropped()` while the sink is healthy again. Add a companion test that
`emitBinaryFrames == false` leaves the counter at 0.

### 12.7 Prioritized confirmed defects

1. **P1 — Set-feature byte 1 is a command counter, not `0x05`** (`Sh2Client.cpp:405–413`,
   `(void)reportId` at 431). Blocks the only configuration path; the host test enshrines the bug
   (`test_main.cpp:457` asserts `frame[5] == 0x00`).
2. **P2 — Set-feature completion depends on an `0xF1` ack** (`BNO085Sensor.cpp:267–281`).
   Reference treats `0xFD` as fire-and-forget. Even with P1 fixed, this can hang the state
   machine.
3. **P3 — `0xFA`/`0xFB` transposed, mishandled and on the wrong channel** (`Sh2Codes.h:17–18`;
   `Sh2Client.cpp:114–133`, `258–272`, `274–298`; `BNO085Sensor.cpp:67`). Device-time
   reconstruction is wrong and `epoch` never marks a discontinuity.
4. **P4 — `tHostUs`/`tMeasUs` are poll-observation times, documented as INT-assertion times**
   (`BNO085Sensor.cpp:165–171`, `Sh2Client.cpp:324–325`; docs `README:121`, `API:179,378`,
   `ARCHITECTURE:169,306–312,436`). Either the code or six documentation statements must change.
5. **P5 — No fault-triggered recovery; `recover()` does not reset its attempt budget**
   (`BNO085Sensor.cpp:139–144`, `369–394`; `ARCHITECTURE.md:410`). Any transient fault is terminal;
   a device that never re-streams is permanently `Failed` after five manual retries (a successful
   recovery resets the counter in `enterStreaming()`).
6. **P6 — Output loss is silent and the drop counter is not even a loss counter**
   (`BinaryEncoder.cpp:27–61`; `BinaryEncoder.h:25`; `BNO085Types.h:93`).

### 12.8 Proposed minimal repair sequence

1. **Byte 1 (P1).** Replace `payload[1] = sequence;` with `payload[1] = reportId;`, delete
   `(void)reportId;`, and stop deriving `cmdSeq_` from the payload field (keep the transport
   sequence as-is). Re-baseline `testSh2SetFeatureBytes`. One-line change, no dependency on
   anything else.
2. **Ack policy (P2).** Decide with evidence: either capture the control channel for 100 ms after
   a real `0xFD`, or match the reference and complete `StageSetFeature` as soon as
   `transport_->send` succeeds, treating `0xFE`/`0xFC` as validation. Keep accepting a stray
   `0xF1` if it arrives. Add the “init completes without `0xF1`” test.
3. **Timestamps (P3).** Swap the two constants; handle both IDs in `handleReports` with a
   per-payload local delta; store the base reference and use `-timebase*100` in `computeMeasTime`,
   retaining the `-12 000` fallback only for payloads before any base reference; increment `epoch_`
   on `0xFA`, not `0xFB`. Add the golden channel-3 tests. After steps 1–2, because it cannot be
   validated without streaming.
4. **INT (P4).** Choose: implement the documented ISR (falling edge → volatile pending flag +
   `edgeUs`, level re-check retained) or amend `README`/`API`/`ARCHITECTURE` to say
   “poll-observation time”. If implementing, extend `MockHal` with an edge hook and assert against
   it. Independent of 1–3.
5. **Recovery (P5).** Wire the `enterFailed` paths (or `update()` while `Failed`) into `recover()`
   with the existing `RecoveryPolicy` limits and backoff, and reset `recoveryAttempts_` in
   `recover()` per the doc — or amend `ARCHITECTURE.md:400–414` to state that recovery is
   application-driven in Phase 1. Invert the “stays Failed” test into “auto-retries after backoff”.
6. **Output loss (P6).** Attach `NullOutputSink` instead of `nullptr` so `dropped_` only counts
   real short writes; propagate `write*` returns into `Sensor`; expose `framesDropped()`; emit one
   rate-limited `ERROR(OutputOverflow)` per loss episode that suppresses itself if the sink is
   still failing. Add the short-write test last — it needs the surfaced counter from this step.

Rationale: 1–2 unblock bring-up; 3–4 fix data correctness; 5 addresses availability; 6 addresses
observability. Each step is independently testable.

### 12.9 Limits of this verification

- No hardware was exercised. Three points remain unproven: (a) whether this hub acks `0xFD`,
  (b) the magnitude of real base timestamps, (c) actual INT-to-poll latency. They are the only
  findings marked uncertain.
- Pass-1 statements corrected in pass 2: the input-channel `0xFA`/`0xFB` skip does **not** abort
  payload parsing (both have lengths), and “set-feature is very likely never to complete” is a
  hypothesis, not a measurement.
- Everything else in §0–§11 held under independent re-reading.

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
| `tMeas` | `tEdge + (referenceDelta + delayTicks) × 100` | `Sh2Client.cpp:13–14, 376–382` |
| `kDefaultReferenceDeltaTicks` | −120 ticks (−12 000 µs); fallback until a `0xFB` arrives | `Sh2Client.cpp:13` |
| `kTimestampTickUs` | 100 µs per tick | `Sh2Client.cpp:14` |
| Reference base timestamp | `0xFB`, u32, `referenceDelta = -timebase` | `sh2.c:293–298, 563–567` |
| Reference timestamp rebase | `0xFA`, i32, `referenceDelta += timebase` | `sh2.c:299–303, 569–572` |

### Appendix C — verification checklists

| Step | Code | Reference behaviour | Check |
| --- | --- | --- | --- |
| Set-feature payload | `Sh2Client.cpp:399–429` | 17 bytes, byte 1 = feature ID | **done (pass 3):** byte 1 = `reportId` |
| Set-feature completion | `BNO085Sensor.cpp:258–281` | completes on `sendCtrl`, no `.rx` | **done (pass 3):** completes on send; stray ack ignored |
| Base timestamp | `Sh2Client.cpp:266–275` inside `handleReports` | input payload, `0xFB`, u32, negated | **done (pass 3):** per-payload delta |
| Timestamp rebase | `Sh2Client.cpp:276–286` inside `handleReports` | input payload, `0xFA`, i32, accumulated | **done (pass 3):** per-payload, `epoch_` here |
| INT capture | `BNO085Sensor.cpp:165–171` | ISR edge per docs | `tHostUs` is poll time today (**open**) |
| Recovery entry | `BNO085Sensor.cpp:369` (manual only) | fault → Recovering | `Failed` is terminal today (**open**) |
| Output loss | `BinaryEncoder.cpp:53–61` | caller decides | returns are ignored today (**open**) |

---

## 13. Implementation pass (pass 3) — P1–P3 applied

**Scope.** Only the two highest-priority protocol defects from §12.7 were changed: **P1/P2**
(set-feature payload and completion) and **P3** (timestamp reference/rebase). **P4** (INT polling),
**P5** (recovery) and **P6** (output loss) were deliberately left untouched. There is no Phase 2
work, no `Manager`, and no refactoring beyond what the two fixes required. The existing architecture
(one `Sensor` → one `ShtpTransport` → one `Sh2Client` → one `BinaryEncoder`) is unchanged.

### 13.1 Set-feature (P1/P2) — fixed

**Reference behaviour.** `SetFeatureReport_t` is 17 bytes:
`reportId(0xFD), featureReportId, flags, changeSensitivity(u16), reportInterval(u32),
batchInterval(u32), sensorSpecific(u32)`. `setSensorConfigStart` calls `sendCtrl` and then
`opCompleted` immediately; `setSensorConfigOp` declares no `.rx` handler, so `opRx` drops a
`0xF1` for `0xFD` even if the device sends one.

**Changes** (`src/sh2/Sh2Client.cpp`, `setFeature`, now lines 399–432):
- `payload[1] = reportId;` (line 406). `nextCmdSeq_` and `(void)reportId` are gone; byte 1 no
  longer changes between calls.
- Completion is send success: on a successful `transport_->send` the function sets
  `cmdState_ = Ok` and `cmdPending_ = false` (lines 422–423). No acknowledgement deadline is armed.
- A stray `0xF1` for `0xFD` therefore reaches `handleCommandResponse` as an unsolicited response:
  it increments `stats_.unexpectedResponses` and cannot fail or complete anything.
- `src/sh2/Sh2Client.h` no longer declares `nextCmdSeq_`. `cmdId_`, `cmdSeq_` and
  `cmdDeadlineUs_` remain as dormant bookkeeping; removing them would touch unrelated code.

`Sensor::advanceInit` still contains its `StageSetFeature` `HubError`/`Timeout` branches
(`BNO085Sensor.cpp:271–278`). They are now unreachable but harmless, and were left in place to keep
the state machine untouched.

### 13.2 Timestamps (P3) — fixed

**Reference behaviour** (`sensorhubInputHdlr` / `touSTimestamp`): the delta is per payload;
`0xFB` **assigns** `referenceDelta = -timebase`; `0xFA` **accumulates** `referenceDelta +=
timebase`; `tMeas = hostInt + (referenceDelta + delay) * 100`.

**Changes:**
- `src/sh2/Sh2Codes.h:17–18`: `kBaseTimestampRef = 0xFB` (u32) and
  `kTimestampRebase = 0xFA` (i32).
- `src/sh2/Sh2Client.cpp` `handleReports` (248–297): both IDs are handled in the input-report
  stream (channels 3/4). A per-payload `int64_t referenceDeltaTicks` starts at the retained
  fallback `-120`; `0xFB` replaces it with `-timebase`; `0xFA` adds the sign-extended `timebase`.
  The delta does not persist into the next payload.
- `handleRotationVector` and `computeMeasTime` take `referenceDeltaTicks`
  (`Sh2Client.cpp:299–300`, `376–382`). `tMeas = tEdgeUs + (referenceDeltaTicks + delayTicks) * 100`.
- `rebaseOffsetUs_`, `kTimebaseReferenceUs`, `kRebaseUnitUs` and `handleTimestampRebase` were
  removed. `handleControl` no longer has `0xFA`/`0xFB` cases.
- `epoch_` increments on `0xFA` and on reset-complete only; a `0xFB` base reference is not a
  discontinuity.
- Intermediate arithmetic is `int64_t`, so a large timebase cannot saturate or overflow.

### 13.3 Regression tests

`test/native/test_main.cpp` now has **35 cases / 194 checks**. New or re-baselined cases:

| Test | Line | Asserts |
| --- | --- | --- |
| `sh2_set_feature_bytes` | 453 | byte 1 = `0x05`, sensorSpecific zero, `Ok`/`!busy`, no counter drift on a second call |
| `sh2_set_feature_no_ack_wait` | 636 | completes without `0xF1`; a later timeout poll changes nothing |
| `sh2_late_ack_ignored` | 650 | a stray `0xF1` is counted unsolicited, not treated as success or failure |
| `sh2_base_timestamp_ref` | 524 | `0xFB` 200 ticks → `tMeas = tEdge − 20 000 µs`; `epoch` unchanged |
| `sh2_timestamp_rebase` | 539 | `0xFA` +50 accumulates onto the same payload's delta |
| `sh2_timestamp_rebase_signed` | 558 | `0xFFFFFFCE` sign-extends to −50 |
| `sh2_timestamp_per_payload` | 574 | the delta does not leak into the next payload |
| `sh2_timestamp_control_ignored` | 592 | `0xFB` on channel 2 has no effect |
| `sensor_init_sequence` | 669 | init advances without an `0xFD` acknowledgement |

`makeTimestampReport` (line 102) builds the 5-byte `0xFA`/`0xFB` reports.

### 13.4 Evidence

| Check | Command | Result |
| --- | --- | --- |
| Host tests (MSVC 2022) | `test\native\build_msvc.bat` | 35 cases, 194 checks, 0 failures, `RESULT: PASS` (`build\native_tests.log`) |
| ESP32-S3 compile | `arduino-cli compile --clean --verbose --warnings all --fqbn esp32:esp32:esp32s3 --library . examples/01_SingleSensor` | exit 0, 0 warnings, 0 errors; 324 210 B flash (24%), 23 880 B RAM (7%) (`build\esp32s3_final.log`) |
| GCC host build | `sh test/native/build_gcc.sh` | **not run**: no `sh` or `g++` in this environment |

The ESP32-S3 numbers are 52 bytes smaller than the §10 pre-fix figure (324 262 B). The host suite
was also run immediately before this document update; nothing in the documentation change touches
the build.

### 13.5 Remaining limitations

- **No hardware validation.** Nothing in this pass was exercised on a BNO085. Still unproven:
  (a) whether this hub acknowledges `0xFD`, (b) the magnitude of real `0xFB` timebases,
  (c) actual INT-to-poll latency. The fixes remove a protocol divergence; they do not prove the
  sensor streams.
- **Fallback delta.** A payload that arrives before any `0xFB` uses `-120` ticks (−12 ms), retained
  from Phase 1 to preserve prior behaviour. The reference starts at 0, so this is an approximation,
  not protocol-exact. It affects only the start of a stream (or a payload with no reference).
- **Timestamp statistics.** `stats_.baseTimestampRefs` counts every `0xFB` report; no separate
  rebase counter exists. This is observability, not correctness.
- **Dormant bookkeeping.** `cmdId_`, `cmdSeq_`, `cmdDeadlineUs_`, and the `StageSetFeature`
  `HubError`/`Timeout` branches remain. They are unreachable and intentionally untouched.
- **Unchanged findings.** P4 (INT is poll-observation, documented as INT assertion), P5 (no
  fault-triggered recovery; attempt counter not reset) and P6 (silent output loss) remain exactly as
  described in §12.4–§12.6. §11's list is otherwise still the backlog.

### 13.6 Files changed in this pass

| File | Change |
| --- | --- |
| `src/sh2/Sh2Client.cpp` | set-feature byte 1 and send-completion; input-channel timestamp handling; `computeMeasTime` signature and per-payload delta; removed rebase offset and control-channel cases |
| `src/sh2/Sh2Client.h` | removed `nextCmdSeq_`, `rebaseOffsetUs_`, `handleTimestampRebase`; `int64_t referenceDeltaTicks` parameters |
| `src/sh2/Sh2Codes.h` | corrected `kBaseTimestampRef` / `kTimestampRebase` values |
| `test/native/test_main.cpp` | re-baselined set-feature bytes; new set-feature, ack and timestamp tests; removed ack dependency from the init sequence |
| `output.md` | this pass-3 record and status notes |

**Commit.** `25deebf` — "Fix SH-2 set-feature payload and timestamp reconstruction". It contains the
four source/test files above; the following commit records this document.
