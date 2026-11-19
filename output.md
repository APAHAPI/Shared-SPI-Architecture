# BNO085Multi — Phase 1 source audit (current HEAD)

**Purpose.** Hand-off document for the next agent. It records what the code on the current
branch actually does, what was executed in this audit, and every place where the
implementation and the documents disagree.

> **This file replaces the earlier pass 1–4 narrative.** The old document was anchored to
> commit `d040c8e`, described pass-4 changes as uncommitted, and kept pre-fix text for
> set-feature and timestamp handling behind "status" notes. That text is superseded. This
> audit is derived from the current source at `e9a41ec` and does not rely on the earlier
> document's conclusions.

| Item | Value |
| --- | --- |
| Repository root | `C:\Users\Apahapi\Desktop\Source codes\Shared SPI Architecture` |
| Branch | `main`, up to date with `origin/main` |
| HEAD | `e9a41eca6b77e947840708d67bd7d15fe164a359` — "Match SH-2 timestamp reference semantics" |
| Working tree at audit start | clean (no modified, staged or untracked source files) |
| Library / version | `BNO085Multi` 0.1.0 (`library.properties`) |
| Stage | Phase 1 — single-sensor baseline, ESP32-S3, Arduino core 3.3.12 |
| Audit date | 2026-11-19 |
| Source changes in this audit | **none** |
| Document changes in this audit | this file only |
| Hardware | ESP32-S3 and BNO085 **disconnected**; COM11 unavailable. No physical evidence was produced or re-used as new evidence. |

**Evidence classes used below.** Every claim is labelled as one of: *source-verified* (read
from the current tree), *host-tested* (executed in this audit), *historical* (recorded in the
repository before this audit, not re-executed), or *unverified* (requires hardware or a
toolchain that is not present).

---

## 1. Verdict summary

| Area | Verdict at `e9a41ec` |
| --- | --- |
| Set Feature payload | **Source-verified correct** against the Hillcrest/CEVA `sh2.c` layout. 17-byte payload, 21-byte SHTP transfer, byte 1 is the feature report ID. Host test asserts the complete golden frame, including a second call's SHTP sequence. |
| Set Feature completion | **Source-verified deliberate:** completes on a successful send, matching the reference `setSensorConfigStart`/`setSensorConfigOp` (no `.rx`). A stray `0xF1` for `0xFD` is counted as unsolicited and cannot fail the request. The hub's real ack behaviour is still unproven. |
| Timestamp reports | **Source-verified correct** in ID, signedness, units and channel: `0xFB` = u32 base reference, `0xFA` = i32 rebase, both handled per input payload on channels 3/4. `0xFB` assigns `-timebase`; `0xFA` accumulates. The delta is local to one payload and starts at 0. |
| `tMeasUs` arithmetic | `tMeas = (uint32_t)(tEdge + (referenceDeltaTicks + delayTicks) * 100)`. No invented clamp; wraps with the 32-bit host counter. Host-tested for the known example, zero fallback, host wrap and negative-result wrap. |
| SHTP normal fragments | **Implemented but not positively host-tested.** Continuation fragments ≤256 B are assembled in `rxPayload_` (192 B) and delivered once complete. Only the rejection path and the oversize path have tests. |
| SHTP oversize drain | **Implemented and host-tested**, including the 552-byte advertisement and a 2-fragment oversized continuation chain. The drain consumes the full declared length and leaves the mock stream idle. |
| Initialization FSM | Boot → Identify → SetFeature → GetFeature → Streaming works on the mock and is host-tested end to end, including a boot with no `0xF1` ack. |
| Recovery | **Manual only.** Nothing calls `recover()` on a fault; `update()` makes `Failed` terminal; `recover()` does **not** reset the attempt counter. This contradicts `docs/ARCHITECTURE.md` §7.4. |
| Output | CRC-framed quaternion output is correct and host-tested. Short writes and null-sink writes are silently counted in `dropped_`; the return values are ignored and no `ERROR(OutputOverflow)` is emitted. |
| Host tests | **39 cases / 185 checks, 0 failures, exit 0** — executed in this audit. |
| ESP32-S3 compile | **Not executed in this audit** (`arduino-cli` is not on `PATH`). The last recorded figure is historical: 324,174 B flash / 23,880 B static RAM (pass 4). |
| Physical validation | **None in this audit.** Product ID, set/get feature, RV streaming, timestamp reconstruction on a real stream, soak, INT-loss and cold boots remain unmeasured. |

---

## 2. Evidence actually collected

| Check | Command | Exit | Result |
| --- | --- | --- | --- |
| Repository state | `git status`; `git branch --show-current`; `git rev-parse HEAD` | 0 | `main`, HEAD `e9a41ec`, working tree clean at start |
| Native host tests | `cmd /c "test\native\build_msvc.bat"` (output to `build\native_audit_2026-11-19.log`) | **0** | **39 cases, 185 checks, 0 failures, `RESULT: PASS`** |
| ESP32-S3 compile | not run | — | `arduino-cli` not found on `PATH` in this environment |
| MSVC `/analyze` | not run | — | not executed in this pass |
| MSVC AddressSanitizer | not run | — | previously failed to link because the ASan runtime is not installed; not retried |
| GCC host build | not run | — | no `g++`, `sh` or `clang++` on `PATH` |
| Hardware | not attempted | — | board disconnected, COM11 unavailable |

No test result, compile figure or hardware observation from an earlier pass is presented here
as if it had been re-executed. Where an older figure is mentioned, it is labelled
*historical*.

---

## 3. Set Feature (`Sh2Client::setFeature`)

### 3.1 Wire layout

`src/sh2/Sh2Client.cpp:401–434` builds a 17-byte payload, zero-filled first
(`Sh2Client.cpp:404–405`), then fills:

| Payload offset | Size | Field | Value in code | Source line |
| --- | --- | --- | --- | --- |
| 0 | 1 | `reportId` | `0xFD` | 407 |
| 1 | 1 | `featureReportId` | `reportId` argument (Sensor passes `Report::RotationVector` = `0x05`) | 408 |
| 2 | 1 | `flags` | `0x00` (no change sensitivity, wake or always-on) | 409 |
| 3–4 | 2 | `changeSensitivity` | 0 | 410 (zero-fill) |
| 5–8 | 4 | `reportInterval_uS` | `intervalUs`, little-endian | 411–414 |
| 9–12 | 4 | `batchInterval_uS` | 0 | 415 |
| 13–16 | 4 | `sensorSpecific` | 0 | 415 |

The payload is sent on the SHTP control channel (`shtp::kChannelControl` = 2,
`ShtpProtocol.h:12`), so the transfer is 4 header bytes + 17 payload = **21 bytes**.
`ShtpTransport::send` adds only the 4-byte SHTP header (`ShtpTransport.cpp:299–332`); no
per-payload command sequence is inserted.

This matches the reference `SetFeatureReport_t` field-for-field. There is no command-sequence
field in that report; the only sequence on the wire is SHTP's per-channel sequence.

### 3.2 Completion semantics

`setFeature` sets `cmdState_ = CmdState::Ok` and `cmdPending_ = false` immediately after a
successful `transport_->send` (`Sh2Client.cpp:421–425`). This follows
`setSensorConfigStart` in the reference: it calls `sendCtrl` then `opCompleted`, and
`setSensorConfigOp` declares no `.rx` handler. The granted interval is validated by the
following get-feature exchange.

Consequences, all source-verified:

- No ack deadline is armed, so a missing `0xF1` cannot time out the request.
- A stray `0xF1` for `0xFD` reaches `handleCommandResponse` with `cmdPending_ == false`
  (`Sh2Client.cpp:182–189`), increments `stats_.unexpectedResponses`, and changes nothing.
- `cmdId_`, `cmdSeq_` and `cmdDeadlineUs_` still exist (`Sh2Client.h:110–113`) but are now
  dormant: `cmdPending_` is never set to true after `setFeature`, so the command-timeout
  branch in `pollTimeouts` (`Sh2Client.cpp:476–487`) is unreachable.
- `Sensor::advanceInit` still has `HubError`/`Timeout` branches for set-feature
  (`BNO085Sensor.cpp:271–274`). They are unreachable but harmless.

### 3.3 Host test

`testSh2SetFeatureBytes` (`test/native/test_main.cpp:453–482`) asserts:

- the complete first frame equals
  `15 00 02 00 FD 05 00 00 00 60 EA 00 00 00 00 00 00 00 00 00 00`
  (21 bytes, control channel, SHTP sequence 0, feature ID `0x05`, interval 60000 LE);
- `cmdState() == Ok` and `!busy()` after the send;
- a second call keeps byte 1 at `0x05` and advances only the SHTP sequence (`0x01`) and the
  interval bytes (`10 27 00 00`).

`testSh2SetFeatureNoAckWait` (`:684–696`) proves that a missing ack does not time out.
`testSh2LateSetFeatureAckIgnored` (`:698–708`) proves that a late `0xF1` is counted
unsolicited, not treated as success or failure.

**Residual uncertainty.** Whether this hub firmware acknowledges `0xFD` at all cannot be
established without hardware. The source no longer depends on the answer, which is the point
of the change.

---

## 4. Timestamp reference and rebase

### 4.1 Report IDs and channel

`src/sh2/Sh2Codes.h:17–18`:

```cpp
constexpr uint8_t kBaseTimestampRef = 0xFB;  // device -> host, u32 timebase
constexpr uint8_t kTimestampRebase  = 0xFA;  // device -> host, i32 timebase
```

Both IDs have a 5-byte entry in `kReportLengths` (`Sh2Codes.cpp:62–63`,
`kTimestampRefLen`). They are processed in `Sh2Client::handleReports`
(`Sh2Client.cpp:249–298`), which is the handler for the input-report channels 3 and 4
(`handlePayload`, `:64–67`). On the control channel they are length-skipped by
`handleControl` and only logged at debug level; they do not mutate any timestamp state.
A host test pins that behaviour (`testSh2TimestampOnControlChannelIgnored`, `:589–600`).

### 4.2 Per-payload delta

`handleReports` declares `int64_t referenceDeltaTicks = 0` at the start of **each** payload
(`Sh2Client.cpp:251–253`). Then, walking the concatenated reports:

- `0xFB` (`:269–278`): `referenceDeltaTicks = -static_cast<int64_t>(timebase)`; increments
  `stats_.baseTimestampRefs`; does **not** increment `epoch_`.
- `0xFA` (`:279–289`): `referenceDeltaTicks += static_cast<int32_t>(readU32(...))`;
  increments `epoch_`.
- `0x05` (`:290–291`): decode the Rotation Vector with the current delta.
- anything else: forwarded to `handlers_.sensorReport` if one is registered; `Sensor` leaves
  it `nullptr` (`BNO085Sensor.cpp:67`), so other reports are skipped but parsing continues
  because their lengths are known.

The delta does not persist into the next payload. This matches the reference
`sensorhubInputHdlr` (`referenceDelta = 0` per payload; `0xFB` assigns; `0xFA` accumulates).

### 4.3 Measurement-time arithmetic

`computeMeasTime` (`Sh2Client.cpp:377–385`):

```cpp
const int64_t t = static_cast<int64_t>(tEdgeUs) +
                  (referenceDeltaTicks + delayTicks) * kTimestampTickUs;
return static_cast<uint32_t>(t);
```

with `kTimestampTickUs = 100` (`:15`). There is no negative clamp and no `-12000 µs`
constant. The result keeps the low 32 bits, so it wraps exactly like the host `micros()`
counter. Intermediate arithmetic is `int64_t`, so a large `0xFB` timebase cannot saturate.

### 4.4 What `tHostUs` actually is

`q.tHostUs = tUs` (`Sh2Client.cpp:325`). `tUs` is the timestamp passed to the transport
handler, which `Sensor::serviceBus` latches from `hal_.timeUs()` on the **first poll
observation** of INT (`BNO085Sensor.cpp:165–174`) and reuses for every packet drained in that
visit (`:189`). It is therefore **poll-observation time, not hardware H_INTN assertion
time**. See §7 and §8 for the documentation conflict.

### 4.5 Host tests

`test/native/test_main.cpp`:

| Test | Line | Asserts |
| --- | --- | --- |
| `sh2_base_timestamp_ref` | 520–533 | `0xFB` 200 ticks → `tEdge − 20 000 µs`; `epoch` unchanged |
| `sh2_timestamp_rebase` | 535–552 | `0xFB` then `0xFA +50` → delta `−150`; `epoch` increments |
| `sh2_timestamp_rebase_signed` | 554–568 | `0xFFFFFFCE` sign-extends to −50 |
| `sh2_timestamp_per_payload` | 570–587 | the delta does not leak into the next payload |
| `sh2_timestamp_control_ignored` | 589–600 | `0xFB` on channel 2 has no effect |
| `sh2_timestamp_known_example` | 602–614 | `1 000 000 + (−120 + 150) × 100 = 1 003 000` |
| `sh2_timestamp_no_reference` | 616–624 | no reference → delta 0 → `tEdge` unchanged |
| `sh2_timestamp_host_wrap` | 626–637 | host-counter wrap → low 32 bits (744) |
| `sh2_timestamp_before_zero` | 639–651 | negative result → two's-complement low 32 bits |

All pass in the executed suite.

**Residual uncertainty.** The magnitude of real `0xFB` timebases, the real 100 µs delay
scale, and the INT-to-poll latency remain unmeasured. There is no separate rebase counter in
`Sh2Client::Stats`; only `baseTimestampRefs` exists (`Sh2Client.h:38–47`).

---

## 5. SHTP framing, reassembly and draining

### 5.1 Receive path (`ShtpTransport::service`, `ShtpTransport.cpp:77–279`)

1. If `hal_->intAsserted()` is false, return `NoData` (`:78–79`).
2. Peek 4 bytes (`readRaw(peek, 4)`, `:81–86`). The design assumes the device re-presents
   the transfer from its header on a new CS assertion; the mock encodes that assumption
   (`test/native/mock_hal.h:58–62`).
3. Parse `rawLength` (`bit 15 = continuation`), `channel = peek[2]`, `sequence = peek[3]`
   (`:88–94`).
4. Reject/consume:
   - `length == 0` → `Spurious` (`:104–108`);
   - reserved `0xFFFF` → `badFragments++`, `discard(length)` (`:110–116`);
   - `length < 4` → `shortFragments++`, `discard` (`:118–124`);
   - `channel >= 8` → `badChannels++`, `discard` (`:126–132`).
5. Sequence tracking is diagnostic only (`:134–144`).
6. If `length > sizeof(rxTransfer_)` (256), take the oversize path (§5.3).
7. Otherwise read the full `length` bytes (header repeated + payload) into `rxTransfer_`
   (`:207–211`), count `packets`/`bytes`, and record `lastChannel`/`lastLength`/
   `lastTimestampUs` (`:213–217`).
8. Assemble payload (`:221–267`):
   - `payloadLen = length - 4`;
   - a continuation must have an assembly in progress, the same channel, and
     `payloadLen <= inRemaining_`, otherwise `badFragments++` and reset (`:223–233`);
   - a non-continuation transfer aborts any partial assembly and starts a new one
     (`:234–246`);
   - copy into `rxPayload_`; if `payloadLen` exceeds the remaining buffer space (192 B
     total), count `oversizePayloads`, reset and return `Oversize` (`:248–259`);
   - when `inRemaining_ == 0`, invoke the handler once with the assembled payload and the
     timestamp captured when the first fragment arrived (`:265–278`).

### 5.2 Normal fragments

The positive reassembly path exists and is source-verified. However, the current registry
contains **no positive two-fragment reassembly test**: `shtp_unexpected_continuation`
(`test_main.cpp:409–416`) tests rejection, and the two oversize tests exercise the drain,
not `rxPayload_` assembly. `test/native/README.md:34` claims continuation coverage; only
the rejection side is covered.

### 5.3 Oversize drain (`ShtpTransport.cpp:146–205`)

When the transfer itself is larger than 256 bytes:

- `oversizePayloads++` (`:151`);
- drain exactly `length` bytes in ≤256-byte chunks (`:162–176`). The 4-byte peek consumed
  nothing, so the whole transfer, header included, must be clocked out;
- if the continuation bit is set, peek the next 4-byte header, require the same channel and
  `lenNext >= 4`, then drain exactly `lenNext` bytes for that fragment (`:177–198`);
- bound the chain to 8 continuation fragments (`guard`, `:178–181`);
- return `Oversize` (`:204`).

The comment at `:158–161` records the original defect: draining `length - 4` stranded the
last four payload bytes, which the device re-presented, and every later `service()` parsed
payload as a header.

### 5.4 Host tests

`test/native/test_main.cpp`:

| Test | Line | Asserts |
| --- | --- | --- |
| `shtp_valid_rotation_vector` | 288–303 | one complete transfer, channel, length, timestamp, payload, packet count |
| `shtp_no_data` | 305–309 | INT low → `NoData` |
| `shtp_zero_length` | 311–317 | length 0 → `Spurious` |
| `shtp_short_fragment` | 319–325 | length < 4 → `Malformed`, `shortFragments == 1` |
| `shtp_reserved_length` | 327–332 | `0xFFFF` → `Malformed`, `badFragments == 1` |
| `shtp_bad_channel` | 334–341 | channel 9 → `Malformed`, `badChannels == 1` |
| `shtp_oversize_transfer` | 343–350 | 300-byte transfer → `Oversize`, nothing delivered |
| `shtp_payload_too_large` | 352–360 | 200-byte payload (fits transfer, not payload buffer) → `Oversize` |
| `shtp_oversize_drain_complete` | 362–390 | 552-byte advertisement fully consumed, queue empty, idle afterwards, next report decodes |
| `shtp_oversize_continuation` | 392–407 | two oversized fragments drained by their own lengths, idle afterwards |
| `shtp_unexpected_continuation` | 409–416 | continuation with no assembly → `Malformed` |
| `shtp_bus_error` | 418–424 | failed transfer → `BusError`, `rxErrors == 1` |
| `shtp_sequence_gap` | 426–435 | gap counted, not repaired |

### 5.5 Unhandled interactions and edge cases (source-verified, not tested)

- **Oversize while an assembly is in progress.** The oversize branch returns before touching
  `inRemaining_`/`inCursor_` (`:146–205`). If a payload has a small fragment followed by an
  oversized fragment, the stale assembly is not cancelled, and later fragments can be
  appended to it. No test covers this.
- **`discard()` is capped at 1024 bytes** (`4 × sizeof(rxTransfer_)`, `:67`). The reserved
  length is 32,767 bytes, so a genuinely long malformed transfer can leave bytes on the wire
  and desynchronise the next `service()`.
- **Continuation header mismatch.** If the next fragment has a different channel or
  `lenNext < 4`, the drain counts `badFragments` and breaks (`:190–193`). The peek is
  non-consuming by design, so the bytes remain for the next `service()`; recovery depends on
  the device re-presenting them correctly.
- **Two different conditions share one counter.** `oversizePayloads` counts both a
  transfer larger than 256 B (`:151`) and a payload larger than the remaining 192-byte
  buffer (`:251`).

---

## 6. Initialization, recovery and output

### 6.1 Initialization state machine

`Sensor::begin` (`BNO085Sensor.cpp:22–96`) validates only: `csPin >= 0`,
`report == RotationVector`, `intervalUs >= kMinIntervalUs` and `reportsValid()`,
`0 < spiHz <= kSpiMaxHz`. It does not validate a sensor count, distinct CS pins,
`csPin != intPin/resetPin`, or `resetGroup`; there is no `Manager` in Phase 1.

`update()` (`:139–163`) returns immediately for `Failed`, `Disabled` and `Unconfigured`.
Otherwise it services the bus, polls SH-2 timeouts, advances init, checks silence and emits a
periodic status frame.

`advanceInit` (`:222–307`):

| Stage | Condition | Action |
| --- | --- | --- |
| `StageBoot` | `shtp_.stats().packets > bootPacketBaseline_` | → `StageIdentify`, state `Identifying`, 100 ms deadline |
| `StageBoot` | boot deadline reached | `Failed(BootTimeout)` |
| `StageIdentify` | `productInfo().valid` | → `StageSetFeature`, state `Configuring` |
| `StageIdentify` | deadline reached | `Failed(Sh2Timeout)` |
| `StageIdentify` | not busy and retry due | send `0xF9 0x00`, retry every 10 ms |
| `StageSetFeature` | not yet sent | send set-feature; break |
| `StageSetFeature` | `cmdState() == Ok` | → `StageGetFeature` |
| `StageSetFeature` | `HubError` / `Timeout` | `Failed(Sh2BadResponse)` / `Failed(Sh2Timeout)` (unreachable after the send-completion change) |
| `StageSetFeature` | not busy and retry due | resend |
| `StageSetFeature` | stage deadline | `Failed(Sh2Timeout)` |
| `StageGetFeature` | not yet sent | send `0xFE 0x05` |
| `StageGetFeature` | `featureState() == Ok` | `enterStreaming()` |
| `StageGetFeature` | `HubError` or `Timeout` | `Failed(Sh2BadResponse)` |
| `StageGetFeature` | stage deadline | `Failed(Sh2Timeout)` |

`enterStreaming` (`:309–327`) records the granted interval, resets `recoveryAttempts_`,
sets `Streaming`, and emits `INIT_RESULT` with result 0.

Important behavioural detail: the boot stage advances on **any complete SHTP packet**. The
552-byte channel-0 advertisement is oversized, returns before `++stats_.packets`, and does
not count; in practice the reset-complete packet on channel 1 advances the stage. The state
machine does not parse the advertisement or a channel-2 initialisation message, and
`onResetComplete` returns early while in `StageBoot`/`StageIdentify` (`:446–450`).

### 6.2 Recovery

- `recover()` (`:369–394`) is manual. It returns `SensorFailed` once
  `recoveryAttempts_ >= maxAttempts`; otherwise increments the counter, sets `Recovering`,
  and schedules `startBoot()` after the backoff.
- `update()` treats `Failed` as terminal (`:141–144`), so a fault never triggers recovery by
  itself.
- `recover()` does **not** reset `recoveryAttempts_`. The counter is reset in `begin()`
  (`:82`) and `enterStreaming()` (`:314`). This contradicts `docs/ARCHITECTURE.md:410`
  ("`recover(i)` … resets the attempt counter").
- A successful `startBoot()` after backoff re-enters `StageBoot`; failures then go through
  the same `enterFailed` paths.
- `checkSilence` (`:329–346`) sets `Degraded`, records `NoReports` and emits one `ERROR`
  frame. `Degraded` self-clears on the next report (`:434–439`). No recovery is attempted.
- `resetOnBootTimeout`, `resetGroup`, per-index phase offset and automatic reconfiguration
  are not implemented.

### 6.3 Output encoder

`BinaryEncoder::emit` (`BinaryEncoder.cpp:25–65`) writes
`A5 5A | ver | type | len(u16) | seq(u16) | payload | CRC16(u16)`, with CRC-16/CCITT-FALSE
over `[2, 8+N)`. `writeQuaternion` (`:79–98`) emits the 24-byte payload documented in
`docs/API.md` §4.3 and is host-tested at the byte level (`test_main.cpp:242–286`).

Source-verified output gaps:

- Every `write*` return value is ignored by `Sensor`. A short write increments `dropped_` and
  logs, but no status bit or `ERROR(OutputOverflow)` is produced. The frame sequence is
  incremented *before* the write (`:43`), so a dropped frame still consumes a sequence
  number.
- When `emitBinaryFrames` is false, `begin()` attaches `nullptr` (`BNO085Sensor.cpp:56`), so
  `writeHello`, `INIT_RESULT` and `ERROR` increment `dropped_` even though nothing was meant
  to be written. `dropped_` therefore means "write not performed", not "frame lost".
- `writeManagerDiagnostics` (`BinaryEncoder.cpp:142–157`) exists but is never called.
- `SENSOR_STATUS` is emitted only by `emitStatusIfDue` (`BNO085Sensor.cpp:348–354`), i.e.
  periodically; it is not emitted on a state change, contrary to `docs/API.md:356`.
- Its payload offset 8 is `reportsDecoded` (`BinaryEncoder.cpp:123`), while `docs/API.md:415`
  documents `rxPackets`.
- `Diagnostics::product`, `csTransactions`, `spiTransfers`, `busTimeouts` and
  `transferBytes` are declared (`BNO085Types.h:200–212`) but never written. The example prints
  `product.partNumber` (`examples/01_SingleSensor/01_SingleSensor.ino:59–64`), which is
  therefore always zero.
- `SpiBusManager::lockTimeouts_` is counted (`SpiBusManager.cpp:102–104`) but never surfaced
  in `Diagnostics`.

### 6.4 Other source-verified notes

- `disable()` only sets `State::Disabled`; it sends nothing to the device, which continues
  streaming and asserting INT.
- `configure()` sends a new set-feature but does not itself run a get-feature to confirm the
  granted interval.
- `Sensor::end()` drives CS high and clears `started_`; it does not reset SHTP/SH-2 or the
  bus manager.
- `begin()` writes `HELLO` with `sensorCount = 1` hardcoded (`:85`).
- `cmdId_`, `cmdSeq_`, `cmdDeadlineUs_` are dormant (§3.2).
- The example prints `tHostUs - tMeasUs` as an unsigned value
  (`01_SingleSensor.ino:164–170`); it can underflow when `delay > 0` and no `0xFB` is present.
  This affects only the text diagnostic, not the binary record.

---

## 7. Source-verified defects and gaps (current, unfixed)

These are current facts at `e9a41ec`, not historical pass-1 findings.

| # | Gap | Evidence | Impact |
| --- | --- | --- | --- |
| D1 | INT is polled, not an ISR | `Esp32SpiHal::intAsserted` is `digitalRead`; `Sensor::serviceBus` latches on first poll | `tHostUs`/`tMeasUs` are late by up to one loop period; docs claim assertion time |
| D2 | No fault-triggered recovery; `Failed` is terminal | `Sensor::update` `:141–144`; `recover()` only called by the example's `r` key | Any transient fault is a permanent outage until the application intervenes |
| D3 | `recover()` does not reset the attempt budget | `Sensor::recover` `:369–394` | Five manual retries exhaust the budget permanently |
| D4 | Output loss is silent | `BinaryEncoder::emit` `:53–61`; callers ignore the result | Consumer sees a sequence gap with no error frame or surfaced counter |
| D5 | `dropped_` is not a loss counter | `BinaryEncoder.cpp:27–30`; null sink when binary output is off | Misleading if exposed |
| D6 | Product/diagnostic fields are dead | `Diagnostics` fields never written; example prints `product.partNumber` | Diagnostics display is misleading |
| D7 | `MANAGER_DIAGNOSTICS` is never emitted | `writeManagerDiagnostics` has no caller | PC never receives manager diagnostics |
| D8 | `SENSOR_STATUS` is periodic only and offset 8 differs from the API | `emitStatusIfDue`; `BinaryEncoder.cpp:123` | Status semantics diverge from `API.md` |
| D9 | No product-ID gate | `handleProdId` sets `valid` on the first `0xF8` (`Sh2Client.cpp:138–175`) | Configures before all entries arrive; `ProductIdMismatch` is never produced |
| D10 | `resetGroup` and `allowGrvFallback` are dead | Declared in `BNO085Types.h:267,271`; never read | Shared-reset and fallback behaviour described in the docs do not exist |
| D11 | Extra reports are accepted but ignored | `enable()` stores extras; `advanceInit` sends only the primary; `get()` returns false; `sensorReport` is null | Silent no-op for optional reports |
| D12 | Oversize during in-progress assembly is not cancelled | `ShtpTransport.cpp:146–205` returns without clearing `inRemaining_` | Possible stale assembly / misframed following fragments |
| D13 | `discard()` caps at 1024 bytes | `ShtpTransport.cpp:67` | A long malformed transfer can desynchronise the stream |
| D14 | Bus-lock timeouts are not surfaced | `SpiBusManager.cpp:102–104`; no `Diagnostics` write | `busBusy`/`BusTimeout` diagnostics are unavailable |
| D15 | `disable()` does not stop the device | `BNO085Sensor.cpp:413–419` | Device keeps streaming and asserting INT |
| D16 | Set-feature hub ack behaviour is unproven | No hardware | Source no longer depends on it, but the device's real behaviour is unknown |
| D17 | Normal fragment reassembly has no positive test | Registry has only rejection + oversize cases | A regression in `rxPayload_` assembly would not be caught |
| D18 | Example latency display can underflow | `01_SingleSensor.ino:164–170` | Cosmetic diagnostic error |

---

## 8. Documentation consistency audit

The documents are a mixture of approved Phase 0 design, Phase 1 delivery notes and
historical evidence. The table lists every material contradiction with the current code. It
does **not** count Phase 2+ material that is explicitly labelled as future work, except where
the document states or implies that Phase 1 already implements it.

### 8.1 `README.md`

| Line | Statement | Current code | Verdict |
| --- | --- | --- | --- |
| 8 | "compiles for the ESP32-S3 and passes its host tests" | Host tests pass (39/185). Compile was not re-executed here. | Host-tested part verified; compile claim is historical |
| 21 | "324 174 B flash (24 %), 23 880 B static RAM (7 %)" | Not re-compiled in this audit | Historical, unverified here |
| 22 | "39 cases / 185 checks, 0 failures" | Executed: exactly 39/185, 0 failures | **Accurate** |
| 24 | Physical stages 1–2 and drain evidence | No hardware in this audit | Historical; not new evidence |
| 121–127 | `tMeas = tEdge + (referenceDelta + delayTicks) × 100`, delta starts at 0, `0xFB` replaces with `−timebase`, `0xFA` accumulates | Matches `Sh2Client.cpp:249–298, 377–385` | **Accurate** |
| 137 | "Fragmented payloads are drained and counted, not reassembled" | Normal fragments **are** reassembled (`ShtpTransport.cpp:221–267`); only oversized transfers are drained | **Wrong** |
| 24 | "552 B channel-0 advertisement is drained in 4 bus transactions" | Historical hardware result; code path matches | Historical |
| 83 | `test/hardware/02_TransportDrain` exists | File exists | Accurate |

### 8.2 `docs/PHASE1-NOTES.md`

| Line | Statement | Current code | Verdict |
| --- | --- | --- | --- |
| 27 | Deviation 4: "`tMeas = tEdge - 12000 us + delayTicks * 100` plus a rebase offset" | Current code starts at 0, applies `0xFB`/`0xFA` per payload, no `−12000`, no persistent rebase offset | **Outdated** (the superseded note at 33–37 mentions it, but the row itself is wrong) |
| 28 | Deviation 5: "Oversize transfers are drained … the payload is not assembled" | Normal fragments are assembled; only oversized transfers are drained | **Partly wrong**; the superseded note does not correct this row |
| 55–58 | "168 checks across 30 cases" and "(Current count: 39 cases / 185 checks)" | Executed 39/185 | Historical + current count is correct |
| 85 | Procedure: "30 cases / 168 checks" | Historical delivery figure | Historical, not current |
| 121 | "324 262 B flash (24 %), 23 880 B static RAM (7 %)" | README gives 324 174 B; not re-compiled here | **Stale/inconsistent with README** |
| 137–139 | Open item: "confirm the 12 000 µs edge-to-reception offset … then decide whether the 0xFA rebase is needed" | Offset removed; `0xFA` rebase implemented per reference | **Outdated open item** |
| 48/52 | Boot 93 665 µs; drain 4 transactions (peek + 256 + 256 + 40) | Historical hardware evidence | Consistent with README |

### 8.3 `docs/ARCHITECTURE.md`

| Line | Statement | Current code | Verdict |
| --- | --- | --- | --- |
| 169, 306–312, 436; 6.4 | ISR on `FALLING`, atomic pending mask, `edgeUs[i] = micros()` | Polls `digitalRead`; no `attachInterrupt`; latches first observation | **Contradiction** (D1) |
| 154 | "IdentifyChk: gate on known-good part number" | `product_.valid` set on first `0xF8`; no gate; `ProductIdMismatch` never produced | **Contradiction** (D9) |
| 381–384 | State machine consumes advertisement + reset-complete + channel-2 init message before configuring | Boot advances on any complete packet; advertisement is oversize and does not count; reset-complete early-returns in Boot/Identify; no init-message handling | **Contradiction** |
| 277 | CS deasserted via RAII guard | No RAII guard; no early return between CS low/high, so the invariant holds in practice | **Partly wrong** (implementation detail) |
| 297–298 | Lock timeout → sensor skipped, `diag.busBusy++` | `lockTimeouts_` counted in `SpiBusManager`; not mirrored to `Diagnostics` | **Contradiction** (D14) |
| 320–346 | Cross-sensor round-robin, cursor, fairness, stuck-INT counter | No `Manager`; per-sensor loop only; no stuck-INT counter | **Phase 2 not implemented** |
| 355 | SH-2 timeout → `Degraded` | `enterFailed` sets `Failed` | **Contradiction** |
| 365–372, 416–423 | Reset groups / shared NRST behaviour | `resetGroup` never read | **Not implemented** (D10) |
| 388–398 | Failure detection: 3 consecutive invalid headers, malformed counter → `Degraded`, product-ID gate | None of these exist; faults go to `Failed` or only increment counters | **Contradiction** |
| 400–414 | Automatic recovery FSM, backoff with `(index × 37 ms)` phase offset, `recover()` resets attempts, status frame per attempt | Manual recovery only; no phase offset; attempts not reset; no status frame per attempt | **Contradiction** (D2/D3) |
| 451 | One request in flight, matched by report ID + command sequence, 100 ms deadline | Per-operation pending flags; command sequence is only logged; set-feature has no request in flight; product ID is matched by nothing | **Contradiction** |
| 453 | Single `new Sensor[n]` allocation | No `Manager` or allocation in Phase 1 | **Phase 2 not implemented** |
| 450 | Stateless CEVA decoder vendored as optional path | `src/sh2/third_party/ceva/` is absent | **Not present** |
| 467 | `begin(configs, n)` for 1–20 sensors | `Sensor::begin` is per sensor; no manager | **Not implemented** |
| 183 | `tMeas = T_int − referenceDelta + delay·100` | Code uses `tEdge + (referenceDelta + delay) × 100`, where `referenceDelta = −timebase` on `0xFB` | **Sign convention is ambiguous/contradictory** with README §121–127 |

### 8.4 `docs/API.md`

| Line | Statement | Current code | Verdict |
| --- | --- | --- | --- |
| 138–163 | `ManagerConfig` has `SPIClass* spi`, `Stream* output`, `OutputMode outputMode` | Actual struct has `IOutputSink* output`, `ILogSink* log`, `bool emitBinaryFrames`; no `spi` or `outputMode` | **Contradiction** |
| 212–228 | `Diagnostics` fields (`productId`, `fwMajor/minor/patch`, `rxPackets`, `busBusy`, `shortPackets`, `recoveryAttempts`, …) | Different field names/types (`ProductInfo product`, `shtpPackets`, `shortFragments`, no `busBusy`, no `recoveryAttempts`) | **Contradiction** |
| 121–130 | `begin()` enforces sensor count, distinct CS, `csPin != intPin/resetPin` | `Sensor::begin` enforces only per-sensor `csPin >= 0`, report, interval, `reportsValid()`, `spiHz` | **Contradiction** |
| 260–279 | `Manager` class and `next(Record&)` | No `Manager`, no aggregation | **Phase 2 not implemented** |
| 179, 378 | `tHostUs` = `micros()` at INT assertion | Poll-observation time | **Contradiction** (D1) |
| 286–288 | `update()` takes at most `serviceBudgetUs` | Budget checked *after* each transfer; a 256-byte read at 1 MHz can already be ~2 ms | **Can overshoot** |
| 356 | `SENSOR_STATUS` emitted periodically **and on state change** | Emitted only by `emitStatusIfDue` | **Contradiction** (D8) |
| 357 | `MANAGER_DIAGNOSTICS` emitted periodically | `writeManagerDiagnostics` never called | **Contradiction** (D7) |
| 355 | `IMU_SAMPLE` emitted for enabled raw/calibrated reports | No `writeImuSample`; extras are ignored; `get()` returns false | **Not implemented** (D11) |
| 367 | QUATERNION flags bit1 = first-after-recovery, bit2 = GRV fallback | Encoder sets only bit0 (never, since source is always RV) and bit3; bit1/bit2 never set | **Partly unimplemented** |
| 415 | `SENSOR_STATUS` offset 8 = `rxPackets` | Encoder writes `reportsDecoded` | **Contradiction** (D8) |
| 495–506 | JSON debug mode | No `JsonDebugEncoder` | **Not implemented** |
| 42–48 | `Report` enum values | Values match the current enum | Accurate |

### 8.5 `docs/PLAN.md`

| Line | Statement | Current code | Verdict |
| --- | --- | --- | --- |
| 157 | "30 host cases / 168 checks, 0 failures" | Executed 39/185 | **Stale** |
| 159–162 | Hardware validated stages 1–2 + drain; stages 3+ open | No hardware in this audit | Historical; still the accurate boundary |
| 203–216 | Deliverable checklist marks HAL, SPI bus manager, SHTP, SH-2, single-sensor example and unit tests as "Planned" | All are implemented | **Outdated status column** |
| 155 | Compiled example and diagnostic sketch | Not re-run here | Historical |
| 137–143 | Test strategy says host tests use `g++` | Only MSVC executed; GCC toolchain absent | Historical/toolchain mismatch |

### 8.6 `test/native/README.md`

| Line | Statement | Current registry | Verdict |
| --- | --- | --- | --- |
| 36 | SH-2 control covers "product id (4/5 entry forms), … hub rejection, command timeout" | Only the 4-entry product form; no 5-entry test; no hub-rejection test; no command-timeout test (no command is ever pending) | **Overstated** |
| 34 | SHTP reject includes "unexpected continuation" | Test exists (`shtp_unexpected_continuation`) | Accurate |
| 34–35 | SHTP continuation/fragmentation coverage | No positive normal-fragment reassembly test; only rejection and oversize | **Incomplete** |
| 37 | `0xFB`/`0xFA` per-payload delta cases | All four named cases exist | Accurate |
| 38 | Sensor FSM cases | All named cases exist | Accurate |
| 61–64 | "every payload the device can produce fits in one transfer, but the channel-0 advertisement … does not" | Self-contradictory: the advertisement is a payload the device produces; the intended statement is "every sensor report payload" | **Wording contradiction** |
| 58–59 | `build_gcc.sh` not executed (no g++) | No `g++` on `PATH` in this environment | Consistent |

### 8.7 Earlier `output.md` (superseded by this file)

The previous header and metadata were already wrong before this rewrite:

| Old statement | Current fact |
| --- | --- |
| "Analysed commit `d040c8e`" | HEAD is `e9a41ec` |
| "Pass-4 state: working tree on `91e537d`, **not committed**" | Pass 4 is committed as `e9a41ec` |
| "35 cases / 194 checks" (pass 3) | Current suite is 39/185 |
| "39 cases / 185 checks (pass 4)" | Correct, but now verified again in this audit |
| Pass 4 compile/analyze figures | Not re-executed in this audit |

---

## 9. Test registry inventory (actual, at `e9a41ec`)

`test/native/test_main.cpp:828–868` registers exactly **39** cases. The executed suite printed
`BNO085Multi host tests (39 cases)` and `185 checks, 0 failures`.

| Group | Cases |
| --- | --- |
| CRC / frame | `crc16_known_vector`, `encoder_quaternion_frame` |
| SHTP accept/reject | `shtp_valid_rotation_vector`, `shtp_no_data`, `shtp_zero_length`, `shtp_short_fragment`, `shtp_reserved_length`, `shtp_bad_channel`, `shtp_oversize_transfer`, `shtp_payload_too_large`, `shtp_unexpected_continuation`, `shtp_bus_error`, `shtp_sequence_gap` |
| SHTP oversize drain | `shtp_oversize_drain_complete`, `shtp_oversize_continuation` |
| SH-2 control | `sh2_product_id`, `sh2_set_feature_bytes`, `sh2_set_feature_no_ack_wait`, `sh2_late_ack_ignored`, `sh2_reset_complete` |
| SH-2 reports / timestamps | `sh2_rotation_vector_decode`, `sh2_delay_field`, `sh2_base_timestamp_ref`, `sh2_timestamp_rebase`, `sh2_timestamp_rebase_signed`, `sh2_timestamp_per_payload`, `sh2_timestamp_control_ignored`, `sh2_timestamp_known_example`, `sh2_timestamp_no_reference`, `sh2_timestamp_host_wrap`, `sh2_timestamp_before_zero`, `sh2_duplicate_suppression`, `sh2_report_gap`, `sh2_unknown_report` |
| Sensor FSM | `sensor_init_sequence`, `sensor_boot_timeout`, `sensor_rejects_grv`, `sensor_survives_malformed`, `sensor_recover_backoff` |

Coverage gaps are listed in §5.2 and §8.6. In particular:

- no positive normal-fragment reassembly test;
- no 5-entry product-ID test;
- no `0xF1` hub-rejection test and no command-timeout test (the command slot is no longer
  armed);
- no test that an oversize transfer arriving during assembly resets the assembly;
- no output short-write / `dropped_` test;
- no test for `Diagnostics` population.

---

## 10. What cannot be claimed from this audit

- **No physical validation.** The board is disconnected and COM11 is unavailable. Product-ID
  retrieval, set/get feature, Rotation Vector configuration at 100 Hz, report decode on real
  data, timestamp reconstruction in a real stream, INT latency, the 10-minute soak, the
  disconnected-INT timeout, 30 cold boots and the Adafruit comparison remain unmeasured.
- **No ESP32-S3 compile in this pass.** `arduino-cli` is absent from `PATH`. The historical
  324,174 B / 23,880 B figure is not evidence about the tree today.
- **No static analysis or sanitizer run in this pass.** MSVC `/analyze` and ASan were not
  executed; the previous ASan attempt could not link the runtime.
- **No GCC build.** `g++`, `sh`, `clang++`, `clang-tidy` and `cppcheck` are not on `PATH`.
- **No hardware proof of the SHTP peek/re-present assumption.** The host tests encode it in
  the mock; the only hardware evidence is the historical advertisement drain.

---

## 11. Recommended next work (ordered)

1. **Correct the documentation rows that are now factually wrong** (no code change):
   - `README.md:137` (fragments are reassembled);
   - `docs/PHASE1-NOTES.md:27–28` (timestamp formula and fragment assembly) and
     `:121,137–139` (stale compile figure and 12 ms open item);
   - `docs/PLAN.md:157` and the deliverable statuses at `:203–216`;
   - `test/native/README.md:36,61–64`.
2. **Add the missing positive host tests** before touching SHTP again: a two-fragment normal
   reassembly case, an oversize-during-assembly case, and a short-write/`dropped_` case.
3. **Decide D1 (INT):** implement the documented ISR + pending mask, or amend
   `README`/`API`/`ARCHITECTURE` to state "poll-observation time" consistently.
4. **Decide D2/D3 (recovery):** wire faults into `recover()` with the existing backoff, and
   reset the attempt counter in `recover()`, or document that Phase 1 recovery is
   application-driven.
5. **Decide D4/D5/D6/D7/D8 (output/diagnostics):** attach a null sink instead of `nullptr`,
   surface `framesDropped()`, emit one rate-limited `OutputOverflow`, populate
   `Diagnostics::product`, and align `SENSOR_STATUS` with `API.md` (or amend the API).
6. **Re-run evidence after any change:** host tests, ESP32-S3 compile, and (if available)
   static analysis. Do not mark stages 3+ physically tested until the board is reconnected
   and the procedure in the old §14.8 (now superseded) is actually performed.

---

## 12. Working-tree state after this audit

This audit changed **only** `output.md`. No source, test, build script, example or other
document was modified, staged, committed or pushed. `build/` output is git-ignored.

The final `git status` / `git diff --stat` are reproduced in the chat response that
accompanies this file; at the time of writing the only expected change is:

```
modified: output.md
```
