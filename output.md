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

> **Pass 5 update (2026-11-19, later).** HEAD is now `5b96597` ("Update output.md").
> Sections 1–12 below are the Pass 4 audit taken at `e9a41ec`; its source-behaviour
> findings still apply. Pass 5A changes are **uncommitted** in the working tree.
> Pass 5B–5D are investigations only, with no implementation. See §13 onward.
> Hardware is still disconnected; nothing in Pass 5 is physical validation.

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

---

## 13. Pass 5A — timestamp diagnostic (implemented, host-tested)

### 13.1 What the diagnostic computes

The example prints `dt=` for each Rotation Vector:

- `tHostUs` = host `micros()` captured when `Sensor::serviceBus` first observes INT
  asserted (`BNO085Sensor.cpp:165–174`), reused for every packet drained in that visit.
  It is **poll-observation time**, not the ISR edge time (see §16).
- `tMeasUs` = `tEdge + (referenceDeltaTicks + delayTicks) * 100`
  (`Sh2Client.cpp:377–385`) — the hub's estimated measurement instant.

`dt = tHostUs - tMeasUs` is the interval between the hub's estimate of when the sample was
taken and when the host noticed the interrupt. It is a transport/scheduling figure: it
includes the hub delay-field compensation, SPI transfer time, loop latency and any
printing/logging time. It is not sensor latency, and it is not a pure INT-to-read latency.

Before Pass 5A the subtraction was unsigned:

```cpp
static_cast<unsigned long>(q.tHostUs - q.tMeasUs)
```

If `tMeasUs > tHostUs` (possible whenever the reference delta is positive, or across a
counter wrap), this printed ~4.29e9 as a "latency". It did not affect `tMeasUs` storage or
the binary frame.

### 13.2 Fix

New header-only helper `src/util/TimeDelta.h`:

```cpp
inline int32_t timeDeltaUs(uint32_t a, uint32_t b);
```

- computes `a - b` with modular 32-bit semantics;
- returns a signed value; positive = `a` later, negative = `a` earlier;
- valid while the readings are < 2^31 µs apart (~35.8 min); exactly half-range is
  documented as ambiguous;
- uses an `int64_t` intermediate and an explicit fold, so there is no
  implementation-defined unsigned-to-signed conversion.

The example now does (`examples/01_SingleSensor/01_SingleSensor.ino:175–184`):

```cpp
const int32_t dtUs = bno085::timeDeltaUs(q.tHostUs, q.tMeasUs);
Serial.printf("... tMeas=%lu dt=%ldus%s\n", /* ... */,
              static_cast<unsigned long>(q.tMeasUs),
              static_cast<long>(dtUs),
              (dtUs < 0) ? " (tMeas ahead of tHost)" : "");
```

and documents the meaning, the polling limitation, negative values and the rollover limit
in the sketch header (`:18–29`). `BNO085Multi.h` includes the helper so sketches can call
`bno085::timeDeltaUs`.

Not changed: `Sh2Client::computeMeasTime`, `Quaternion::tMeasUs`, `tHostUs`, the binary
QUATERNION payload, or the `0xFB`/`0xFA` handling. The investigation found no defect in the
reconstruction itself; only the display arithmetic was wrong.

### 13.3 Tests and results

`test/native/test_main.cpp` adds `#include "util/TimeDelta.h"` and five cases:

| Case | Checks |
| --- | --- |
| `time_delta_positive` | 1 003 000 − 1 000 000 = 3000; 1 − 0 = 1 |
| `time_delta_zero` | 0 − 0; equal large values |
| `time_delta_negative` | 1000 − 1200 = −200; 0 − 1 = −1 |
| `time_delta_rollover` | 50 − 0xFFFFFFF0 = 66; reverse = −66; 0 − 0xFFFFFFFF = 1; reverse = −1 |
| `time_delta_boundary` | ±INT32_MAX; half-range → INT32_MIN; INT32_MIN+1; 0x7FFFFFFF−0xFFFFFFFF = INT32_MIN |

Commands executed in this pass:

```
cmd /c "test\native\build_msvc.bat"
→ BNO085Multi host tests (44 cases)
→ 201 checks, 0 failures
→ RESULT: PASS   (exit 0)
```

```
.\.tools\arduino-cli.exe compile --fqbn esp32:esp32:esp32s3 --library . ^
    --output-dir build\esp32s3_pass5 examples\01_SingleSensor
→ Sketch uses 324262 bytes (24%) of program storage space.
→ Global variables use 23880 bytes (7%) of dynamic memory.
→ exit 0
```

The first tooling attempt used `--libraries .`, which made the CLI treat `src/` as a
library named `src`; the link failed with undefined references. The correct flag for one
library root is `--library .`. The successful log is `build/esp32s3_pass5.log`; `build/` is
git-ignored. Compile size is 88 B above the README's historical 324 174 B; static RAM is
unchanged. This is a compile result, not a hardware run.

### 13.4 Remaining diagnostic limitations

- `tHostUs` is still poll-observation time, so `dt` includes loop latency (§16).
- `dt` mixes hub delay compensation with host latency. `Quaternion::delayTicks` is exposed
  if a host wants to separate the known part.
- The value is only meaningful if the hub's `0xFB` base is aligned with host time; a
  misaligned base appears as a constant or drifting offset.
- No hardware measurement of the real `dt` distribution exists.

---

## 14. Pass 5B — binary output reliability (investigation only, NOT implemented)

### 14.1 Actual path

```
Sensor::update()                                  BNO085Sensor.cpp:139
  serviceBus()                                    :153 / :165
    shtp_.service(intEdgeUs_)                     :189
      ShtpTransport assembles a payload           ShtpTransport.cpp:221–278
        handler_ → Sensor::onChannel              :274–277 / :421
          sh2_.handlePayload → quaternion handler
            Sensor::onQuaternion                  BNO085Sensor.cpp:426
              encoder_.writeQuaternion            :442
                BinaryEncoder::emit               BinaryEncoder.cpp:25
                  sink_->write(frame, total)      :53
                    StreamSink::write             ArduinoSink.h:18–20
                      HardwareSerial::write       core 3.3.12
```

The output write is synchronous and runs **inside the SHTP service callback**, before the
rest of the current INT visit has been drained. The service budget
(`ManagerConfig::serviceBudgetUs`, checked at `BNO085Sensor.cpp:216` only after each
transfer) cannot interrupt a blocked `write`.

HELLO, INIT_RESULT, ERROR and SENSOR_STATUS are emitted from
`begin`/`advanceInit`/`checkSilence`/`emitStatusIfDue`, i.e. outside `serviceBus` but still
inside the `update()` path.

### 14.2 What the sink actually guarantees

`StreamSink::write` returns `stream_.write(data, len)`. In the installed Arduino-ESP32 core
3.3.12:

```cpp
size_t HardwareSerial::write(const uint8_t *buffer, size_t size) {
  uartWriteBuf(_uart, buffer, size);
  return size;               // unconditional
}
```

`uartWriteBuf` calls `uart_write_bytes` and ignores its return value. On the `Serial` used
by this sketch (`ARDUINO_USB_CDC_ON_BOOT=0`, so hardware UART0):

- A short write is **never reported**; `write` returns `len` even when `_uart == NULL`.
- A zero-byte or failed transmission is **never reported**.
- `BinaryEncoder`'s `written != total` branch is effectively unreachable for this backend.
- `uart_write_bytes` blocks while the TX ring buffer is full. With no flow control the UART
  drains continuously, so the block is bounded by the frame size at the current baud; it is
  not bounded by a timeout.
- Switching to USB CDC (`ARDUINO_USB_CDC_ON_BOOT=1`) would change this: `USBCDC::write` has
  its own timeout/blocking behaviour and needs separate verification.

Frames are 34 bytes (QUATERNION: 8 header + 24 payload + 2 CRC). At 115200 8N1 that is
~3.0 ms if the ring is full; the default service budget is 2 ms. Text output is heavier: a
line of ~100 bytes is ~8.7 ms, so the text example can spend most of a 10 ms report
interval printing and delay INT observation.

### 14.3 Counter semantics today

`BinaryEncoder` has three counters (`BinaryEncoder.h:24–25, 43–44`):

| Counter | Incremented when | Problem |
| --- | --- | --- |
| `frames_` (`framesWritten`) | `sink_->write` returned `total` | Not exposed by `Sensor` |
| `dropped_` (`framesDropped`) | sink is `nullptr`; payload too large; short write | Conflates three different events; unused outside the encoder |
| `sequence_` | before the write attempt (`BinaryEncoder.cpp:43`) | Consumed even when the frame is dropped; the consumer sees a gap but cannot tell why |

When `emitBinaryFrames` is false, `begin()` attaches `nullptr` (`BNO085Sensor.cpp:56`), but
HELLO, INIT_RESULT, ERROR and STATUS are still emitted unconditionally. Those intentional
suppressions increment `dropped_`, so `dropped_` currently means "write not performed", not
"data lost". No `Diagnostics` field carries frame counts, no error frame is generated on
loss, and `framesDropped()` has no caller.

### 14.4 Answers to the specific questions

- **Partial writes:** possible at the `IOutputSink` contract level, but not observable
  through `HardwareSerial`; if a sink did write a prefix and return short, the encoder would
  count a drop after corrupting the byte stream.
- **Zero-byte writes:** not reported; `HardwareSerial` returns `size` regardless.
- **Failed writes:** not reported; same reason.
- **Buffer exhaustion:** manifests as blocking inside `uart_write_bytes`, not as a return
  value.
- **Frame loss:** silent; only a sequence gap in the consumer hints at it.
- **Can the application observe failures?** No. `Sensor` ignores every `write*` return and
  `Diagnostics` has no counter.
- **Can output block sensor servicing?** Yes. The write is inside `serviceBus`, and the
  2 ms budget is checked only after each transfer. A full UART ring adds the frame drain
  time; text logging (`StreamLogSink::println`) adds more. The block is bounded per frame,
  but it can exceed the budget and delay the next INT observation.
- **Intentional suppression vs drop vs failure:** currently all three collapse into
  `dropped_`.

### 14.5 Minimal proposed design (for review; not implemented)

Goal: every byte that is not delivered is classified and countable, with no queue and no
new blocking.

1. **Make the sink honest and atomic.** `StreamSink::write` checks
   `stream_.availableForWrite()` first; if the whole frame does not fit, return 0 without
   writing. This never blocks and never emits a partial frame. Keep
   `IOutputSink::write` semantics: "returns bytes accepted; a value other than `len` means
   the frame was not emitted". `HardwareSerial` implements `availableForWrite()` (TX ring
   free space); frames (<= 34 B) fit the default 256 B ring.
2. **Separate the three outcomes.** `suppressed_`: no sink or binary output intentionally
   off. `dropped_`: a real frame could not be written. `frames_`: delivered. Keep
   `sequence_` advancing for real attempted frames (including drops) so a gap still signals
   loss; do **not** advance it for suppressed frames.
3. **Stop counting suppression as loss.** Either guard every encoder call with
   `manager_.emitBinaryFrames` (HELLO/INIT/ERROR/STATUS as well as QUATERNION), or attach a
   `NullOutputSink` and have `emit` treat it as suppressed. The guard is simpler.
4. **Surface the loss.** Add `framesDropped` to `Diagnostics` (mirrored on emit) and to
   `SENSOR_STATUS` (use the reserved byte or extend the payload; version and document it).
   Emit one `ERROR(OutputOverflow)` on the first drop in a window, rate-limited (e.g. once
   per second), with `context` = drop count. Do not retry the failed frame. Optionally add a
   `HELLO` capability bit so a host knows the counter is valid.
5. **Bound the work.** One frame per call, no queue, drop-newest when the sink is full. A
   2-frame queue would only move the loss and add latency; not recommended.
6. **Document** that `dropped_` is a real-loss counter, frames are atomic, and the UART ring
   is the only buffer.

### 14.6 Proposed regression tests

Host-simulatable (new `MockSink` with settable capacity):

1. Capacity exactly `len` → delivered, `framesWritten == 1`.
2. Capacity `len - 1` → `framesDropped == 1`, `framesWritten == 0`, sink byte stream empty
   (no partial prefix).
3. `emitBinaryFrames == false` → suppressed increments, dropped stays 0.
4. A real drop → the next successful frame carries sequence +2 (consumer-visible gap).
5. Status frame reports the drop count.
6. Repeated drops → at most one ERROR frame per rate-limit window.
7. A drop does not stop the next report from decoding.
8. The sink is never asked for more than it can accept in one call.

Hardware-only: actual `availableForWrite()` ring behaviour with a stalled USB host; whether
UART ever blocks at 115200/100 Hz; USB CDC timeout/partial/unplug behaviour; real drop
counts with a deliberately slow consumer.

---

## 15. Pass 5C — automatic recovery (investigation only, NOT implemented)

### 15.1 Every transition into `Failed`

| # | Trigger | Code | Error code | Recoverable today? |
| --- | --- | --- | --- | --- |
| 1 | HAL begin fails | `BNO085Sensor.cpp:70–79` | `InvalidConfig` | No — `started_` stays false |
| 2 | Boot deadline (300 ms) | `:235–237` | `BootTimeout` | Manual only |
| 3 | Identify deadline (100 ms) | `:250–251` | `Sh2Timeout` | Manual only |
| 4 | Set-feature stage deadline | `:279–281` | `Sh2Timeout` | Manual only |
| 5 | Set-feature `HubError`/`Timeout` | `:271–274` | `Sh2BadResponse`/`Sh2Timeout` | Unreachable (send completes immediately) |
| 6 | Get-feature `HubError`/`Timeout` | `:292–294` | `Sh2BadResponse` | Manual only |
| 7 | Get-feature deadline | `:295–297` | `Sh2Timeout` | Manual only |
| 8 | `recover()` with budget exhausted | `:373–375` | `BootTimeout` (misleading) | Terminal |

Not transitions to `Failed` today:

- SPI transfer failure → `setError(SpiRead)` only (`:203–206`), state unchanged.
- Malformed/oversize SHTP → `setError(ShtpBadFrame)` only (`:207–213`).
- Silence → `Degraded` + `NoReports` (`:329–346`), self-clears on the next report
  (`:434–439`).

A permanently broken bus or a device that never reports again does not fail; it either
retries forever with only a counter or sits in `Degraded`.

### 15.2 Answers

- **Which errors trigger recovery?** None automatically. Only the example's `r` key calls
  `recover()`.
- **Which require manual intervention?** Every `Failed` state, plus the permanent-bus and
  permanent-silence cases.
- **Recovery while the bus is unavailable?** `recover()` does not touch SPI; it schedules
  `startBoot()`. `startBoot()` calls `pulseReset()` (blocking `delay(1)` plus
  `digitalWrite`) and, when `resetPin < 0` and `recovering_`, `sh2_.sendReset()` through the
  bus. A broken bus fails at the boot deadline; it is bounded.
- **Are reset and init deadlines bounded?** Yes: `bootTimeoutMs` 300 ms and
  `commandTimeoutMs` 100 ms per stage. Identify resends every 10 ms until its deadline;
  Get-feature sends once and waits for its deadline. No unbounded loop.
- **Backoff:** `backoffForAttempt(1..5)` = 250, 500, 1000, 2000, 4000 ms, clamped later.
  `recover()` increments first, so attempt 1 waits 250 ms; `update()` in `Recovering` calls
  `startBoot()` when `retryAtUs_` is reached.
- **Starvation:** with one sensor, `Recovering` costs one `update()` call and a 1 ms reset
  delay when the deadline fires. It does not starve the loop. In Phase 2, several sensors
  resetting together would add up; the manager will need staggered or non-blocking resets.
- **Diagnostics counts:** `resetPulses` counts reset pulses and `resets` counts
  reset-complete messages, but attempts, successes, terminal state and last-recovery reason
  are not stored. `recover()` logs the attempt number only. `diag_.lastError` can be
  overwritten by later transient errors.

### 15.3 Proposed fault-recovery FSM (for review; not implemented)

```
Streaming ──fault──> (classify)
   │                   ├─ fatal config ──────────────> Failed(terminal)
   │                   ├─ transient (< N in window) ─> stay, count
   │                   └─ recoverable ───────────────> Recovering(backoff)
   │                                                        │
   │                                                        v
   │                                                     Booting
   │                                                        │
   │                                                        v
   │                                     Identifying -> Configuring
   │                                                        │
   └──────────────── Streaming (success: attempts = 0) <────┘
                                                             │
                              attempts >= max ───────────────┴──> Failed(terminal)
```

Rules:

1. **Classification.** `InvalidConfig`/`UnsupportedReport` are fatal. `BootTimeout`,
   `Sh2Timeout`, `Sh2BadResponse` are recoverable. `SpiRead`/`ShtpBadFrame`/`NoReports` are
   transient until N occur in a window, then recoverable.
2. **Bounded attempts.** `RecoveryPolicy::maxAttempts` (default 5) with the existing backoff
   table. Increment only when a recovery is actually scheduled.
3. **Success resets.** `enterStreaming()` sets `attempts = 0`, `successes++`,
   `terminal = false`. `enterStreaming()` already resets attempts; keep that.
4. **Terminal failure.** After `maxAttempts` consecutive recoveries without reaching
   `Streaming`, enter `Failed` with `terminal = true` and the actual reason. `update()`
   stays a no-op. Do not rewrite the cause as `BootTimeout`.
5. **Manual override.** `recover()` from terminal resets the budget and schedules an
   immediate attempt, but rate-limit manual calls (e.g. one per second) so an application
   loop cannot thrash the device. Document it as an operator action.
6. **No recovery in `begin()` failure.** A HAL/configuration failure is fatal; return it
   and stay `Unconfigured`.
7. **Bus-unavailable handling.** Do not pulse reset while the last K bus transfers failed.
   Either probe a cheap transfer first or count the attempt and let the boot deadline
   govern. Keep the reset pulse non-blocking or accept the bounded 1 ms delay explicitly.
8. **Diagnostics.** Add `recoveryAttempts`, `recoverySuccesses`, `recoveryFatal`,
   `lastRecoveryReason`; emit an `INIT_RESULT`/status frame on each attempt and on the
   terminal transition (ARCHITECTURE already promises this).

### 15.4 Proposed tests

Host-simulatable (MockHal clock + `failTransfers`):

1. Bus error during streaming triggers recovery after the configured threshold.
2. Deadline failure in each init stage schedules recovery, not immediate `Failed`.
3. Backoff schedule is exactly 250/500/1000/2000/4000 ms.
4. Exhaustion enters terminal `Failed` with the real error code and stops servicing.
5. A successful recovery resets the attempt counter.
6. Manual `recover()` from terminal resets the budget; a second call inside the rate-limit
   window is ignored.
7. Config/HAL begin failure stays terminal and never schedules recovery.
8. Silence beyond the window triggers recovery (or explicitly does not, if the policy is
   "degrade only").
9. `update()` during `Recovering` stays under the documented budget (mock clock measures the
   reset delay).
10. `resetPulses`/`resets`/attempt counters match the number of transitions.

Hardware-only: real NRST timing and device boot time; software reset over SHTP with no
reset pin; recovery after physically disconnecting MISO/INT; repeated cold boots and soak.

---

## 16. Pass 5D — INT handling and timestamp semantics (investigation only)

### 16.1 Current behaviour

`Esp32SpiHal::intAsserted()` is `digitalRead(intPin) == LOW` (`Esp32SpiHal.cpp:91–95`);
there is no `attachInterrupt`. `Sensor::serviceBus` samples it once per `update()` and
latches `intEdgeUs_ = nowUs` on the first asserted observation (`BNO085Sensor.cpp:165–180`).
All packets drained in that visit share that timestamp (`:189`). `Quaternion::tHostUs`
therefore means "host time when the loop noticed INT", not "hardware INT assertion time".
`BNO085Types.h:141` still says "host time at INT assertion"; that comment is inaccurate.

### 16.2 Option comparison

| Criterion | A: keep polling, define observation time | B: ISR captures edge time + pending flag |
| --- | --- | --- |
| Timestamp accuracy | Error = physical edge to next `update()`; up to one loop period. At 100 Hz with binary output a few ms; with per-report text printing it can approach or exceed 10 ms. | Error = ISR entry latency (µs on ESP32-S3), independent of loop load. |
| Interrupt behaviour | No ISR. INT is level; the loop drains up to `maxTransfersPerVisit` per visit. | `FALLING` edge only. The device holds INT low until serviced, so one edge covers all pending transfers. A partial drain produces no new edge. |
| Concurrency / atomicity | None; single-threaded. | ISR writes a 32-bit stamp and a pending flag while the loop reads/clears them. Needs a critical section or sequence counter; 32-bit aligned writes are atomic on Xtensa, but the pair still needs ordering. |
| Missed / coalesced events | A slow loop coalesces reports in the INT window into one observation; multiple pulses between updates collapse. | Same protocol-level coalescing; the ISR at least records the first edge. A second edge while pending is lost unless counted. |
| ESP32-S3 complexity | Low; already portable and mockable. | Medium: `attachInterrupt`, IRAM-safe handler, detach in `end()`, `volatile`/atomic state, portable `IHal` hook, mock support. |
| Testability | Deterministic in `MockHal` (set `intState`, advance clock). | Needs mock ISR injection; races are harder to cover. |
| Future multi-sensor | Poll latency grows with sensor count and bus time; per-sensor stamps become less comparable. | Per-pin stamps stay comparable; the manager still schedules service, so stamp accuracy is preserved. |

### 16.3 Recommendation

**Option A for Phase 1.** The requirement is a single-sensor baseline; `tHostUs` currently
feeds (a) the `dt` diagnostic and (b) silence detection (`lastReportUs_`). Neither needs
sub-millisecond accuracy. `tMeasUs` reconstruction uses `tHostUs` only as the edge anchor,
and the 100 µs tick already dominates. Polling is deterministic, portable and covered by the
mock.

Adopt Option B when at least one of these becomes true:

- timestamps feed fusion/filtering with a sub-millisecond error budget;
- multi-sensor scheduling makes poll latency comparable to the report interval;
- the text example remains the primary output and its print time cannot be bounded.

Cheap interim measures under A: keep per-report printing out of the hot path when timestamps
matter, document `tHostUs` as observation time everywhere, and use the exposed
`Quaternion::delayTicks` to separate hub compensation from host latency.

### 16.4 Proposed tests

Host-simulatable under A:

1. `tHostUs` equals the mock clock at the first asserted observation.
2. All packets drained in one visit share that stamp.
3. A visit with INT low does not change the stamp.
4. Silence detection uses the observation stamp (existing behaviour).

Under B (future): ISR stamp earlier than the loop observation by a controlled mock delay;
pending flag not lost when the edge arrives between read and clear; `end()` detaches the
interrupt (no callback after teardown); coalesced edges increment a missed-edge counter.

Hardware-only in both cases: actual INT-to-first-SPI time measured with a scope/logic
analyser; whether the BNO085 deasserts INT between reports at 100 Hz; behaviour with a
shared bus and multiple sensors.

---

## 17. Confirmed defects versus design recommendations

Confirmed defects in the current tree (source-verified, no hardware required):

| # | Defect | Evidence |
| --- | --- | --- |
| C1 | The example printed an unsigned timestamp difference, so a negative `dt` became ~4.29e9 | Fixed in Pass 5A |
| C2 | `HardwareSerial::write` returns `size` unconditionally, so the encoder's short-write detection cannot fire on the Arduino backend | core 3.3.12 `HardwareSerial.cpp`; `ArduinoSink.h:18–19` |
| C3 | `dropped_` conflates intentional suppression, oversize payload and failed write | `BinaryEncoder.cpp:27–36, 54–61`; null sink at `BNO085Sensor.cpp:56` |
| C4 | Output loss is not exposed; `framesDropped()` has no caller and no `Diagnostics` field | grep of `src/`, `examples/` |
| C5 | Every `Failed` state is terminal; `update()` returns immediately and nothing calls `recover()` automatically | `BNO085Sensor.cpp:141–144, 369–394` |
| C6 | `recover()` does not reset the attempt budget; only `begin()` and `enterStreaming()` do | `:82, :314, :369–394` |
| C7 | Exhausted recovery reports the wrong cause (`BootTimeout`) | `:373–375` |
| C8 | Permanent bus failure never reaches `Failed`; it only increments error counters | `:203–213` |
| C9 | `tHostUs` is poll-observation time while the type comment and docs say INT assertion time | `Esp32SpiHal.cpp:91–95`; `BNO085Types.h:141` |
| C10 | `Diagnostics::product`, `csTransactions`, `spiTransfers`, `busTimeouts`, `transferBytes` are never written | prior audit, still true |
| C11 | Normal two-fragment reassembly has no positive host test; only rejection/oversize are covered | registry §9 |

Design recommendations (not defects, pending approval):

- Atomic, non-blocking `StreamSink` with an `availableForWrite()` pre-check (5B).
- Separate suppressed/dropped/written counters and a rate-limited overflow error (5B).
- Bounded automatic recovery FSM with a terminal state and attempt diagnostics (5C).
- Keep INT polling for Phase 1 and standardise the "observation time" wording (5D).

---

## 18. Remaining hardware validation

Nothing in Pass 5 is hardware evidence. Still unmeasured:

- Product ID and the 4/5-entry product-ID forms on a real device.
- Set-feature acceptance, including whether the hub sends `0xF1` for `0xFD`.
- Get-feature granted interval.
- Rotation Vector decode and timestamp reconstruction on a real stream; distribution of
  `dt`.
- INT-to-service latency with the actual loop and output mode.
- UART backpressure / frame loss at 115200, and the USB CDC variant.
- Recovery with a wired NRST, and software reset over SHTP with `resetPin = -1`.
- The 10-minute soak, disconnected-INT timeout, 30 cold boots, and the Adafruit comparison.
- Bus behaviour with the BNO085 disconnected (MISO floating) on a real board.

---

## 19. Recommended next development step

1. Review and commit Pass 5A (helper + example + tests) before touching the output path.
2. Implement the 5B minimal design next: honest atomic sink, suppressed/dropped separation,
   `framesDropped` in diagnostics/status, one rate-limited overflow error. It is the
   smallest change that removes silent loss and is fully host-testable.
3. Then implement the 5C FSM behind a configuration switch (automatic recovery off by
   default until hardware is available), with the host tests in §15.4.
4. Treat 5D as a documentation decision first: standardise on "observation time". Add the
   ISR only if the timestamp error budget or Phase 2 scheduling demands it.
5. After any change, re-run the host suite and the ESP32-S3 compile; do not mark any stage
   physically validated until the board is reconnected.

---

## 20. Pass 5 working-tree checkpoint

At the end of this pass (uncommitted, not staged):

| File | Status | Change |
| --- | --- | --- |
| `src/util/TimeDelta.h` | new | rollover-safe signed time-delta helper |
| `src/BNO085Multi.h` | modified | include the helper |
| `examples/01_SingleSensor/01_SingleSensor.ino` | modified | signed `dt`, negative marker, documented semantics/limits |
| `test/native/test_main.cpp` | modified | 5 new cases, 16 new checks |
| `output.md` | modified | this Pass 5 record |

Executed evidence: host suite **44 cases / 201 checks / 0 failures / PASS (exit 0)**;
ESP32-S3 compile **exit 0**, **324 262 B flash (24%)**, **23 880 B static RAM (7%)**.
Pass 5B/5C/5D are investigations only; no implementation was added for them. No commit,
stage, push, amend or history change was performed.
