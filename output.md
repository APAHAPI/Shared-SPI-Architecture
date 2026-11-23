# BNO085Multi — Phase 1 reliability audit (Pass 6)

Audit date: 2026-11-19
Branch: `main`
Base commit: `d9fa757374c9f0e640a6559c725026a61f680d79` ("Print signed rollover-safe
timestamp deltas")
Working tree: **modified, uncommitted Pass 6 changes** (19 tracked files at the time
of writing; `output.md` is rewritten by this pass)

This document is a fresh audit of the current tree. Earlier statements about
"Pass 4 uncommitted" are obsolete: Pass 5A was committed at `d9fa757` and the tree was
clean before Pass 6. Nothing here is staged, committed, amended or pushed.

> **Update (2026-11-24).** The audit body below was written at `d9fa757`. The evidence
> and verification sections have been refreshed for the current tree: HEAD `f0f228a`
> ("Fix continuous-CS BNO085 reads"), with only `output.md` left modified. The Pass 6
> findings themselves are unchanged. Current physical evidence and the Stage 1 blocker
> are in [hardware_test_report.md](hardware_test_report.md).

Status tags used below:

- **source-inspected** — read in the current tree, reasoning only.
- **host-tested** — executed by `test/native` on the host PC with mock hardware.
- **compiled** — built for `esp32:esp32:esp32s3`; not executed.
- **hardware-verified** — measured on the physical ESP32-S3 + BNO085. Pass 6 itself
  added no hardware evidence; the 2026-11-24 update adds the read-framing result.

---

## 1. Verdict

Pass 6 closes the three Phase 1 reliability gaps that could hide failures:

1. **Output loss is no longer silent.** Every encoder outcome is classified and
   counted. A full UART/USB sink drops the frame atomically before any byte is written,
   and the application can distinguish "no sink", "too large", "no capacity", "partial"
   and "failed" writes. Frame sequence numbers are consumed by real attempts, so a PC
   parser sees a gap instead of silent continuity.
2. **A failed sensor no longer latches permanently.** Recoverable faults — boot/command
   timeouts, repeated SPI/protocol faults and report silence — enter a bounded
   `Recovering` state with backoff, attempt accounting and a terminal `Failed` state.
   `recover()` is coherent, rate-limited and can restart a terminal sensor.
3. **Timestamp semantics are explicit.** Phase 1 uses **Option A: polling**. `tHostUs`
   is the host time at first observation of INT, not the physical edge; `tMeasUs` remains
   the hub measurement time reconstructed from the report delay and `0xFB`/`0xFA`
   reference. No binary-protocol change was made for timing.

The cross-system review (6D) found and fixed one real ordering defect introduced by 6B
(a transport fault could rewind the stage and then let `advanceInit()` overwrite
`Recovering`), one stale-protocol-state defect after a device reset, and several
smaller coherence/observability defects. The remaining unfixed observations are listed
in §8 and are outside the 6A–6C scope.

**Hardware status in Pass 6: no physical validation was performed.** The 2026-11-24
update adds the read-framing result; Stage 1 is still blocked at Product ID (see §2 and
`hardware_test_report.md`).

---

## 2. Evidence collected (current tree)

| Evidence | Result |
| --- | --- |
| `git rev-parse HEAD` | `f0f228a` ("Fix continuous-CS BNO085 reads"); only `output.md` is modified |
| `git branch --show-current` | `main` |
| `git status --short` | 1 modified tracked file (`output.md`); the read-framing fix is committed as `f0f228a` |
| Host suite | **61 cases / 438 checks, 0 failures**; `cl /analyze` 0 warnings |
| ESP32-S3 example build | **326 806 B flash (24 %), 24 640 B RAM (7 %)**, `--warnings all`, no warnings |
| Hardware | Continuous-CS read fix verified on COM11: 276 B advertisement in one CS assertion; Stage 1 blocked at Product ID (PS0/WAKE not wired to a GPIO) |

Source-inspected but not hardware-tested: real UART/USB backpressure, actual INT
observation lag, real boot/backoff durations, and a PC parser consuming the sequence
gaps. NRST reset was exercised on the bench; SHTP software reset acceptance is
unverified because host writes are blocked by the PS0/WAKE wiring.

Pass 6 ran 54 cases / 348 checks and built 326 142 B flash / 23 968 B RAM. Those numbers
are historical; the table above is the current tree.

---

## 3. 6A — Binary output reliability

### 3.1 Contract change (sink boundary)

`IOutputSink` now exposes `availableForWrite()` before `write()`:

- `npos` means "capacity unknown; attempt the write".
- `write()` returns the number of bytes accepted and must not block indefinitely.
- `NullOutputSink` keeps the old behaviour but is no longer used to represent "output
  disabled"; the encoder is simply not attached.

`StreamSink` maps `Stream::availableForWrite()` to that contract. On the ESP32 core
3.3.12, `HardwareSerial` reports free TX-ring space and `write()` copies into the ring;
therefore a capacity pre-check prevents the blocking `uartWriteBuf()` path. A `Stream`
that returns 0 (an unconnected USB CDC, or a Stream with no capacity implementation)
causes frames to be **dropped and counted**, never queued or blocked.

### 3.2 Outcome classification (`BinaryEncoder::emit`)

| Outcome | Trigger | Counter | Sequence consumed |
| --- | --- | --- | --- |
| `Ok` | whole frame accepted | `framesWritten` | yes |
| `Suppressed` | no sink attached | `framesSuppressed` | **no** |
| `TooLarge` | payload > 64 B (internal invariant) | `framesTooLarge` | no |
| `NoCapacity` | capacity < frame size; **nothing written** | `framesNoCapacity` | yes |
| `PartialWrite` | sink accepted `0 < n < len` | `framesPartial` | yes |
| `FailedWrite` | sink accepted 0 bytes | `framesFailed` | yes |

`framesDropped = tooLarge + noCapacity + partial + failed`. `Sensor` mirrors these into
`Diagnostics`, together with `outputEnabled`, `lastOutputError` and `lastOutputErrorUs`.

A sensor now:

- attaches the encoder only when `emitBinaryFrames` **and** a sink exist;
- reports `ErrorCode::OutputUnavailable` when frames were requested without a sink;
- observes the return value of every `HELLO`, `INIT_RESULT`, `QUATERNION`, `STATUS` and
  `ERROR` write instead of ignoring it.

### 3.3 Partial-frame recoverability

`NoCapacity` is atomic: the capacity check happens before the first byte, so no partial
frame is produced. `PartialWrite` can only happen if a sink over-reports capacity or
returns a short count; the prefix is already on the wire. The protocol already supports
recovery: the receiver scans for `A5 5A`, validates version/type/length/CRC, and rejects
the truncated frame. The encoder never retries the remainder, so the stream
resynchronises at the next good frame. This is documented in API §4.11.

### 3.4 Service impact and logging

- Output is written from the SHTP handler **after** the SPI body transfer and CS release,
  so the bus and CS are never held during a serial write.
- There is no output queue and no unbounded buffering. When the sink is full the frame is
  dropped and counted.
- Encoder logging is limited to the first drop of an episode (reset on the next success),
  and `Sensor::observeOutput()` logs totals at most once per second. This prevents a
  broken sink from flooding the log inside the sensor path.

### 3.5 Host tests (6A)

- `encoder_output_outcomes` — success, full sink (no `write()` call, no bytes, sequence
  consumed), zero-byte write, partial write, per-cause counters and totals.
- `encoder_drop_then_recovery` — three drops, then a good frame whose on-wire sequence
  shows the gap; suppressed frames do not consume a sequence.
- `sensor_output_drops` — a permanently full sink from `begin()`; counters increase while
  the sensor still boots and streams, then a recovered sink emits with the expected
  sequence gap.
- `sensor_output_unavailable` — output requested without a sink reports
  `OutputUnavailable` and counts no loss.

Not host-testable: the actual `HardwareSerial` return value and USB CDC backpressure.
The mock sink tests the encoder contract at the abstraction boundary.

---

## 4. 6B — Bounded automatic recovery

### 4.1 States and classification

```
Unconfigured → Booting → Identifying → Configuring → Streaming ⇄ Degraded
                                  │                        │
                                  └── recoverable fault ───┴──► Recovering
                                                                 │ backoff
                                                                 ▼
                                                              Booting …
                                                                 │ budget exhausted
                                                                 ▼
                                                          Failed (terminal)
```

Recoverable while the sensor is started:

- boot window expiry (`BootTimeout`);
- product-ID / set-feature / get-feature deadline (`Sh2Timeout`);
- hub rejection (`Sh2BadResponse`);
- `transientErrorThreshold` consecutive SPI failures (`SpiRead`);
- the same threshold of consecutive malformed/oversize SHTP results (`ShtpBadFrame`);
- report silence: `2.5 × interval + 50 ms` → `Degraded`, then `silenceRecoveryMs`
  (default 2 s) → recovery (`NoReports`).

Terminal without retry: configuration rejected at `begin()` and HAL `begin()` failure.

### 4.2 Timing and attempts

- `Recovering` does **no SPI and no SH-2 work**. It waits for the backoff deadline; a
  failed device cannot monopolise the bus or hang `update()`.
- Backoff: 250, 500, 1000, 2000, 4000 ms (`RecoveryPolicy::backoffMs`), bounded by
  `maxAttempts` (default 5).
- A successful init clears the attempt streak, increments `recoverySuccesses`, and
  returns to `Streaming`.
- Each failed boot/config attempt increments `recoveryFailures`. Exhaustion enters
  terminal `Failed`, sets `recoveryTerminal`, and records the real cause in
  `lastRecoveryReason` (no more synthetic `BootTimeout`).
- `autoRecover = false` makes the first recoverable fault terminal, for applications
  that want manual control.

### 4.3 Manual recovery

`Status Sensor::recover()`:

- returns `InvalidConfig` if not started or disabled;
- is rate-limited by `manualRecoverIntervalMs` (default 1000 ms) and returns
  `Status::RecoveryRateLimited` when called too soon;
- resets the automatic attempt budget (an operator action gets a fresh bounded budget);
- works even when `autoRecover` is false and after terminal failure;
- schedules the same backoff/reset path, so manual and automatic recovery cannot diverge.

### 4.4 Host tests (6B)

- `sensor_boot_timeout` — timeout enters `Recovering`, not terminal.
- `sensor_recovery_succeeds` — exact backoff boundary, boot, streaming, success counter.
- `sensor_recovery_exhausts` — all five backoffs, terminal `Failed`, no bus traffic while
  terminal.
- `sensor_manual_recovery` — recovery after terminal failure, rate-limited second call,
  successful re-init.
- `sensor_transport_recovery` — three consecutive SPI failures trigger recovery, then a
  successful re-init.
- `sensor_silence_recovery` — `Degraded` first, recovery only after the silence dwell.

Hardware-only: actual boot/backoff durations after NRST or SHTP reset, and whether a
real device recovers on the first attempt.

---

## 5. 6C — INT and timestamp semantics

### 5.1 Chosen design: Option A (polling)

Phase 1 has **no interrupt handler**. `Sensor::serviceBus()` polls
`IHal::intAsserted()`; on first observation it latches `intEdgeUs_ = micros()`. All
packets drained in that visit (including all fragments of one payload) carry that same
observation time. Option A was chosen over a minimal ISR because:

- it removes ISR/shared-bus races in a single-writer design;
- a level-poll cannot miss H_INTN while the device holds it asserted until CS;
- it keeps the portable core free of platform interrupt APIs.

Option B (ISR edge capture) remains a Phase 2 candidate only if hardware measurements
show that observation lag matters.

### 5.2 The four times, explicitly

| Time | Meaning | Where |
| --- | --- | --- |
| Physical H_INTN edge | Device finished a report and asserted INT | Hardware; **not measured** |
| ESP32 observation (`intEdgeUs_`) | First `digitalRead` in `update()` that saw INT asserted | `BNO085Sensor.cpp` |
| Sensor measurement (`tMeasUs`) | Hub estimate: `tObs + (referenceDelta + delayTicks) × 100 µs` | `Sh2Client` |
| Host output timestamp (`tHostUs`) | Same observation time, exported in the record | `Quaternion` / `Sh2Client` |

Consequences, documented in `Quaternion::tHostUs`:

- `tHostUs` can lag the physical edge by up to one loop period;
- several reports drained together share one `tHostUs`;
- `tMeasUs` may be later than `tHostUs` (negative `dt`); that is reference skew, not a
  huge latency;
- neither timestamp changes the binary layout.

### 5.3 Host tests (6C)

- Existing timestamp tests cover the `0xFB`/`0xFA` arithmetic, per-payload reset,
  host-counter wrap and negative results.
- `sensor_observation_timestamp` drains two queued reports in one pass and verifies the
  shared observation time, distinct `tMeasUs`, and the latest sequence.

Hardware-only: real edge-to-observation lag and whether a ISR is warranted.

---

## 6. 6D — Cross-system review

Reviewed: sensor FSM, recovery scheduling, SHTP service/send, SH-2 command state,
bus-manager interaction, INT latching, output encoder, sink, example and diagnostics.

### 6.1 Defects fixed in this pass

1. **Recovery clobbered by the same update.** A transport fault inside `serviceBus()`
   rewound the stage and entered `Recovering`; `update()` then continued into
   `advanceInit()`, which could immediately move the sensor back to `Identifying`
   because the boot packet baseline was stale. Fixed by skipping all init stages when
   the drain leaves the sensor `Recovering` or `Failed`.
2. **Stale SHTP sequence state after a device reset.** `onResetComplete()` reconfigured
   SH-2 but left SHTP `nextInSeq_`/`nextOutSeq_` from before the reset, so every packet
   after a device restart counted a false gap. Fixed by resetting SHTP and clearing
   `intPending_`.
3. **`recover()` was incoherent when the budget was exhausted.** It entered `Failed`
   with `BootTimeout` regardless of the real cause. Fixed: manual recovery resets the
   budget and schedules normally; exhaustion is reached only through the automatic path
   and keeps the actual reason.
4. **Set-feature → get-feature transition could wait a full retry interval.**
   `nextAttemptUs_` was not reset and `getFeatureSent_` was not cleared on the stage
   transition. Fixed, and the get-feature request now has an explicit retry cadence and
   a recoverable deadline.
5. **Every output return value was ignored.** Fixed throughout the sensor; `HELLO`,
   `INIT_RESULT`, `QUATERNION`, `STATUS` and `ERROR` now update loss counters.
6. **No sink capacity check.** A full sink could block or partially write. Fixed with
   the atomic pre-check in §3.
7. **Binary output requested without a sink silently did nothing.** Now
   `OutputUnavailable`.
8. **Example binary mode mixed text logs with frames on the same Serial.** Fixed: logging
   is disabled when `BNO085_BINARY_OUTPUT` is 1 (binary diagnostics remain available
   through the protocol).
9. **Encoder could flood the log on every dropped frame.** Fixed with one log per loss
   episode plus the sensor's one-per-second summary.
10. **`recover()` on a disabled sensor returned `Ok` while doing nothing.** Now
    `InvalidConfig`.

### 6.2 Reviewed and left unchanged (out of scope or intentional)

- `Diagnostics::product`, `csTransactions`, `spiTransfers`, `busTimeouts` and
  `transferBytes` are still not populated. They are pre-existing diagnostics gaps, not
  caused by 6A–6C; the product can be mirrored from `Sh2Client` later, and the bus
  counters need HAL manager wiring.
- `RecoveryPolicy::resetOnBootTimeout` is still unused. Behaviour is always "reset and
  retry" on boot timeout; the flag is documented as part of the policy surface.
- `SENSOR_STATUS` keeps its 24-byte layout; output and recovery counters are exposed via
  `Diagnostics`, not the wire. This avoids a protocol change.
- Output still occurs synchronously inside the report handler. It is capacity-gated, so
  the added latency is bounded by a ring-buffer copy; a multi-report drain can still wait
  for that copy before the next SPI transfer. There is no queue by design.
- The shared observation time for all packets in one drain is a documented limitation of
  Option A, not a defect.

---

## 7. Files changed in Pass 6

Source:

- `src/output/IOutputSink.h` — capacity contract, outcome vocabulary.
- `src/output/ArduinoSink.h` — `StreamSink::availableForWrite`, non-blocking write.
- `src/output/BinaryEncoder.h/.cpp` — outcome classification, counters, one-log-per-episode.
- `src/BNO085Sensor.h/.cpp` — output observation, recovery FSM, silence/transient triggers,
  manual recovery, terminal state, SHTP reset on device restart, update ordering.
- `src/BNO085Types.h/.cpp` — `ErrorCode::OutputUnavailable`,
  `Status::RecoveryRateLimited`, recovery/output diagnostics, recovery policy fields,
  timestamp comment.
- `src/sh2/Sh2Client.cpp` — comment-only correction of the host timestamp wording.

Tests:

- `test/native/mock_hal.h` — `CapacitySink` with capacity/accept/forced-return knobs.
- `test/native/test_main.cpp` — 10 new cases (§3.5, §4.4, §5.3).

Example and docs:

- `examples/01_SingleSensor/01_SingleSensor.ino` — recovery/output diagnostics, binary-mode
  logging isolation.
- `README.md`, `docs/API.md`, `docs/ARCHITECTURE.md`, `docs/PHASE1-NOTES.md`,
  `docs/PLAN.md`, `test/native/README.md` — focused corrections.
- `output.md` — this audit.

No file was deleted. No history was modified.

---

## 8. Exact verification (current tree)

Host suite:

```
> test\native\build_msvc.bat
61 cases, 438 checks, 0 failures
RESULT: PASS
```

MSVC `/W4` produced no warnings. Static analysis (`cl /analyze` over the portable core,
`build\analyze_msvc.bat`): 0 warnings, 0 errors, exit 0. AddressSanitizer could not be
linked in this MSVC installation (`LNK1104: cannot open file
'clang_rt.asan_static_runtime_thunk-x86_64.lib'`), so the suite was not run under ASan.

The Pass 6 host output was `BNO085Multi host tests (54 cases)` / `348 checks, 0
failures`. The Pass 6 cases were:

```
encoder_output_outcomes
encoder_drop_then_recovery
sensor_boot_timeout
sensor_recovery_succeeds
sensor_recovery_exhausts
sensor_manual_recovery
sensor_transport_recovery
sensor_silence_recovery
sensor_output_drops
sensor_output_unavailable
sensor_observation_timestamp
```

(`sensor_boot_timeout` existed but its expectation changed from terminal `Failed` to
recoverable `Recovering`.)

Current ESP32-S3 example build:

```
> arduino-cli compile --fqbn esp32:esp32:esp32s3 --library . `
    --build-path build\stage1-final3 --warnings all examples\01_SingleSensor
Sketch uses 326806 bytes (24%) of program storage space.
Global variables use 24640 bytes (7%) of dynamic memory.
```

No compiler warnings or errors were emitted. This is a compile result; the physical
result is in `hardware_test_report.md`.

---

## 9. Limitations and assumptions

1. **Pass 6 had no hardware evidence.** The recovery, output and timestamp behaviour
   described above is source-inspected and host-tested only. The later continuous-CS
   read fix does have physical evidence, but Product ID and streaming are still blocked
   by the PS0/WAKE wiring.
2. **Sink honesty.** The atomic-drop guarantee depends on `availableForWrite()` being
   truthful. A sink that over-reports can still produce a partial frame; the counters make
   it visible, and the receiver resynchronises on `A5 5A` + CRC.
3. **UART/USB backpressure.** `HardwareSerial` copies into its TX ring after the capacity
   check; USB CDC may report zero while disconnected, which yields drops rather than
   blocking. Neither path was exercised on hardware in this pass.
4. **Observation time.** Polling means `tHostUs` is not the physical edge. The magnitude
   of the lag is unknown until measured.
5. **Recovery timing.** Backoff and boot deadlines are host-tested with a mock clock; real
   device timing (NRST, SHTP reset, hub boot) is not measured.
6. **Single sensor.** No multi-sensor scheduling, shared-reset grouping or Phase 2 output
   aggregation was added.

---

## 10. Hardware validation still required

The following remain open before Phase 1 can be called physically complete:

1. Product ID retrieval, Rotation Vector configuration and granted interval. Read framing
   is now verified on hardware: the 276 B advertisement is read in one CS assertion.
   Product ID is **blocked** because PS0/WAKE is tied HIGH instead of wired to a GPIO
   (`product=0x00000000`, `sh2-timeout`). A probe that wrote F9 while the hub asserted
   INT received the real `0xF8` responses, so the read path, MOSI and decoder are healthy.
2. Continuous quaternion decode and accuracy bits at 100 Hz.
3. A 10-minute soak with zero CRC/frame errors and zero unexplained sequence gaps.
4. Disconnected INT: reach `Degraded`, start recovery after the silence dwell, never hang.
5. 30 cold boots and repeated manual/automatic recovery cycles.
6. Measure real observation lag and decide whether Phase 2 needs an ISR.
7. Fill the UART/USB ring and confirm `framesNoCapacity`/`framesDropped` increase, no
   service stall occurs, and the PC parser sees the sequence gap.
8. Confirm a real failing device reaches terminal `Failed` after the configured attempts
   and that `recover()` restarts it.

---

## 11. Next milestone

After the hardware items in §10, Phase 2 can start: a `Manager` that owns the bus and
schedules several `Sensor` instances round-robin, with per-sensor recovery and the
existing single-sensor API preserved. No Phase 2 work was started in this pass. Stage 1
remains blocked at Product ID, so Phase 2 has not started.
