# Latest response — Independent review of `60aef72` (2026-11-27)

## Repository state

- Branch `main`; HEAD `40bbddc` (`Update output.md`).
- `60aef72` (`Log SPI wake and advertisement evidence`) is two commits behind HEAD.
- `git diff 60aef72..HEAD -- src/hal/Esp32SpiHal.cpp src/sh2/Sh2Client.cpp hardware_test_report.md` is empty.
- The only post-`60aef72` change is `output.md` (`2e89b86`, `40bbddc`).
- At review start the worktree was clean; nothing staged.

## Verdict

**APPROVE WITH NOTES.**

`60aef72` is suitable as the current Stage 2 baseline. It is an evidence/logging
and report commit, not a behavioral change. The WAKE and automatic feature-response
logic it documents comes from parent `fe284f3` and is correct for the single-sensor
path.

## What the commit changes

| File | Change |
|---|---|
| `src/hal/Esp32SpiHal.cpp:167-176` | Promotes the successful wake log from Debug to Info. No WAKE timing or GPIO changes. |
| `src/sh2/Sh2Client.cpp:53-64` | Adds an Info advertisement-size summary; full hex remains at Debug. No SH-2 state changes. |
| `hardware_test_report.md` | Updates A–D to A–F, records the 2026-11-27 re-validation, keeps the earlier run as history. |

## Code review

### WAKE / `Esp32SpiHal.cpp`

- GPIO14 is configured in `examples/01_SingleSensor/01_SingleSensor.ino:45`.
- `begin()` configures WAKE HIGH before the reset pulse.
- `prepareWrite()` at `src/hal/Esp32SpiHal.cpp:151-191` drives WAKE LOW, reads the
  pad back, holds LOW until INT or 2 ms, restores HIGH on both paths.
- No path leaves WAKE low after return.
- 2 ms wait is bounded but blocking. Acceptable for two boot writes; note for
  multi-sensor runtime traffic.
- No mutable global state, no cross-sensor coupling, no SPI from ISR, no CS change.

### SH-2 / `Sh2Client.cpp`

- Automatic `0xFC` handling is instance-local: `featurePending_`, `featureId_`,
  `featureDeadlineUs_`, `cmdState_`, `featureState_`.
- `expectFeatureResponse()` arms a 250 ms wait; `pollTimeouts()` converts a missing
  response into a bounded recovery.
- Response matching is by report ID only; no sequence/epoch and no explicit retry.
  Safe for one-shot init, but a stale same-report response is a future risk.
- `60aef72` itself only adds the advertisement log; it does not alter this state
  machine.

## Findings, ranked

| Severity | Finding | File/line | Assessment |
|---|---|---|---|
| Low | Advertisement log omits `sensorId_`; multi-sensor logs would be ambiguous. | `src/sh2/Sh2Client.cpp:60-62` | Acceptable for one sensor; add ID before multi-sensor. |
| Low | Wake log omits sensor/pin and is Info; one synchronous serial line per write can briefly block. | `src/hal/Esp32SpiHal.cpp:172-174` | Acceptable for validation; prefer Debug/non-blocking in production. |
| Info | Comment says "one line per boot," but code logs every channel-0 payload. | `src/sh2/Sh2Client.cpp:57-62` | Comment/edge-case wording only. |
| Note (inherited) | `prepareWrite()` returns true after a 2 ms wake timeout, so the write is still attempted. | `src/hal/Esp32SpiHal.cpp:151-191` | Pre-existing; no timeout observed. |
| Note (inherited) | No stale-response guard and no explicit `0xFC` retry. | `src/sh2/Sh2Client.cpp:449-461`, `516-526` | Safe for current init; review before runtime reconfig/multi-sensor. |

## Evidence verification

- Code supports Product ID `10004148`, firmware `3.2.13`, RV `0x05` at 10 000 us,
  and quaternion decoding.
- Host suite: 63 cases / 449 checks / 0 failures (same source tree).
- ESP32-S3 build: 327 274 B flash / 24 640 B RAM, no warnings.
- Flash: 327 424 B, hash verified.
- Prior hardware run on the same source: 1 738 reports over 17.4217 s = 99.70 Hz,
  zero link errors, one host-induced gap at the blocking `d` print.
- No new hardware test was run for this commit; `60aef72` changes no behavior.

## Remaining observations

| Observation | Classification |
|---|---|
| RV accuracy `unreliable` while stationary | Acceptable for current milestone; motion/calibration still untested. |
| No explicit `0xFC` retry | Acceptable now; must address before multi-sensor to avoid reset-on-loss. |
| Blocking `d` print drops one sample | Acceptable; diagnostic text path only. Use binary/non-blocking in production. |
| No soak/fault/multi-sensor testing | Acceptable for this commit; required before Phase 2/multi-sensor. |

None is a commit blocker for `60aef72`.

## Source diff from the commit

```diff
diff --git a/src/hal/Esp32SpiHal.cpp b/src/hal/Esp32SpiHal.cpp
index edbc900..31bd8e0 100644
--- a/src/hal/Esp32SpiHal.cpp
+++ b/src/hal/Esp32SpiHal.cpp
@@ -167,8 +167,10 @@ bool Esp32SpiHal::prepareWrite() {
     if (intAsserted()) {
       const uint32_t waited = micros() - start;
       digitalWrite(config_->wakePin, HIGH);
-      if (log_.enabled(LogLevel::Debug, LogSpi)) {
-        log_.log(LogLevel::Debug, LogSpi, "wake: INT after %luus (pad=%d)",
+      // One line per host write: the pad readback proves WAKE was actually
+      // driven low (no 3V3 tie), and the interval is the observed wake time.
+      if (log_.enabled(LogLevel::Info, LogSpi)) {
+        log_.log(LogLevel::Info, LogSpi, "wake: INT after %luus (pad=%d)",
                  static_cast<unsigned long>(waited), readback);
       }
       return true;
diff --git a/src/sh2/Sh2Client.cpp b/src/sh2/Sh2Client.cpp
index a3003c7..528a6bb 100644
--- a/src/sh2/Sh2Client.cpp
+++ b/src/sh2/Sh2Client.cpp
@@ -53,7 +53,15 @@ void Sh2Client::handlePayload(uint8_t channel, const uint8_t* payload,
                               uint16_t len, uint32_t tUs) {
   switch (channel) {
     case shtp::kChannelCommand:
-      if (log_) log_->hex(LogLevel::Debug, LogShtp, "advertisement", payload, len);
+      if (log_) {
+        // One line per boot: the channel-0 advertisement is the proof that the
+        // 276-byte transfer was read coherently (272-byte payload + 4-byte SHTP
+        // header). The full hex remains available at Debug.
+        log_->log(LogLevel::Info, LogShtp,
+                  "advertisement payload=%u transfer=%u",
+                  static_cast<unsigned>(len), static_cast<unsigned>(len + 4u));
+        log_->hex(LogLevel::Debug, LogShtp, "advertisement", payload, len);
+      }
       break;
     case shtp::kChannelExecutable:
       handleExecutable(payload, len);
```

The `hardware_test_report.md` portion of the commit is documentation-only; its full
diff is available with:

```
git show 60aef72 -- hardware_test_report.md
```

## Final state

At review start: clean worktree, `git diff` empty. This handoff updates
`output.md` only; no source, test, report, or hardware configuration was changed.
Nothing was staged, committed, pushed, reset, or discarded.
