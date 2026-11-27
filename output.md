# Latest response (2026-11-27) — Stage 2 review

## Repository state at review start

- Branch `main`; HEAD `2e89b86` (`Refresh Stage 2 validation handoff`).
- Working tree was clean; no staged files.
- The described eight-file change set is already committed across `fe284f3`
  (`Fix BNO085 WAKE and feature initialization`) and `60aef72`
  (`Log SPI wake and advertisement evidence`), not uncommitted.
- `2e9a2d8..HEAD` modifies 8 implementation/report files plus `output.md`.
  No files were added or deleted.

## Verdict

No blocking defect found. The WAKE fix and the delayed feature-response fix are
correct for the single-sensor path and were independently re-verified on the
board. The code is ready for the current commit line; there is nothing left to
commit in the eight-file set.

## Findings, ranked

1. **Medium, pre-existing:** runtime `Sensor::configure()` still sends a Set
   Feature without arming `expectFeatureResponse()`. The hub's automatic 0xFC is
   therefore counted as unexpected, and `grantedIntervalUs` / diagnostics are not
   refreshed. Not exercised by the current init path, but relevant to Phase 2 and
   multi-sensor runtime reconfiguration.
2. **Low:** `Esp32SpiHal::prepareWrite()` returns `true` even when WAKE does not
   assert INT within 2 ms. The warning is logged, but the caller still writes.
   With PS0 wired, a failed wake can therefore produce a command timeout and
   recovery instead of a clean send failure.
3. **Low:** the 2 ms wake wait is a blocking loop, and the 250 ms feature-apply
   window is a compile-time constant rather than a `ManagerConfig` value. Fine for
   two boot writes and the measured ~119 ms response, but not tunable for a slow
   hub or many sensors.
4. **Low:** `handleGetFeatureResponse()` accepts any granted interval, including
   zero, and matches only by report ID. A stale 0xFC for the same report could
   satisfy a later expectation. The new 250 ms path avoids the observed problem,
   but there is no explicit stale/zero guard.
5. **Info:** `Sh2Client::getFeature()` is now unused by the production state
   machine. It is harmless dead API.

## Fix assessment

- **WAKE:** correct. `begin()` drives WAKE HIGH before the NRST pulse; WAKE is
  driven LOW only in `prepareWrite()`, held until INT (bounded at 2 ms), then
  restored HIGH. Pad readback is captured while low.
- **Feature application:** correct for init. Set Feature is sent once; the host
  waits for the hub's automatic Get Feature Response for 250 ms instead of
  issuing an immediate Get Feature. Measured response was ~119 ms and granted
  10 000 us.

## Tests and build

- Host suite: **63 cases, 449 checks, 0 failures**, `RESULT: PASS`.
- Static analysis: completed with no diagnostics.
- ESP32-S3 build: **327 274 B flash (24%)**, **24 640 B RAM (7%)**, no warnings.
- Flash: 327 424 B, `Hash of data verified.`, exit 0.

## Physical repeat on COM11

The current review build was flashed (no wiring change) and captured for 18 s at
115 200 8N1 to `build/stage2/review_stage2.log` (1784 lines).

- Advertisement: payload 272 / transfer 276.
- WAKE: INT after 856 us and 548 us, `pad=0` both times.
- Product ID: `part=10004148 build=6 fw=3.2.13 resetCause=4`.
- Streaming began at 0.596 s; granted interval 10 000 us; `tx=2`.
- 1738 reports over 17.4217 s = **99.70 Hz** using `(N-1)/span`.
- `spiErrors=0 busTimeouts=0 badFrag=0 oversize=0 seqGaps=0 spurious=0`
  `sh2Timeouts=0 badResp=0`; `resets=1`; recovery attempts 0.
- One host-induced sequence gap (`118 -> 120`) at the 50 ms `d` print.
- All quaternions finite; accuracy remained `unreliable` while stationary.

## Remaining validation gaps

- No motion/calibration test, so RV accuracy remains `unreliable`.
- No soak, fault injection, output-loss-under-load, recovery-under-load, or
  multi-sensor test.
- No test of runtime `configure()`, the wake-timeout path, or a stale/zero
  Get Feature Response.

## Final git state

At review start the worktree was clean and `git diff` was empty. The only change
made by this review handoff is this `output.md` update; no source, test, report,
or hardware configuration was changed.
