# Latest response (2026-11-27)

Validation and report update are complete. The report and the two logging-source changes were committed externally as `60aef72`; I made no new commit. This handoff file, `output.md`, is the only working-tree modification.

## Final checkpoint

**HEAD:** `60aef72` (`Log SPI wake and advertisement evidence`)

**Working tree:**
```
 M output.md
```

`git diff --check` exits 0. Nothing staged or committed by this turn. The LF→CRLF messages are Git's normal line-ending notices.

**Commit `60aef72` contains:**
```
 hardware_test_report.md | 212 +++++++++++++++++++++++++++++-------------------
 src/hal/Esp32SpiHal.cpp |   6 +-
 src/sh2/Sh2Client.cpp   |  10 ++-
```

## Report changes (now committed)

- **§13.4** carries the current identity: HEAD `fe284f3` at test time, `build/stage2-af`, 327 274 B flash / 24 640 B RAM, 327 424 B written and hash-verified, host suite 63 cases / 449 checks, static analysis clean.
- **§13.5** is the fresh `serial_stage2_af.log` capture.
- **§13.6** explicitly lists **A–F**, all PASS.
- **§13.7–13.9** report 1738 reports / 17.432 s, 99.65 Hz overall, 99.82 Hz steady, all finite; zero link errors; one host-induced drop during `d`.
- **§13.11** is a compact 2026-11-26 bring-up record, so current acceptance evidence comes first.
- Header, verdict table, reproduce commands, and artifact list are reconciled with the current build and capture.

## Test outcome

All A–F stages passed:

| Stage | Result |
|---|---|
| A. Reset and WAKE | PASS — WAKE idle HIGH, reset pulse, INT after 852 µs / 547 µs, `pad=0` |
| B. Advertisement / reset-complete | PASS — payload 272, transfer 276 at 0.365 s; reset-complete 0.428 s |
| C. Product ID | PASS — `part=10004148 build=6 fw=3.2.13 resetCause=4` |
| D. SH-2 initialization | PASS — streaming at 0.581 s, `resets=1`, no recovery |
| E. Rotation Vector config | PASS — requested/granted 10 000 µs |
| F. Quaternions | PASS — 1738 reports, 99.65 Hz overall, all finite |

**Open observations, not failures:** RV accuracy remained `unreliable` while stationary; no explicit get-feature retry if the automatic `0xFC` is lost; the blocking `d` print drops one sample; no soak, fault, or multi-sensor testing was performed.

## Evidence

- Host suite: 63 cases / 449 checks, 0 failures.
- Static analysis: exit 0, 0 warnings.
- ESP32-S3 build: 327 274 B flash (24%), 24 640 B RAM (7%), no warnings.
- Flash: COM11, 327 424 B written, hash verified, exit 0.
- Capture: `build/stage2/serial_stage2_af.log`, 1776 lines, 18 s.
- Link counters: `spiErrors=0 busTimeouts=0 badFrag=0 oversize=0 seqGaps=0 spurious=0 sh2Timeouts=0 badResp=0`; `resets=1`; `tx=2`.

## Next action

Review commit `60aef72`. Do not begin Phase 2, soak, fault injection, or multi-sensor work.
