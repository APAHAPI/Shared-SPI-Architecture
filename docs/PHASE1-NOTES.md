# Phase 1 notes: deviations, evidence, and what is still unverified

Phase 1 delivers the single-sensor SPI baseline: HAL, SPI bus manager, SHTP
transport, minimal SH-2 (init + Rotation Vector), instance-owned state, bounded
waits, diagnostics, one example, host tests and an ESP32-S3 build.

## Deviations from the approved architecture

| # | Approved document said | What was built | Why |
| --- | --- | --- | --- |
| 1 | `Sensor` constructs its own HAL | `Sensor(IHal&)` takes the HAL by reference | Keeps the core free of Arduino headers and host-testable; Phase 2 `Manager` will own the bus and the HALs and hand each sensor a reference |
| 2 | Use the CEVA SH-2 library where possible | Own minimal SHTP + SH-2 for product id, reset, set/get feature and Rotation Vector | The upstream library is a process-wide singleton with unbounded wait loops; both are disqualifying for a shared-bus design. See `UPSTREAM-AUDIT.md` |
| 3 | Frame sequence is a 32-bit field at offset 6 | 16-bit sequence at offset 6 | The table entry (offset 6, 4 bytes) overlaps the payload at offset 8 and makes the frame 36 B, which contradicts the same document's 34 B quaternion frame, 74 B maximum frame, and CRC range `[2, 8+N)`. The 8-byte header is what the bandwidth analysis in `RESOURCE-ANALYSIS.md` assumes, so the header was kept and the field narrowed. `API.md` §4.1 now states the 16-bit rule. Hosts must compute loss modulo 65536 |
| 4 | `tMeas` reconstructed from the datasheet | `tMeas = tEdge - 12000 us + delayTicks * 100` plus a rebase offset | The 0xFA base-timestamp-reference report is recorded but not applied; applying it needs measured hardware behaviour, so it is deferred rather than guessed. See "Unverified on hardware" |
| 5 | Handle fragmented or continued SHTP transfers | Fragments beyond one transfer are rejected as `Oversize`; an unexpected continuation is `Malformed` | With a 192 B payload buffer and a 256 B transfer buffer, `payload + 4 <= transfer`, so every payload a BNO085 emits arrives in a single transfer. Reassembly would cost per-sensor RAM for a path that cannot occur at these sizes |
| 6 | Full wake/reset sequencing | Reset pulse implemented; optional `wakePin` supported but defaults to `-1` | Whether a given board wires PS0/WAKE is a board decision; the HAL supports it and the default assumes the part is awake. Unverified without hardware |
| 7 | `Sensor::get()` delivers records | Returns `false` in Phase 1 | Records are delivered through the callback and the output sink; the manager queue that backs `get()` is Phase 2 |

Deliberately **not** implemented, per instruction: multi-sensor scheduling,
`Manager`, per-sensor ring buffers, Phase 2 timing. No ESP32-side sensor fusion,
no GRV substitution (a GRV config is rejected with `UnsupportedReport`).

## Evidence

- Host tests: 150 checks across 28 cases, 0 failures, via
  `test\native\build_msvc.bat` (MSVC 2022). See `test/native/README.md` for what
  each case covers and for the simulated-versus-real distinction.
- ESP32-S3 Arduino build: `examples/01_SingleSensor` compiles for
  `esp32:esp32:esp32s3` (ESP32 core 3.3.12), 324262 B flash (24%),
  23880 B static RAM (7%).
- Source tree and file inventory are listed in the Phase 1 report.

## Unverified on hardware

None of the following can be settled by simulation. They are the bring-up
checklist for the first board:

1. **Header peek semantics.** The transport reads 4 bytes, then re-reads the
   header plus payload in a second CS assertion, relying on the device
   re-presenting its header. Confirm on a scope.
2. **Timestamp reconstruction.** Verify the 12 000 us edge-to-reception offset
   and the 100 us delay-tick scale against a known motion timing, then decide
   whether the 0xFA rebase is needed.
3. **Sleep/wake.** Confirm whether writes to a sleeping part NAK, and whether a
   board needs PS0/WAKE wired (`wakePin`).
4. **Bus timing.** 1 MHz default and 3 MHz ceiling with the intended wiring;
   check for CRC failures or dropped transfers as stubs and sensors are added.
5. **Reset polarity and duration.** 10 us low pulse plus 1 ms settle is the
   datasheet minimum; verify the part re-advertises on channel 0 afterwards.
6. **Accuracy field.** Confirm the 2-bit status field maps to the reported
   accuracy values on real reports.
