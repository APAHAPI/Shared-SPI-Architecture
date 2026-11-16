# Phase 1 notes: what is built, what deviates, what is proven

Phase 1 delivers the single-sensor SPI baseline: HAL, SPI bus manager, SHTP
transport, minimal SH-2 (init + Rotation Vector), instance-owned state, bounded
waits, diagnostics, one example, host tests and an ESP32-S3 build.

## What Phase 1 implements

| Area | Files |
| --- | --- |
| CRC-16/CCITT-FALSE and binary framing | `util/Crc16`, `output/OutputProtocol`, `output/BinaryEncoder` |
| SHTP transport (per instance) | `shtp/ShtpTransport`, `shtp/ShtpProtocol` |
| SH-2 control and report decode | `sh2/Sh2Client`, `sh2/Sh2Codes`, `sh2/Sh2Reports` |
| Bus, HAL, diagnostics, state machine | `hal/SpiBusManager`, `hal/Esp32SpiHal`, `diag/*`, `BNO085Sensor` |

Not implemented, by instruction: multi-sensor scheduling, `Manager`, per-sensor
ring buffers, Phase 2 timing. No ESP32-side sensor fusion, no GRV substitution (a
GRV configuration is rejected with `UnsupportedReport`).

## Deviations from the approved architecture

| # | Approved document said | What was built | Why |
| --- | --- | --- | --- |
| 1 | `Sensor` constructs its own HAL | `Sensor(IHal&)` takes the HAL by reference | Keeps the core free of Arduino headers and host-testable; Phase 2 `Manager` will own the bus and the HALs and hand each sensor a reference |
| 2 | Use the CEVA SH-2 library where possible | Own minimal SHTP + SH-2 for product id, reset, set/get feature and Rotation Vector | The upstream library is a process-wide singleton with unbounded wait loops; both are disqualifying for a shared-bus design. See `UPSTREAM-AUDIT.md` |
| 3 | Frame sequence is a 32-bit field at offset 6 | 16-bit sequence at offset 6 | The table entry (offset 6, 4 bytes) overlaps the payload at offset 8 and makes the frame 36 B, which contradicts the same document's 34 B quaternion frame, 74 B maximum frame, and CRC range `[2, 8+N)`. The 8-byte header is what the bandwidth analysis in `RESOURCE-ANALYSIS.md` assumes, so the header was kept and the field narrowed. `API.md` §4.1 now states the 16-bit rule. Hosts must compute loss modulo 65536 |
| 4 | `tMeas` reconstructed from the datasheet | `tMeas = tEdge - 12000 us + delayTicks * 100` plus a rebase offset | The 0xFA base-timestamp-reference report is recorded but not applied; applying it needs measured hardware behaviour, so it is deferred rather than guessed |
| 5 | Handle fragmented or continued SHTP transfers | Oversize transfers are drained in bounded chunks following continuation headers; the payload is not assembled | The real advertisement is 552 B (measured), larger than the 256 B transfer buffer and the 192 B payload buffer. Assembling everything would cost ~1.1 KB more per sensor for a payload nothing consumes. The drain is bounded to 8 fragments |
| 6 | Full wake/reset sequencing | Reset pulse implemented; optional `wakePin` supported but defaults to `-1` | Whether a given board wires PS0/WAKE is a board decision; the HAL supports it and the default assumes the part is awake |
| 7 | `Sensor::get()` delivers records | Returns `false` in Phase 1 | Records are delivered through the callback and the output sink; the manager queue that backs `get()` is Phase 2 |
| 8 | Advertisement is ~276 B (implied by the channel math) | 552 B on the tested part | Measured on hardware. `MAX_PAYLOAD_IN`-style assumptions in the upstream docs are 4 × 68 B entries; this firmware sends twice that. Any buffer sizing based on 276 B would fail at the first packet of every cold boot |

## Evidence

**Measured on hardware** (ESP32-S3, Adafruit BNO085, COM11 = CH343 UART bridge,
SPI mode 3 at 1 MHz, pins 12/11/13/10/9/8):

| Item | Result |
| --- | --- |
| Upload | `Hash of data verified`, hard reset, exit 0 |
| Bus + CS | `hal.begin -> ok`; 4 transactions; 0 lock timeouts; 0 bus errors |
| Boot after reset pulse | INT asserted after **93 665 µs** (datasheet typical 90 + 4 ms) |
| First packet | `len=552 ch=0 seq=0`, continuation bit clear |
| Packet consumption | 552 B drained in 3 bounded chunks; then INT deasserted (no leftover fragments) |
| `sizeof(Quaternion)` / `sizeof(Diagnostics)` | 44 B / 124 B |

**Host tests:** 150 checks across 28 cases, 0 failures, via
`test\native\build_msvc.bat` (MSVC 2022). See `test/native/README.md` for what each
case covers and for the simulated-versus-real distinction.

**Compilation:** `examples/01_SingleSensor` compiles for `esp32:esp32:esp32s3`
(ESP32 core 3.3.12), 324 262 B flash (24 %), 23 880 B static RAM (7 %).

## Still to verify on hardware

Stages 1–2 above are done. The rest of the Phase 1 exit criteria are open:

1. **Product ID retrieval** through the SH-2 control channel.
2. **Rotation Vector configuration at 100 Hz** and verification of the granted
   interval (set-feature, then get-feature).
3. **Continuous quaternion output** and decode correctness. Includes confirming the
   report layout, the accuracy bits, and that `rawW..rawZ` match the wire values.
4. **10-minute soak** at 100 Hz: zero frame errors, zero sequence gaps.
5. **Disconnected INT line**: the sensor must fail with a diagnostic inside its
   bounded timeout and never hang the sketch.
6. **30 consecutive cold boots.**
7. **Adafruit comparison** on identical wiring.
8. **Timestamp reconstruction**: confirm the 12 000 µs edge-to-reception offset and the
   100 µs delay-tick scale against a known motion, then decide whether the 0xFA rebase
   is needed.
9. **Sleep/wake**: whether writes to a sleeping part fail, and whether PS0/WAKE must be
   wired.
