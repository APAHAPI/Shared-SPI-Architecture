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
| 4 | `tMeas` reconstructed from the datasheet | `tMeas = tObservation + (referenceDelta + delayTicks) × 100 µs`; `referenceDelta` starts at 0 per payload, `0xFB` sets `−timebase`, `0xFA` adds a signed rebase | Pass 3 corrected the report IDs/channel and pass 4 made the per-payload fallback reference-exact. `tObservation` is the polled host time, not the physical INT edge (see §6.4 in `ARCHITECTURE.md`) |
| 5 | Handle fragmented or continued SHTP transfers | Oversize transfers are drained in bounded chunks following continuation headers; the payload is not assembled | The real advertisement is 552 B (measured), larger than the 256 B transfer buffer and the 192 B payload buffer. Assembling everything would cost ~1.1 KB more per sensor for a payload nothing consumes. The drain is bounded to 8 fragments |
| 6 | Full wake/reset sequencing | Reset pulse implemented; optional `wakePin` supported but defaults to `-1` | Whether a given board wires PS0/WAKE is a board decision; the HAL supports it and the default assumes the part is awake |
| 7 | `Sensor::get()` delivers records | Returns `false` in Phase 1 | Records are delivered through the callback and the output sink; the manager queue that backs `get()` is Phase 2 |
| 8 | Advertisement is ~276 B (implied by the channel math) | 552 B on the tested part | Measured on hardware. `MAX_PAYLOAD_IN`-style assumptions in the upstream docs are 4 × 68 B entries; this firmware sends twice that. Any buffer sizing based on 276 B would fail at the first packet of every cold boot |

> **Superseded (2026-11-19).** Deviation 4 and the host-test count below described the
> Phase 1 delivery. Pass 3 corrected the timestamp IDs/channel handling; pass 4 made
> the per-payload fallback reference-exact (0 ticks, no negative clamp); pass 6 added
> bounded automatic recovery, output loss accounting and the formal polling/observation
> timestamp contract. Current behaviour is in `output.md` and `README.md`. The suite is
> now 54 cases / 348 checks.

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
| Transport drain of the 552 B advertisement | one `Oversize` result, 4 bus transactions (peek + 256 + 256 + 40), device idle afterwards, 0 bad fragments, 0 short fragments, 0 rx errors |
| Timestamps drift / cold boots | not measured (stages 3–8 not started) |

**Host tests at Phase 1 delivery:** 168 checks across 30 cases, 0 failures, via
`test\native\build_msvc.bat` (MSVC 2022). See `test/native/README.md` for what each
case covers and for the simulated-versus-real distinction. **Current suite:** 54 cases /
348 checks, 0 failures; it now includes bounded recovery, exhaustion/manual recovery,
transport-fault and silence triggers, output loss accounting, and polling observation
time.

## Transport fix: oversized transfer drain (validated on hardware)

**Failure.** Every cold boot, `service()` returned `Oversize` for the 552-byte
channel-0 advertisement and left the stream desynchronised: the device kept INT
asserted, and each following `service()` parsed payload bytes as a header, so it
reported malformed/oversize results indefinitely. Probe v2 showed the same symptom as
295 consecutive transfer failures.

**Root cause.** Byte accounting, not protocol design. `service()` peeks the 4-byte
header with a non-consuming read and then re-reads the transfer from the start, so
`length` (which counts the whole transfer) is still entirely on the wire. The oversize
drain subtracted the header (`remaining = length - 4`) and clocked out only 548 of 552
bytes. The device re-presented the surviving 4 payload bytes, and the transport read
them as a length/channel/sequence. Same off-by-4 on each continuation fragment
(`lenNext - 4`). `discard()` was already correct because it drains `length` bytes from
the start.

**Fix.** Two expressions changed to drain the full `length` / `lenNext`; plus removal
of an `rxErrors` increment that flagged a legal, fully-drained transfer as a bus
error (it made a healthy boot indistinguishable from a desync in diagnostics).
Nothing else changed: per-sensor SHTP context, shared bus manager, buffer sizes,
result codes, timeouts and the bounded 8-fragment guard are untouched.

**Procedure.**

1. `test\native\build_msvc.bat` — 54 cases / 348 checks (30 cases / 168 checks at the
   time of this fix).
2. Re-introduce `length - 4` temporarily: 6 checks fail in the two regression cases,
   confirming the tests detect the defect rather than merely coexisting with it.
3. Compile `test/hardware/02_TransportDrain` for `esp32:esp32:esp32s3`.
4. `arduino-cli upload -p COM11 --fqbn esp32:esp32:esp32s3 --input-dir build/transportdrain`.
5. `arduino-cli monitor -p COM11 -c baudrate=115200` for 20 s.

The harness drives the real `ShtpTransport` — reset, then `service()` while INT is
asserted — with no SH-2 traffic, so it does not begin stage 3.

**Observed (COM11, 2026-11-16):**

```
=== BNO085Multi transport drain validation ===
hal.begin -> ok
boot: INT asserted after 93889 us (up)
servicing while INT asserted:
  service[1] -> Oversize
  INT deasserted: device idle
service calls=1 oversize=1 ok=0 other=0
final service -> NoData (idle, no stranded tail)
stats: packets=0 bytes=0 oversizePayloads=1 badFragments=0 shortFragments=0
       badChannels=0 seqGaps=0 rxErrors=0
bus transactions=4 lockTimeouts=0 busErrors=0
RESULT: PASS - oversized transfer fully drained, stream idle
```

Four bus transactions is 1 peek + 3 reads totalling 552 bytes, i.e. the header is
re-read as part of the first chunk and exactly the transfer is consumed. Log:
`build/transportdrain_run3.log`.

**Not proven by this.** The drain is fixed and the stream returns to idle after the
advertisement; nothing here exercises SH-2, report configuration, timestamps or
soak behaviour, which remain stages 3–8.

**Compilation:** `examples/01_SingleSensor` compiles for `esp32:esp32:esp32s3`
(ESP32 core 3.3.12), 326 142 B flash (24 %), 23 968 B static RAM (7 %), no warnings with
`--warnings all`.

## Still to verify on hardware

Stages 1–2 above are done. The rest of the Phase 1 exit criteria are open:

1. **Product ID retrieval** through the SH-2 control channel.
2. **Rotation Vector configuration at 100 Hz** and verification of the granted
   interval (set-feature, then get-feature).
3. **Continuous quaternion output** and decode correctness. Includes confirming the
   report layout, the accuracy bits, and that `rawW..rawZ` match the wire values.
4. **10-minute soak** at 100 Hz: zero frame errors, zero sequence gaps.
5. **Disconnected INT line**: the sensor must reach `Degraded`, then start bounded
   recovery after `silenceRecoveryMs`, and never hang the sketch.
6. **30 consecutive cold boots.**
7. **Adafruit comparison** on identical wiring.
8. **Timestamp reconstruction**: confirm the 100 µs delay-tick scale and the signed
   `0xFB`/`0xFA` reference arithmetic against a known motion; measure the polled
   observation lag to decide whether Phase 2 needs an ISR edge capture.
9. **Recovery timing**: measure the real boot/backoff durations, confirm attempts stop
   after `maxAttempts`, and that a manual `recover()` after terminal failure works.
10. **Output backpressure**: fill the UART/USB ring and confirm frames are dropped with
    `framesNoCapacity`/`framesDropped` increasing, no stall in `service`, and a
    sequence gap visible to the PC parser.
9. **Sleep/wake**: whether writes to a sleeping part fail, and whether PS0/WAKE must be
   wired.
