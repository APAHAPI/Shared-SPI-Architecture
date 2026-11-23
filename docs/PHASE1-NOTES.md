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
| 5 | Handle fragmented or continued SHTP transfers | Oversize transfers are drained inside the same CS assertion; the payload is not assembled | The advertisement is 276 B (measured; the earlier 552 B figure was a one-bit slip and is invalidated). With `BNO085_RX_TRANSFER_BYTES` = 512 it is buffered normally. Payloads larger than `BNO085_RX_PAYLOAD_BYTES` (320) or transfers larger than 512 B are drained in bounded chunks in the same CS window. A continuation fragment is a separate device presentation and is rejected cleanly rather than clocked with CS still low |
| 6 | Full wake/reset sequencing | Reset pulse implemented; `wakePin` drives PS0/WAKE HIGH through reset and pulses it low before a write | The HAL supports it, but the Stage 1 board has PS0 tied HIGH and `wakePin = -1`. With PS0 HIGH the hub accepts host writes only while it asserts INT; once the boot reads are consumed, INT is idle and the write is dropped. This currently blocks Product ID, so PS0/WAKE must be wired to a GPIO |
| 7 | `Sensor::get()` delivers records | Returns `false` in Phase 1 | Records are delivered through the callback and the output sink; the manager queue that backs `get()` is Phase 2 |
| 8 | Advertisement is ~276 B (implied by the channel math) | 276 B on the tested part (272-byte payload) | Confirmed on hardware with one continuous CS assertion. The earlier 552 B figure was `0x0114 << 1`, a one-bit slip caused by releasing CS between header and body and by applying SPI mode 3 after CS was already low. That measurement is invalid |

> **Superseded (2026-11-19).** Deviation 4 and the host-test count below described the
> Phase 1 delivery. Pass 3 corrected the timestamp IDs/channel handling; pass 4 made
> the per-payload fallback reference-exact (0 ticks, no negative clamp); pass 6 added
> bounded automatic recovery, output loss accounting and the formal polling/observation
> timestamp contract. Current behaviour is in `output.md` and `README.md`. The suite is
> now 61 cases / 438 checks, 0 failures.

## Evidence

**Measured on hardware** (ESP32-S3, Adafruit BNO085, COM11 = CH343 UART bridge,
SPI mode 3 at 1 MHz, pins 12/11/13/10/9/8):

| Item | Result |
| --- | --- |
| Upload | `Hash of data verified`, hard reset, exit 0 |
| Bus + CS | `hal.begin -> ok`; one CS assertion per packet; 0 lock timeouts; 0 bus errors |
| Boot after reset pulse | INT asserted after **93.9 ms** (datasheet typical 90 + 4 ms) |
| First packet | `hdr len=276 cont=0 ch=0 seq=0`; 272-byte payload; contains `1.0.0` |
| Packet consumption | 276 B read inside one CS assertion; `oversize=0`, `badFrag=0`, `seqGaps=0`; INT deasserts |
| `sizeof(Quaternion)` / `sizeof(Diagnostics)` | 44 B / 176 B |
| Product ID | **blocked**: `product=0x00000000`; PS0/WAKE is not wired (see `hardware_test_report.md` §9) |
| Timestamps drift / cold boots | not measured (stages 3-8 not started) |

**Host tests at Phase 1 delivery:** 168 checks across 30 cases, 0 failures, via
`test\native\build_msvc.bat` (MSVC 2022). See `test/native/README.md` for what each
case covers and for the simulated-versus-real distinction. **Current suite:** 61 cases /
438 checks, 0 failures; it includes the continuous-CS framing regressions, bounded
recovery, exhaustion/manual recovery, transport-fault and silence triggers, output loss
accounting, and polling observation time.

## Transport fix: continuous-CS SHTP read (validated on hardware)

**Failure.** `service()` read the 4-byte header in one CS assertion and the payload in a
second. The BNO085 drops and re-presents a transfer when CS is released, so the body read
never returned the payload the header described. Every packet was misparsed, the device
stayed busy, and the original report's 552-byte/`Oversize` symptoms followed.

**Second cause, exposed by the first cold boot after the split read was fixed.**
`SpiBusManager` asserted CS before `SPI.beginTransaction()` applied `SPI_MODE3`. On the
first transfer after `SPI.begin()` the clock changes from idle-low to idle-high while the
device is selected; the device sees one extra edge and the header shifts by one bit:
`0x0114` (276) reads as `0x0228` (552). The final cold-boot capture reproduced exactly
this (`552 > 512`); after reordering, the first boot reads 276.

**Fix.** `IHal` gained `beginRead()/read()/endRead()`; `SpiBusManager` gained
`beginTransaction()/transferInTransaction()/endTransaction()`. One bounded bus lock and
one CS assertion now span the header and the whole payload; `endTransaction()` is
idempotent and releases CS before ending the SPI transaction; `end()` remembers the
active CS pin. The bus is put into `SPI_MODE3` before CS is asserted and is only returned
after CS is high. `ShtpTransport` defers all logging and payload delivery until after
`endRead()`. `BNO085_RX_TRANSFER_BYTES` is 512 and `BNO085_RX_PAYLOAD_BYTES` is 320, so
the 276-byte advertisement (272-byte payload) is buffered and delivered normally; larger
payloads still take the bounded drain path inside the same CS assertion.
`test/native/mock_hal.h` now models CS assertion/deassertion, transaction boundaries,
bytes clocked per assertion, aborts on mid-transfer release, illegal reads and injected
mid-transfer failures.

**Procedure and result (host).** `test\native\build_msvc.bat` -> 61 cases / 438 checks,
0 failures; `/analyze` -> 0 warnings. The 10 framing/recovery regressions are listed in
`hardware_test_report.md` §5.5.

**Procedure and result (COM11, 2026-11-23).**

1. `arduino-cli compile ... --warnings all examples/01_SingleSensor` -> no warnings,
   326 806 B flash / 24 640 B RAM.
2. Upload to COM11, reset, capture at Trace: `hdr len=276 cont=0 ch=0 seq=0`, payload
   `[272]` containing `1.0.0`, then a 20-byte control packet and a 5-byte reset-complete.
3. Final Info build, RTS cold boot: `oversize=0`, `badFrag=0`, `seqGaps=0`,
   `maxService=2367 us`; two complete boot bursts = `packets=6 bytes=602`.
4. `test/hardware/02_TransportDrain` -> `RESULT: PASS - 276-byte advertisement, one CS
   assertion per packet, stream idle` (1 transaction, 0 errors).
5. `test/hardware/01_HeaderProbe` v4 -> 3 packets / 3 transactions, 0 errors.

**Product ID is still blocked** at this point by the PS0/WAKE wiring, not by the read:
`product=0x00000000`, `sh2-timeout`; see `hardware_test_report.md` §9.

**Earlier 552-byte drain result (invalidated).** The 552-byte advertisement and the
"oversized transfer fully drained" PASS were a one-bit slip, not a real transfer. The old
harness drained a re-presented/slipped presentation with split reads; returning to idle
does not prove the payload was consumed coherently. After the fix the same harness reads
276 B in one CS and passes. The old `length - 4` drain-accounting fix is still correct
for genuinely oversized transfers and is covered by the host regressions.

**Compilation:** `examples/01_SingleSensor` compiles for `esp32:esp32:esp32s3`
(ESP32 core 3.3.12), 326 806 B flash (24 %), 24 640 B static RAM (7 %), no warnings with
`--warnings all`.

## Still to verify on hardware

Stages 1–2 above are done. The rest of the Phase 1 exit criteria are open:

1. **Product ID retrieval** through the SH-2 control channel. Currently **blocked**:
   PS0/WAKE is not wired, so the F9 request is ignored when INT is idle
   (`product=0x00000000`, `sh2-timeout`). A probe that wrote F9 while the hub was
   asserting INT received the real `0xF8` Product ID responses, so the read path,
   MOSI and the decoder are proven; only the wake wiring is missing.
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
11. **Sleep/wake**: answered for this board. PS0 is protocol-select at reset and the
    active-low WAKE input afterwards, so it must be a GPIO, not tied HIGH. With it tied
    HIGH, writes to an idle hub are ignored even when the part is fully awake; this is
    the current Product ID blocker. Whether a *sleeping* part behaves differently is
    still unmeasured, but waking is required regardless.
