# Resource analysis — memory, bandwidth, latency, startup

Phase 0 deliverable. **All numbers below are calculated from the declared design and
the datasheet, not measured.** Every table states its assumptions so Phase 1 and
Phase 5 can replace estimates with measurements. No hardware run has been performed.

Sources for device constants: BNO08X datasheet 1000-3927 v1.8, §6.5.2 (SPI timing),
§6.5.3 (startup), §1.2.4.1 (host interrupt timeout), §1.3.1 (SHTP header), §1.3.5
(timestamps).

---

## 1. Fixed assumptions

| Assumption | Value | Source / note |
| --- | --- | --- |
| SPI clock (bring-up / default) | 1 MHz | Adafruit reference runs here successfully |
| SPI clock (maximum) | 3 MHz | Datasheet §6.5.2 |
| SPI mode | 3 (CPOL=1, CPHA=1), MSB first | Datasheet §6.5.2 |
| Rotation Vector payload | 14 B | SH-2 report 0x05 (report id, seq, status, delay, i, j, k, real, accuracy) |
| SHTP header | 4 B | Datasheet §1.3.1 |
| Header transmitted twice per transfer | +4 B | Datasheet §1.3.1: read 4 bytes, then repeat the read |
| Wire bytes per RV report | 22 B = 176 bit | 4 (peek) + 4 (header) + 14 (payload) |
| Per-report software + CS overhead | 12 µs (estimate) | 2 CS transactions, `SPI.transferBytes` overhead, decode, timestamp, enqueue. **Must be measured.** |
| Report rate cases | 100 / 200 / 400 Hz | 100 Hz default |

---

## 2. Wire cost of one report

| Clock | Pure wire time | + 12 µs overhead | Effective per report |
| --- | --- | --- | --- |
| 1 MHz | 176.0 µs | +12 | **188.0 µs** |
| 2 MHz | 88.0 µs | +12 | **100.0 µs** |
| 3 MHz | 58.7 µs | +12 | **70.7 µs** |

Notes:

- The 4-byte header is paid twice because the host must first learn the length. A
  device with several reports queued sends them in one payload, amortising this cost;
  the table is the worst case (one report per transfer).
- Enabling extra reports increases payload bytes and therefore bus time roughly
  linearly, and also increases interrupt rate.

---

## 3. SPI bandwidth by sensor count

Quaternion stream only, one report per transfer, 12 µs overhead.

### 3.1 At 3 MHz (maximum)

| Sensors | Reports/s @100 Hz | Bus time/s | Utilisation | @200 Hz | @400 Hz |
| --- | --- | --- | --- | --- | --- |
| 1 | 100 | 7.1 ms | 0.7 % | 1.4 % | 2.8 % |
| 5 | 500 | 35.3 ms | 3.5 % | 7.1 % | 14.1 % |
| 10 | 1 000 | 70.7 ms | 7.1 % | 14.1 % | 28.3 % |
| 15 | 1 500 | 106.0 ms | 10.6 % | 21.2 % | 42.4 % |
| 20 | 2 000 | 141.3 ms | 14.1 % | 28.3 % | 56.5 % |

### 3.2 At 1 MHz (bring-up default)

| Sensors | @100 Hz | @200 Hz | Ceiling (80 % budget) |
| --- | --- | --- | --- |
| 1 | 1.9 % | 3.8 % | ~4 250 reports/s |
| 5 | 9.4 % | 18.8 % | |
| 10 | 18.8 % | 37.6 % | |
| 15 | 28.2 % | 56.4 % | |
| 20 | 37.6 % | 75.2 % | ≈ 212 Hz per sensor |

### 3.3 Practical ceiling

At an 80 % bus budget, ignoring init traffic and foreign devices:

| Clock | Max report throughput | With 20 sensors | With 10 sensors |
| --- | --- | --- | --- |
| 1 MHz | ≈ 4 250 /s | ≈ 212 Hz each | ≈ 425 Hz each |
| 2 MHz | ≈ 8 000 /s | ≈ 400 Hz each | ≈ 800 Hz each |
| 3 MHz | ≈ 11 300 /s | ≈ 550 Hz each | > 1 kHz each |

**No promise is made that 20 sensors sustain 400 Hz.** The honest statement is:
20 sensors at 100 Hz leave ~86 % headroom at 3 MHz; 20 at 200 Hz fit comfortably at
2–3 MHz; 20 at 400 Hz are feasible only at 3 MHz with a clean harness, no foreign bus
traffic, and the measured overhead at or below the 12 µs assumption. Phase 5 measures
it.

Interrupt load: 2 000 INT/s at 20 × 100 Hz, each ISR ≈ 1–2 µs ⇒ ≈ 0.3 % CPU.

---

## 4. Latency analysis

### 4.1 Budget from the device

The BNO085 waits ≈10 ms for the host to respond to an asserted `H_INTN`, then
deasserts and retries (datasheet §1.2.4.1). Service latency is therefore a functional
limit, not just a performance metric: exceed it and the device retries, producing
duplicate reports.

### 4.2 Worst-case service latency

| Scenario | 1 MHz | 3 MHz |
| --- | --- | --- |
| 20 sensors assert INT simultaneously | 20 × 188 µs = 3.76 ms | 20 × 70.7 µs = 1.41 ms |
| Plus one `update()` budget (2 ms) | 5.76 ms | 3.41 ms |
| Margin to 10 ms | 4.2 ms | 6.6 ms |

A single sensor is serviced in one visit (≤ 3 transfers, ≈ 210 µs at 3 MHz), so the
common case is far below the worst case.

### 4.3 Duplicate-report handling

If the host does exceed the window (USB stall, long application callback, foreign SPI
transfer), the device retries and the same measurement can be delivered twice. The
aggregator detects this via the per-report-ID 8-bit `reportSeq`: an unchanged
sequence with an unchanged `tMeasUs` is a retransmission, counted as
`diag.duplicates` and not forwarded as new motion. A skipped sequence increments
`diag.missedReports`.

---

## 5. Memory by sensor count

### 5.1 Per-sensor footprint (design target)

| Component | Bytes | Contents |
| --- | --- | --- |
| `ShtpTransport` | 564 | rxTransfer 256, rxPayload 192, txTransfer 32, 8 channel states, counters, cursor |
| `Sh2Client` | 200 | pending request 16, product IDs 88, feature cache 56, timestamp state 16, counters 24 |
| `BNO085Sensor` runtime | 432 | latest RV 24, latest aux 72, queue 8 × 32 = 256, diagnostics 64, FSM 16 |
| **Per sensor** | **≈ 1 196 (1.2 KB)** | |

Fixed overhead (independent of sensor count): manager 256, sensor table 160,
aggregator lanes 480, output buffer 64, scheduler masks 16, bus manager 64, recovery
policy 32 → **≈ 1.1 KB**.

### 5.2 Totals

| Sensors | Sensor block | Fixed | **Total** | % of 512 KB SRAM |
| --- | --- | --- | --- | --- |
| 1 | 1.2 KB | 1.1 KB | **2.3 KB** | 0.4 % |
| 5 | 5.8 KB | 1.1 KB | **6.9 KB** | 1.3 % |
| 10 | 11.7 KB | 1.1 KB | **12.8 KB** | 2.5 % |
| 15 | 17.5 KB | 1.1 KB | **18.6 KB** | 3.6 % |
| 20 | 23.4 KB | 1.1 KB | **24.5 KB** | 4.8 % |

ESP32-S3 has 512 KB of on-chip SRAM (some variants add PSRAM, which this library does
not require). Outside the library, allow ~10–20 KB for the Arduino core, USB CDC
buffers, and the sketch's own stack — i.e. the 20-sensor configuration still fits with
large margin.

### 5.3 Lean configuration

`BNO085_RX_TRANSFER_BYTES=128`, `BNO085_RX_PAYLOAD_BYTES=128`, queue depth 4, no aux
report retention:

| Sensors | Per sensor | Total |
| --- | --- | --- |
| 1 | 0.8 KB | 1.9 KB |
| 20 | 15.4 KB | **16.5 KB** |

### 5.4 Comparison with the CEVA reference

| Implementation | Per device | 20 devices |
| --- | --- | --- |
| CEVA `sh2_t` + `shtp_t` as shipped | ≈ 2.7 KB (1024 B in-transfer + 1024 B in-payload + 128 B out + 400 B SH-2) | ≈ 54 KB |
| This design | ≈ 1.2 KB | ≈ 23 KB |

### 5.5 Allocation policy

- Exactly **one** allocation at `begin()` for the sensor array (`new Sensor[n]`, n ≤ 20),
  plus one bus-manager object. Nothing is allocated per report, per packet or during
  recovery.
- `BNO085_STATIC_STORAGE` build option replaces the heap allocation with a `.bss` pool
  (`Sensor pool[BNO085_MAX_SENSORS]`), for projects that forbid dynamic allocation.
- Queue depths are compile-time constants so worst-case memory is knowable at build
  time.

---

## 6. Initialization time budget

Per sensor, typical values:

| Step | Time | Source |
| --- | --- | --- |
| Reset pulse | 1 ms | datasheet minimum is 10 ns; 1 ms is a safe, negligible margin |
| Boot until `H_INTN` | 94 ms typ (90 + 4) | datasheet §6.5.3 |
| Advertisement + reset-complete + SH-2 init message | ~1–3 transfers ≈ 0.1 ms bus | datasheet §5.2.1 |
| Product ID request/response (4 entries) | ~8 ms (assumed 2 ms round trip each) | SH-2 control channel |
| Set feature (RV) | ~3 ms | |
| Get feature (verify granted interval) | ~3 ms | |
| **Per sensor after boot** | **≈ 14 ms** | |

| Scenario | Estimate |
| --- | --- |
| 1 sensor, typical | ~110 ms |
| 20 sensors, boots overlapped, config serialised | ~100 ms + 20 × 14 ms ≈ **380 ms** |
| 20 sensors, boot budget exhausted (300 ms each, still overlapped) | ~300 ms + 280 ms ≈ **0.6 s** |
| 20 sensors, every config step timing out (100 ms each, 4 steps) | ≈ 8 s, then all marked `Failed` |
| 20 sensors on a shared reset line, worst case | same as above; group is reconfigured together |

The last row is the important one: startup is **bounded**, and a dead device costs a
known number of milliseconds rather than an indefinite hang.

---

## 7. Output link budget

Quaternion frame = 8 B header + 24 B payload + 2 B CRC = **34 B**.

| Stream | Frames/s | Bandwidth |
| --- | --- | --- |
| 1 sensor @ 100 Hz | 100 | 3.4 kB/s (27 kbit/s) |
| 20 sensors @ 100 Hz | 2 000 | 68 kB/s (544 kbit/s) |
| 20 sensors @ 200 Hz | 4 000 | 136 kB/s (1.09 Mbit/s) |
| 20 sensors @ 400 Hz | 8 000 | 272 kB/s (2.18 Mbit/s) |

Link options:

| Link | Practical throughput | Verdict |
| --- | --- | --- |
| Native USB CDC (S3 USB-OTG) | ~1–3 MB/s | 20 @ 400 Hz fits with room for status traffic |
| UART bridge @ 921 600 baud (≈92 kB/s) | ~2 700 frames/s | 20 @ 100 Hz fits; 200 Hz does not |
| UART bridge @ 460 800 baud | ~1 350 frames/s | 10 @ 100 Hz fits; 20 does not |

The Arduino `Serial` on an S3 with native USB is the CDC device; the baud value is
ignored by USB but is still set so the same sketch works through a UART bridge.

Status/diagnostic traffic is additional: one `SENSOR_STATUS` per sensor per second by
default (20 × 34 B/s ≈ 680 B/s, negligible). Set `statusIntervalMs = 0` to disable.

---

## 8. What these numbers do not cover

| Not measured | Why it matters | When resolved |
| --- | --- | --- |
| Per-report overhead (assumed 12 µs) | Dominates the bandwidth model at high rates | Phase 1: instrument `micros()` around a service call |
| ESP32 SPI driver behaviour at 3 MHz with 20 CS lines | CS setup/hold and GPIO toggling cost | Phase 1/5 |
| Harness signal integrity | 3 MHz may be unusable with long stubs | Phase 2 (scope), Phase 5 |
| Real report jitter | Drives the "no report" timeout and gap detection | Phase 2 |
| USB CDC sustained throughput with the actual PC host | Frame drops look like lost sensors | Phase 3 |
| Flash-cache stalls during SPI | Cause latency spikes | Phase 5 |
| Actual `sizeof()` of every struct | Design targets above are layout estimates | Phase 1: build-time static_assert report |
