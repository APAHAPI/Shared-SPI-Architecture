# BNO085Multi — Architecture

Phase 0 deliverable. Documentation only; no code exists yet.

---

## 1. Design rules

These rules are normative for the implementation. Each one exists because a specific
class of bug is otherwise easy to reintroduce.

1. **No mutable file-scope state.** Every layer is an object; every entry point takes
   `this`. The only globals allowed are `constexpr` limits and `const` tables.
2. **One writer per state item.** A field is written by exactly one layer and read by
   the layers above it (table in §5).
3. **Every wait has a deadline.** No `while (!ready)` without an elapsed-time test and a
   failure exit. Every loop increments a counter that is visible in diagnostics.
4. **No SPI inside an interrupt handler.** The ISR does two things: capture `micros()`
   and set a bit in an atomic mask.
5. **CS is deasserted on every exit path**, including early return and error, via an
   RAII guard around the transaction.
6. **The bus is never held across a USB write**, a `delay()`, or a callback into
   application code.
7. **A sensor failure never escalates to a bus or system reset** unless the hardware
   wires resets together (documented in §7.5).
8. **No sensor fusion on the ESP32.** Quaternions are copied and converted, never
   computed.

---

## 2. Proposed directory structure

Repository root is the library root. When installing into Arduino, the folder must be
named `BNO085Multi` (Arduino derives the library name from the folder).

```
BNO085Multi/
├── library.properties
├── keywords.txt
├── LICENSE                      Apache-2.0 (CEVA-derived files keep their header)
├── NOTICE                       attribution for vendored CEVA components
├── README.md
├── src/
│   ├── BNO085Multi.h            umbrella public header (the only header users include)
│   ├── BNO085Config.h           compile-time limits and defaults (constexpr only)
│   ├── BNO085Types.h            public POD types: Quaternion, Record, Status, Error
│   ├── BNO085Manager.h/.cpp     sensor table, scheduler, recovery orchestration
│   ├── BNO085Sensor.h/.cpp      per-device state machine and owned sub-objects
│   ├── hal/
│   │   ├── IHal.h               abstract SPI/CS/INT/reset/time interface
│   │   ├── SpiBusManager.h/.cpp shared bus: mutex, transaction, CS guard
│   │   └── Esp32SpiHal.h/.cpp   ESP32-S3 implementation (Arduino SPI)
│   ├── shtp/
│   │   ├── ShtpProtocol.h       header layout, channel enum, limits
│   │   ├── ShtpTransport.h/.cpp per-sensor framing, sequencing, assembly
│   ├── sh2/
│   │   ├── Sh2Codes.h           report IDs, command IDs, response lengths
│   │   ├── Sh2Client.h/.cpp     product ID, feature config, async op pump
│   │   ├── Sh2Reports.h/.cpp    fixed-point report decode + accuracy/timestamp
│   │   └── third_party/ceva/    optional vendored SH-2 sensor-value decoder
│   ├── aggregate/
│   │   ├── ReportQueue.h        fixed-capacity ring buffer (no allocation)
│   │   └── ReportAggregator.h/.cpp
│   ├── output/
│   │   ├── OutputProtocol.h     frame layout, packet type IDs
│   │   ├── BinaryEncoder.h/.cpp CRC-16 framing, little-endian packing
│   │   ├── JsonDebugEncoder.h/.cpp  optional, debug only
│   │   └── IOutputSink.h        Serial / USB CDC / user-provided
│   ├── diag/
│   │   ├── Diagnostics.h        per-sensor counters and state enum
│   │   └── RecoveryPolicy.h     attempt limits, backoff table
│   └── util/
│       ├── Crc16.h/.cpp
│       ├── RingBuffer.h
│       └── Time.h               32-bit micros arithmetic, wrap-safe
├── examples/
│   ├── 01_SingleSensor/01_SingleSensor.ino
│   ├── 02_DualSensor/02_DualSensor.ino
│   └── 03_MultiSensorConfigurable/03_MultiSensorConfigurable.ino
├── extras/host/bno085_stream.py  reference PC-side frame parser (optional tool)
├── test/
│   ├── native/                  host-compiled tests of the pure layers
│   └── hardware/                hardware procedures + diagnostic sketches
└── docs/                        this package
```

Rationale for `src/` recursive layout: Arduino 1.5+ library format compiles `src/`
recursively, so sub-folders cost nothing at build time and keep the four layers from
turning into a flat pile of files.

---

## 3. Layer diagram

```
                    ┌──────────────────────────────────────────────┐
   application ────►│  BNO085Manager        update() / callbacks   │
                    │  · sensor table (1..20)                      │
                    │  · round-robin scheduler                     │
                    │  · init + recovery orchestration             │
                    └───────┬───────────────────────┬──────────────┘
                            │ per-sensor            │ records
                    ┌───────▼──────────┐   ┌────────▼──────────────┐
                    │  BNO085Sensor     │   │  ReportAggregator     │
                    │  · state machine  │   │  · per-sensor lanes   │
                    │  · latest values  │   │  · seq gap detection  │
                    │  · diagnostics    │   └────────┬──────────────┘
                    └───┬──────────┬───┘            │
                        │          │                │
             ┌──────────▼───┐  ┌───▼──────────┐  ┌──▼───────────────┐
             │  ShtpTransport│  │  Sh2Client   │  │  BinaryEncoder   │
             │  · framing    │◄─┤  · prod id   │  │  · frame + CRC   │
             │  · channels   │  │  · cfg ops   │  └──┬───────────────┘
             │  · assembly   │  │  · decode    │     │
             └──────┬────────┘  └──────────────┘  ┌──▼───────────────┐
                    │                            │  IOutputSink     │
             ┌──────▼──────────────────────┐     │  (USB CDC)       │
             │  IHal  (per sensor)         │     └──────────────────┘
             │  read / write / int / reset │
             │  timeUs                     │
             └──────┬──────────────────────┘
                    │  transactions
             ┌──────▼──────────────────────┐
             │  SpiBusManager (one/bus)    │
             │  mutex · SPI settings · CS  │
             └──────┬──────────────────────┘
                    │
                [ SPI bus ]──► BNO085 #0 … #19  (one CS each)
```

Dependencies point downward only. `ReportAggregator` and `BinaryEncoder` have **no**
knowledge of SPI, SHTP or SH-2 — they consume plain `Record` structs. That is what
allows a different IMU driver to reuse the PC protocol unchanged.

---

## 4. Data flow

### 4.1 Cold start (per sensor, advanced by `update()`)

```
begin(configs, n)
  ├─ validate: n ∈ [1,20]; CS pins distinct; no pin reused for CS/INT/reset;
  │            CS ≠ INT; report intervals within limits
  ├─ allocate one contiguous sensor block (single allocation, §8.4)
  └─ schedule reset phase for all sensors in parallel

per update(), round-robin over sensors in state ≠ Streaming:
  Reset        : drive NRST low ≥ 1 µs (datasheet min 10 ns), release
  BootWait     : poll INT / elapsed; sensor ready when INT asserts or 300 ms budget
  Advertise    : read the SHTP advertisement on channel 0 (channel map)
  Identify     : send 0xF9 (product ID request) → collect 0xF8 responses (≤ 5)
  IdentifyChk  : record swPartNumber / fw version; gate on known-good part number
  Configure    : for each enabled report: 0xFD set feature (reportInterval_us)
  Verify       : 0xFE get feature → store the *actual* interval granted
  Streaming
```

Each step is one bounded action. The device's own boot is concurrent (datasheet
§6.5.3: 90 ms init + 4 ms config, communicate after H_INTN asserts), so 20 sensors
reach `Streaming` in roughly one boot period plus N × a few configuration exchanges
(§7.2, and [RESOURCE-ANALYSIS.md](RESOURCE-ANALYSIS.md#6-initialization-time-budget)).

### 4.2 Steady state (one report, byte level)

```
H_INTN ↓  ─► ISR: mask |= (1<<i); tEdge[i] = micros()
update()  ─► pick next pending sensor (round robin)
             acquire bus (bounded)
             CS↓
             SPI read 4 bytes → len = header[0] | header[1]<<8
             if len == 0 → release, treat as spurious INT
             CS↑   (device deasserts INT ~800 ns after CS is observed)
             CS↓
             SPI read len bytes (header repeated + payload)
             CS↑
             release bus
             SHTP: validate len, channel, sequence; assemble fragments
             SH-2 : iterate reports inside payload (a payload may carry several)
             decode: RV at report[4..13] → i/j/k/real Q14, accuracy Q12
             timestamp: tMeas = T_int − referenceDelta + delay·100 µs   (datasheet §1.3.5)
             store latest + enqueue 32-byte Record
```

Why two reads: the datasheet (§1.3.1) documents exactly this — read the first four
bytes to learn how many clocks are needed, then repeat the read for the full transfer.
The header is therefore transmitted twice, which costs 4 wire bytes per transfer
(accounted for in the bandwidth numbers).

### 4.3 Output path

```
Records ─► aggregator lanes ─► drain (outside the bus lock) ─► BinaryEncoder
        ─► frame: A5 5A | ver | type | len | seq | payload | CRC16 ─► USB CDC
```

Draining happens only when the bus is released, so a slow or blocking USB write cannot
delay an interrupt service (the BNO085 deasserts INT and retries if the host takes
longer than ~10 ms — datasheet §1.2.4.1 — so bus latency is a correctness property,
not just performance).

### 4.4 Where CPU time goes

Per report the ESP32-S3 does: 2 SPI transactions, one header parse, one report decode
(integer only), one timestamp reconstruction, one ring-buffer push, and 34 bytes of
framing. There is no floating-point work in the hot path unless the application asks
for float output (see §9.3).

---

## 5. State ownership analysis

### 5.1 Ownership table

| State | Owner | Cardinality | Written by |
| --- | --- | --- | --- |
| SPI peripheral, clock, mode | `SpiBusManager` | 1 per bus | `begin()` |
| Bus mutex | `SpiBusManager` | 1 per bus | runtime |
| CS / INT / reset pin numbers | `SensorConfig` | per sensor, immutable | `begin()` |
| SHTP `nextOutSeq` / `nextInSeq` per channel | `ShtpTransport` | per sensor | SHTP TX/RX |
| SHTP reassembly cursor, channel, remaining | `ShtpTransport` | per sensor | SHTP RX |
| SHTP RX/TX buffers | `ShtpTransport` | per sensor | SHTP |
| Pending SH-2 request, expected response | `Sh2Client` | per sensor | SH-2 |
| Product IDs, granted feature intervals | `Sh2Client` | per sensor | SH-2 |
| Hub timestamp rollover / rebase state | `Sh2Client` | per sensor | SH-2 |
| Latest quaternion + accuracy + timestamps | `BNO085Sensor` | per sensor | report path |
| Init/recovery state, attempt counter | `BNO085Sensor` | per sensor | scheduler |
| Per-sensor diagnostics counters | `BNO085Sensor` | per sensor | all layers via accessor |
| Aggregated records | `ReportAggregator` | 1 (N lanes) | report path |
| Output frame sequence | `BinaryEncoder` | 1 | drain |

### 5.2 Rules that fall out of the table

- **Nothing about a device is stored outside its `BNO085Sensor`.** The manager holds
  pointers/indices only.
- **Sequence numbers are per sensor and per channel.** Two devices both starting at
  sequence 0 must not be able to look like one device with a duplicate sequence.
- **Callbacks carry the sensor index.** No callback may infer identity from an
  implicit "current sensor" variable.
- **`const` tables may be shared**: report-length tables, CRC table, scale constants.
  These are read-only and identical for every device.

### 5.3 Why each sensor owns a full SHTP and SH-2 context

SHTP is a *point-to-point* protocol. Sequence numbers, fragment assembly, and channel
subscriptions are properties of one link. Two links sharing one context produce
exactly the failure modes this project must avoid: sequence mismatch on every packet,
fragments from sensor A completing a payload started by sensor B, and a callback that
cannot tell which device produced the report.

The same applies to SH-2: its pending-operation slot, command sequence counter and
timestamp extrapolation are per device. A shared pending-request slot means a
`getFeature` response from device 7 can complete a request issued to device 3.

### 5.4 Sharing that is allowed

| Shared thing | Why it is safe |
| --- | --- |
| SPI peripheral | Serialised by one mutex; each transaction carries its own CS. |
| Clock/mode settings | Identical for all BNO085s (mode 3); applied per transaction. |
| `const` report-length table | Read-only. |
| CRC table / scale constants | Read-only. |
| Output sink | Single writer (the drain), sequential frames. |

---

## 6. SPI bus management and scheduling

### 6.1 Bus invariants

1. At most one CS is low at any moment.
2. CS is low only between `acquire()` and `release()` of the bus lock.
3. Every transaction uses `SPI_MODE3`, MSB-first, and the configured clock
   (default 1 MHz for bring-up, max 3 MHz per datasheet §6.5.2).
4. CS is deasserted even on early return, via an RAII guard.
5. All CS pins are driven HIGH and the bus is initialised **before** any device is
   reset, so no device sees a floating select during boot.
6. A transaction that fails leaves the bus unlocked and CS high; the failure is
   counted, never swallowed.

### 6.2 Why the bus lives below the HAL

`SpiBusManager` owns the peripheral and the lock. Each `IHal` instance holds a
reference to the bus plus its own CS pin, so a HAL cannot accidentally touch the bus
without acquiring it. Sensors therefore never know the bus exists — they see
`read()/write()/intAsserted()/reset()/timeUs()`.

Foreign devices (an SD card, a display) can share the bus through the same manager
(`bus.withBus(fn)`), which is the only supported way to avoid a foreign driver
interleaving a transaction with a sensor read.

### 6.3 Locking

- FreeRTOS recursive-free binary mutex, created in `begin()`.
- `acquire(timeoutUs)` is **bounded** (default 2 ms). On timeout the sensor is skipped
  this pass and `diag.busBusy++` is incremented; no spin, no priority inversion.
- `SPI.beginTransaction()` / `endTransaction()` are nested inside the mutex, never
  outside, so the ESP32 driver's own arbitration cannot reorder transactions.
- The lock is never held while calling application callbacks or writing to USB.

### 6.4 Interrupt path

```
attachInterrupt(digitalPinToInterrupt(intPin), isr, FALLING)
ISR: pendingMask |= bit; edgeUs[i] = micros();
```

- The mask is a single `uint32_t` accessed atomically (20 sensors < 32 bits).
- `micros()` in an ISR is safe on ESP32 and gives the reception timestamp the
  datasheet's timing model expects (host timestamps at H_INTN assertion).
- **Level re-check:** because H_INTN is level-asserted until CS is observed, each
  `update()` also samples `digitalRead(intPin) == LOW` for sensors in `Streaming`.
  This recovers from a missed edge without any ISR work.
- No-INT configuration (`intPin == -1`) is supported: the sensor is polled on a
  schedule derived from its report interval.

### 6.5 Round-robin service loop

```
update():
  now = micros()
  mask = pendingMask | levelPending()        // snapshot + clear atomically
  while (mask && (micros() - now) < serviceBudgetUs):
      i = nextSetBit(mask, cursor)           // cursor advances unconditionally
      mask &= ~(1<<i)
      sensor[i].service(maxTransfersPerVisit) // ≤ 3 transfers, each bounded
      cursor = i + 1
  drainOutput()                              // outside the bus lock
  advanceRecoveryTimers()
```

Properties:

- **Fairness:** the cursor advances even when a sensor fails or is skipped, so a
  permanently asserted INT cannot monopolise the loop.
- **Starvation:** a global per-`update()` budget (default 2 ms) returns control to the
  sketch; leftover bits stay set for the next call.
- **Stuck INT:** a sensor that still asserts INT after `maxTransfersPerVisit` keeps its
  bit set but is moved to the back of the queue; `diag.intStuck++` after 3 consecutive
  visits so a genuinely wedged device is visible.
- **Continuous streams:** a sensor producing data faster than it can be serviced will
  accumulate in the device's own buffer; the hub then sends larger, multi-report
  payloads, which amortise the 4-byte header cost. The scheduler therefore prefers
  draining up to 3 transfers per visit rather than exactly one.

### 6.6 Bounded work per step

| Operation | Bound | On expiry |
| --- | --- | --- |
| Bus acquire | 2 ms | skip, `busBusy++` |
| One SPI transaction | fixed by clock rate and byte count (no infinite wait possible) | — |
| SHTP read | ≤ `BNO085_RX_TRANSFER_BYTES` (256) | reject oversize, count, drain |
| SH-2 request | 100 ms per command | `SH2_TIMEOUT`, sensor → `Degraded` |
| Boot wait | 300 ms | recovery attempt |
| Reset pulse | 1 ms | continue, count |

---

## 7. Initialization, reset and recovery

### 7.1 Reset topologies

| Topology | Config | Behaviour |
| --- | --- | --- |
| Independent (default) | `resetPin` per sensor, all distinct | `recover(i)` touches only sensor `i` |
| Shared | one `ResetGroup` ID, or the same pin listed for several sensors | `recover(i)` resets the whole group and re-initialises every member |
| None | `resetPin == -1` | software reset only: send SHTP executable `RESET` (0x01) on channel 1, then re-init |

The manager detects a shared pin at `begin()` and records the group. It never assumes
a shared reset when the pins differ.

### 7.2 Boot sequence timing

Datasheet §6.5.3: NRST low ≥ 10 ns, then 90 ms internal init + 4 ms configuration;
the host may communicate once H_INTN asserts (§5.2.1). The library waits for INT with
a 300 ms budget (≈3× the datasheet figure) instead of a fixed `delay()`, so a fast
device is not penalised and a dead device fails deterministically.

After reset the device sends, in order: SHTP advertisement (channel 0), reset-complete
on the executable channel (1), and an unsolicited initialisation message on the SH-2
control channel (2). The state machine consumes all three and only then configures
reports.

### 7.3 Failure detection

A sensor is marked `Degraded` when any of these holds:

- no report within `2.5 × interval + 50 ms` while `Streaming`;
- 3 consecutive SPI transactions returning no valid header;
- SHTP malformed-packet counter increases (bad channel, bad fragment, oversize,
  sequence gap beyond threshold);
- product ID query fails or returns an unrecognised part number at init;
- the device signals a reset (reset-complete) — reconfigure that device only.

`Degraded` still streams if data arrives; it exists to make recovery decisions
visible rather than silently retrying.

### 7.4 Recovery state machine (per sensor)

```
        ┌───────────── Failed ◄── attempts exhausted (default 5)
        │                 ▲
Streaming ──fault──► Recovering ──ok──► Reconfiguring ──ok──► Streaming
                        │  backoff: 250 ms, 500 ms, 1 s, 2 s, 4 s
                        └─ phase offset = (index × 37 ms)  ← avoids recovery storms
```

- `recover(i)` is public and callable at any time; it resets the attempt counter.
- Recovery is independent: no other sensor's state is touched unless resets are
  physically shared (§7.1).
- Every attempt emits a `SENSOR_STATUS` and, on final failure, an `ERROR` frame, so the
  PC sees the failure instead of a silent dropout.

### 7.5 Shared-reset caveat (documented compromise)

If hardware ties NRST of several boards together, recovering one sensor necessarily
interrupts the others. The library cannot avoid this; it minimises the cost by
(a) treating the group as a unit, (b) resetting at most one group at a time, and
(c) re-initialising group members concurrently rather than serially. The recommended
wiring for a 20-sensor rig is one reset GPIO per board (ESP32-S3 has enough pins) or a
shared reset plus an independent software-reset path.

---

## 8. Reported output, aggregation and protocol boundaries

Detailed frame layout is in [API.md](API.md#4-binary-output-protocol). The
architectural points:

1. **Every record is immutable once enqueued** and carries `sensorId`; the aggregator
   never merges fields from two devices.
2. **Two timestamps per record.** `tMeasUs` is the device's own measurement time
   (derived from the host interrupt timestamp and the report's delay field, per
   datasheet §1.3.5); `tHostUs` is ESP32 `micros()` at INT assertion. The two are
   different clock domains and are never averaged or substituted.
3. **No assumption of synchronism.** Records are emitted as they arrive; there is no
   "frame" that waits for all 20 sensors.
4. **Gaps are counted, not hidden.** Per report ID, an 8-bit `reportSeq` gap increments
   `missedReports` and is reported in the status packet.
5. **The output layer knows nothing about BNO085.** It consumes `Record`.

---

## 9. Documented architectural compromises

| # | Compromise | Why | Cost / mitigation |
| --- | --- | --- | --- |
| 9.1 | **Own SH-2 control layer instead of vendoring `sh2.c`** | Upstream is a singleton (`sh2_t _sh2`), uses file-static timestamp history, and its `opProcess()` loops forever when an operation defines no timeout — the exact pattern that makes a missing sensor look like a firmware hang. Making it multi-instance needs a fork across every API signature. | We implement product-ID, set/get-feature and report decode only (the subset used). Consequence: no FRS, tare, calibration or DFU in v1. Mitigation: the stateless CEVA decoder `sh2_SensorValue.c` is vendored unchanged as an optional typed-decode path, and the audit lists the full fork patch set if that route is preferred later. |
| 9.2 | **SH-2 configuration is asynchronous in our layer, not call-and-wait as upstream** | Required for "one sensor must not block the others". | One request in flight per sensor, matched by report ID + command sequence, 100 ms deadline. Requests are serialised per sensor, which is all SH-2 permits anyway. |
| 9.3 | **Fixed-point quaternions on the wire; floats are the PC's job** | Removes float work from the ESP32 hot path and preserves the device's exact values (no double rounding). | PC multiplies by 1/16384. The library exposes a float accessor for convenience, but the binary stream stays integer. |
| 9.4 | **One runtime allocation at `begin()`** | Sensor count is a runtime parameter, so the array cannot be a fixed array unless the user picks a compile-time maximum. | A single `new Sensor[n]` (n ≤ 20), sized before use, no per-report allocation, no growth. A `BNO085_STATIC_STORAGE` build option places the pool in `.bss` for users who forbid heap. |
| 9.5 | **No cross-sensor clock synchronisation attempted** | Each BNO085 has an independent clock domain; the ESP32 cannot measure their offset without extra stimuli, and fusing clocks is not the ESP32's job. | Both timestamps are exported per record; the PC can align streams and estimate drift. Stated explicitly so nobody assumes hardware-synchronous sampling. |
| 9.6 | **Default 1 MHz SPI, 3 MHz maximum** | Adafruit's working reference runs at 1 MHz; 3 MHz is the datasheet limit and depends on harness quality with 20 stubs. | Clock is configurable per bus. Bandwidth table shows what each clock buys. |
| 9.7 | **INT-driven with polling fallback** | Some harnesses cannot spare 20 GPIOs, and a missed edge must not stall a sensor. | Polling costs bus time; the bandwidth analysis accounts for it. |
| 9.8 | **No DFU / bootloader support in v1** | It requires different SPI timing (mode 0, ≤1 MHz, 28 µs inter-byte delays) and is unrelated to mocap streaming. | Firmware updates stay with the vendor tool for now. |
| 9.9 | **Reset recovery may reset a whole group** | Hardware decision, not a software one. | §7.5. |
| 9.10 | **Performance numbers are calculated, not measured** | No hardware run has been performed in this phase. | Every figure in RESOURCE-ANALYSIS.md is labelled as an estimate with its assumptions, and Phase 1/5 replace them with measurements. |

---

## 10. Requirement traceability

| Requirement | Where satisfied |
| --- | --- |
| 1–20 sensors, no source edits | `begin(configs, n)`; `BNO085_MAX_SENSORS = 20` limit only |
| HAL isolated, no hardcoded pins | `IHal` + `Esp32SpiHal`; pins come from `SensorConfig` |
| Shared bus, independent CS, one at a time | `SpiBusManager` (§6.1) |
| SHTP framing/sequencing/fragments per sensor | `ShtpTransport` (§5.3) |
| SH-2 per sensor, no globals | `Sh2Client` (§5, audit §4) |
| Quaternion + accuracy + both timestamps + sensor ID + seq | `Record` (§8, API §3.1) |
| Optional reports, per-sensor, off by default | `SensorConfig::enable()` (API §2.3) |
| Aggregation independent of transport | `ReportAggregator` (§8) |
| Binary protocol with version/type/len/seq/CRC | API §4 |
| Non-blocking, INT-driven, starvation-free | §6.4–6.5 |
| Bounded init/recovery, per-sensor isolation | §6.6, §7.4 |
| Explicit validation status | README, PLAN §4 |
