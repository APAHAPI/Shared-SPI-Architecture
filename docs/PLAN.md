# Risks and phased implementation plan

Phase 0 deliverable. This is the document that must be approved before code is
written.

---

## 1. Technical risks

Likelihood and impact are judgements from the datasheet, the upstream audit and the
mechanical design — not from a hardware run.

| # | Risk | Impact | Likelihood | Mitigation | Retired at |
| --- | --- | --- | --- | --- | --- |
| R1 | SPI framing error (header read twice, length includes header, bit 15 continuation) | No data at all; looks like dead hardware | Medium | Single-sensor baseline first, byte-compared against an Adafruit sketch trace on a logic analyser | Phase 1 |
| R2 | Unbounded waits re-enter the design (upstream pattern) | Sketch hangs with no diagnostics | Medium | Normative rule: every loop has a deadline and a counter; code review checklist; test that a disconnected sensor fails within its budget | Phase 1, 4 |
| R3 | Service latency exceeds the BNO085's ~10 ms host-response window | Duplicate reports, jitter, apparent packet loss | Low at ≤200 Hz/20 sensors (3.4 ms worst case computed) | Budgets in §6.5 of the architecture; `maxUpdateUs` in diagnostics; duplicate detection via sequence + timestamp | Phase 2, 5 |
| R4 | A foreign SPI device (SD, display) interleaves a transaction | Corrupted CS selection, lost packets | Medium | Only `SpiBusManager::withBus()` may touch the bus; document the rule; diagnose with `busBusy`/`spiErrors` | Phase 2 |
| R5 | Harness signal integrity at 3 MHz with 20 stubs | Intermittent CRC/frame errors that look like firmware bugs | Medium-High | Default 1 MHz; 3 MHz opt-in; 33 Ω series resistors at the MCU, short stubs, one ground return per device; scope check at Phase 2 | Phase 2/5 |
| R6 | ESP32-S3 pin choice collides with strapping/USB/flash pins | Boot failures, unusable USB, no INT response | Medium | Validated pin list, runtime warning for reserved pins (0/3/19/20/45/46 and flash/PSRAM pins) | Phase 1 |
| R7 | Power/ground: 20 breakouts on one 3V3 rail | Brownouts, resets, corrupted init | Medium | Separate rail sized for worst-case current, 10 µF + 100 nF per board, star ground at the regulator | Phase 2/5 |
| R8 | Timestamp reconstruction wrong (rebase / 12 ms reference / delay exponent) | Motion-capture data temporally skewed and unusable | Medium | Implement exactly the datasheet model; export raw status+delay bytes in diagnostics; validate against a known rotation rate; compare with Adafruit's `timestamp` | Phase 1 |
| R9 | Per-sensor clock drift treated as if synchronised | Silent fusion errors on the PC | Medium | Two timestamps exported, no merging anywhere; documented in architecture §9.5 | Phase 3 |
| R10 | Recovery storm (all sensors fault together, all retry at once) | Bus saturation exactly when the system is already unhealthy | Medium | Per-sensor backoff with an index-derived phase offset; one reset group at a time | Phase 4 |
| R11 | Q-format assumption wrong for an optional report | Wrong physical units in `IMU_SAMPLE` | Low-Medium | `scaleExponent` in every frame; per-report unit tests against known vectors; device metadata read once in Phase 5 | Phase 3 |
| R12 | Arduino-ESP32 core version differences (`SPI`, `attachInterrupt`, USB CDC) | Build breaks or timing changes | Medium | Use only long-stable APIs; note tested core version; compile matrix in CI; no `analogWrite`-style tricks | Phase 1 |
| R13 | Malformed/oversized packet wedges a sensor | One sensor dead, others fine (by design) | Medium | Reject oversize, count, drain until INT clears, then optionally recover; never trust a length before validating it | Phase 4 |
| R14 | Flash-cache / interrupt stalls at 3 MHz | Rare latency spikes → duplicates | Low-Medium | Measure `maxServiceUs`; keep transactions ≤ 256 B; avoid flash writes in the streaming path | Phase 5 |
| R15 | Firmware/part-number variance across BNO085 units | Some features silently unavailable (e.g. Rotation Vector) | Medium | Log `swPartNumber` + version in `INIT_RESULT`; gate capabilities; GRV only with explicit opt-in | Phase 1, 5 |
| R16 | Shared NRST line resets the whole rig during recovery | Full-stream interruption for one bad sensor | Depends on wiring | Detect shared pins, treat as a group, recommend independent resets | Phase 4 |

---

## 2. Phases

Each phase has an exit criterion that can be demonstrated, not just asserted.
"Simulated" and "physical" results are reported separately and never mixed.

### Phase 0 — Architecture (this deliverable)

- **Output:** this document set.
- **Exit criterion:** review approval of §6 open decisions.

### Phase 1 — Single-sensor baseline (physical)

Scope: `SpiBusManager`, `Esp32SpiHal`, `ShtpTransport`, minimal `Sh2Client`, RV
configuration and decode, binary output for one sensor.

Tasks:

1. Confirm R1/R2 hypotheses with an instrumented Adafruit two-object sketch (audit §5).
2. Bus manager + HAL; CS guard; bounded transactions.
3. SHTP open: read advertisement, validate channels, handle reset-complete.
4. SH-2: product ID, set feature (RV at 100 Hz), get feature verification.
5. Report decode, timestamp reconstruction, `Quaternion` population.
6. Binary frame + CRC; PC parser.
7. Measure: `sizeof()` of every struct, per-report service time, init time.

Exit criteria (all on hardware):

- 10 minutes continuous at 100 Hz with zero frame errors and zero sequence gaps.
- Quaternion matches an Adafruit sketch on the same wiring to within Q14 rounding.
- 30 consecutive cold boots succeed.
- A sensor with its INT line disconnected fails with a diagnostic within 1 s and does
  not hang the sketch.
- Measured service time and init time recorded, replacing the estimates in
  RESOURCE-ANALYSIS.md.

### Phase 2 — Multi-instance (2 physical sensors)

Tasks: second instance, independent contexts, scheduler round-robin, per-sensor
diagnostics, independent recovery.

Exit criteria:

- Rotating sensor 0 changes only sensor 0's quaternion stream; verified visually and
  by PC-side correlation.
- Logic-analyser trace shows correct CS per transaction and no overlapping asserts.
- Both sensors at 100 Hz for 30 minutes; no sequence gaps attributable to the host.
- Unplugging sensor 1 leaves sensor 0 streaming; reconnecting recovers sensor 1 only.
- Bus/SPI error counters remain zero during steady state.

### Phase 3 — Aggregation and PC protocol

Tasks: per-sensor lanes, gap/duplicate detection, status and diagnostic frames, host
parser finishing, JSON debug mode.

Exit criteria:

- PC reconstructs all streams with correct sensor IDs; no record appears under two
  IDs.
- Deliberate packet loss (host-side pause) shows up as gaps in `SENSOR_STATUS`, not as
  wrong data.
- CRC check rejects a corrupted frame and resynchronises on the next `A5 5A`.
- Both timestamps present, monotonic per sensor (modulo wrap), and `tMeasUs` advances
  at the configured rate.

### Phase 4 — Recovery and diagnostics (physical, 2 sensors + fault injection)

Fault cases: missing sensor at boot, sensor unresponsive mid-run, reset during
operation, INT stuck low, corrupted/invalid SHTP packets, 3.3 V brownout of one board.

Exit criteria:

- Every fault produces a bounded, visible outcome (`SENSOR_STATUS` / `ERROR`) within
  the configured timeout.
- One sensor's recovery never interrupts the other (independent reset wiring).
- Shared-reset wiring is exercised deliberately and its impact documented.
- No fault produces an indefinite stall in `update()`; `maxUpdateUs` stays bounded.

### Phase 5 — Scalability

Two tracks, reported separately:

**Simulated (mock HAL):** 20 configured instances, deterministic report injection at
100/200/400 Hz. Measures: per-`update()` duration, queue depth, CPU headroom, memory
from the linker map, starvation behaviour with one sensor stuck.

**Physical:** 5, 10, 15, 20 real sensors if the hardware is available. Measures:
actual bus utilisation, error counters, thermal/power behaviour, harness behaviour at
1 MHz and 3 MHz.

Exit criteria:

- 20 configured instances at 100 Hz for 1 hour: zero unbounded stalls, zero crashes,
  queue drops zero or explained.
- Report separating what was measured on hardware from what was simulated.
- Bandwidth table in RESOURCE-ANALYSIS.md updated with measured overhead; any
  unsupported rate documented rather than promised.

---

## 3. Test strategy

| Layer | How tested | Where |
| --- | --- | --- |
| CRC-16, ring buffer, time wrap | Host unit tests (g++, no Arduino) | `test/native/` |
| SHTP framing, continuation, sequence, malformed packets | Host unit tests with a scripted byte stream | `test/native/` |
| SH-2 report decode (RV, GRV, raw/calibrated) | Host unit tests with captured payloads; known-vector assertions | `test/native/` |
| Scheduler fairness/starvation | Host unit tests with a mock clock and 20 fake devices | `test/native/` |
| Bus arbitration, CS guard | Mock HAL records the CS sequence; assert one-at-a-time and always-released | `test/native/` |
| Real device behaviour | Hardware sketches + logic analyser + PC parser | `test/hardware/` |
| Timing/latency | `micros()` instrumentation in diagnostics, PC-side statistics | Phase 1/2/5 |

The pure layers (SHTP, decode, CRC, queue, scheduler) are written with no Arduino
dependency so they can be compiled and tested on a PC. That is a design constraint,
not an afterthought: it is how framing bugs get caught without a scope.

---

## 4. Validation status

- **Compiled:** nothing.
- **Unit tested:** nothing.
- **Simulated:** nothing.
- **Hardware validated:** nothing.

The architecture phase produces documents only. Every performance number in
RESOURCE-ANALYSIS.md is a calculation with stated assumptions.

---

## 5. What will be built first

Order matters: the first code must be the smallest thing that can be wrong in a
detectable way.

1. `util/Crc16` + `output/OutputProtocol` + host test → proves framing on a PC.
2. `shtp/ShtpTransport` + mock HAL host test → proves parsing before any hardware.
3. `hal/SpiBusManager` + `Esp32SpiHal` + a "read 4 bytes and print them" sketch →
   proves the bus and CS handling on the real board.
4. Minimal `Sh2Client` → product ID, then RV.
5. Then — and only then — the sensor object, manager and scheduler.

---

## 6. Open decisions for review

| # | Decision | Recommendation | Why it needs sign-off |
| --- | --- | --- | --- |
| D1 | **SH-2 control layer:** own minimal implementation vs fork CEVA `sh2.c` | Own (audit §7). Fork patch set is documented if preferred. | Determines whether the library can ever expose tare/FRS/DFU, and whether future CEVA changes must be merged. |
| D2 | **Default SPI clock:** 1 MHz vs 3 MHz | 1 MHz default, 3 MHz opt-in | Affects bring-up reliability vs maximum rate; harness quality decides. |
| D3 | **Reset wiring:** independent per board vs shared | Independent for a 20-sensor rig; shared supported and documented | Changes recovery isolation and pin count. |
| D4 | **Wire format:** fixed-point only vs float quaternions | Fixed-point only (PC converts) | Affects frame size, ESP32 load and PC tooling. |
| D5 | **Storage:** single heap allocation at `begin()` vs static pool | Heap default, static option | Some projects ban dynamic allocation. |
| D6 | **INT wiring:** one GPIO per sensor vs polling some subset | One per sensor where pins allow; polling supported | Directly affects latency and bus budget. |
| D7 | **Sensor count ceiling for physical validation** | 2 now, up to 20 if hardware exists | Determines what can honestly be reported as hardware-validated in Phase 5. |

Answers to D1–D4 unblock Phase 1. D5–D7 can be settled during Phase 1–2.

---

## 7. Deliverable checklist (from the brief)

| # | Deliverable | Phase | Status |
| --- | --- | --- | --- |
| 1 | Organized source/header files | 1–3 | Planned (§2 structure) |
| 2 | Public API documentation | 1 | Proposal in API.md |
| 3 | HAL implementation | 1 | Planned |
| 4 | SPI bus manager | 1 | Planned |
| 5 | SHTP transport | 1 | Planned |
| 6 | SH-2 integration | 1 | Planned |
| 7 | Sensor manager | 2 | Planned |
| 8 | Data aggregation | 3 | Planned |
| 9 | Binary output protocol | 3 | Specified in API.md §4 |
| 10 | Diagnostics and recovery | 2–4 | Planned |
| 11 | Single-sensor example | 1 | Planned |
| 12 | Dual-sensor example | 2 | Planned |
| 13 | Configurable multi-sensor example | 5 | Planned |
| 14 | Unit tests where practical | 1+ | Planned (§3) |
| 15 | Hardware testing instructions | 1+ | Per phase, `test/hardware/` |
| 16 | Architecture documentation | 0 | **This package** |
| 17 | Performance and memory analysis | 0 | RESOURCE-ANALYSIS.md (estimated; measured in 1/5) |
