# BNO085Multi — modular multi-sensor BNO085 library for ESP32-S3

A ground-up Arduino library for **1–20 BNO085 IMUs on one shared SPI bus**, built for a
professional motion-capture system. The BNO085 performs its own sensor fusion; the
ESP32-S3 only moves bytes, keeps state separate per device, timestamps and forwards.

> **Status: Phase 1 — single-sensor baseline.**
> The driver source exists, compiles for the ESP32-S3 and passes its host tests.
> Hardware validation is **in progress**: stages 1–2 (bus, CS, SHTP framing) are
> confirmed on a physical BNO085, including the continuous-CS read fix that reads the
> 276-byte advertisement coherently. Stage 3 onwards (product ID, Rotation Vector
> configuration, streaming) is **blocked at Product ID**: PS0/WAKE is not wired to a
> GPIO, so the hub ignores host writes while INT is idle. See
> [Validation status](#validation-status) and
> [hardware_test_report.md](hardware_test_report.md) for the evidence.

## Validation status

Kept deliberately granular. "Compiles" is not "works", and simulated is not physical.

| Category | State |
| --- | --- |
| Implemented | HAL, shared SPI bus manager, per-instance SHTP transport, minimal SH-2 (product ID, reset, set/get feature, Rotation Vector), `Sensor` state machine with bounded automatic recovery, CRC-framed binary output with loss accounting, single-sensor example, host test suite |
| Compiled | `esp32:esp32:esp32s3`, ESP32 Arduino core 3.3.12 — 326 806 B flash (24 %), 24 640 B static RAM (7 %) for the example (`--warnings all`, no diagnostics) |
| Host-unit-tested | 61 cases / 438 checks, 0 failures (MSVC 2022, `test/native`); `cl /analyze` 0 warnings |
| Simulated | Mock-HAL models CS assertion/deassertion and transaction boundaries; single device. No 20-instance simulation yet |
| Physically tested | Stages 1–2 with the continuous-CS read fix: ESP32-S3 + Adafruit BNO085 on COM11 (CH343 UART bridge), SPI mode 3 @ 1 MHz. Boot after reset **93.9 ms**; the **276 B channel-0 advertisement** (272-byte payload) is read in **one CS assertion**; `oversize=0`, `badFrag=0`, `seqGaps=0`; 0 bus errors, 0 lock timeouts |
| Blocked | Product ID (`product=0x00000000`, `sh2-timeout`): PS0/WAKE is tied HIGH instead of wired to a GPIO, so writes with INT idle are ignored. SH-2 init and streaming were not reached |

Not yet physically tested: product ID retrieval, Rotation Vector configuration at
100 Hz, report decode, the 10-minute soak, the disconnected-INT timeout, 30 cold boots,
and comparison against the Adafruit library. The old "552 B advertisement drained"
result is invalid: 552 was a one-bit slip of `0x0114` (276) and is superseded by the
continuous-CS read.

## Hardware baseline

| Signal | GPIO |
| --- | --- |
| SCK | 12 |
| MOSI / DI | 11 |
| MISO / DO | 13 |
| CS | 10 |
| INT (H_INTN, active low) | 9 |
| RST (NRST, active low) | 8 |

3.3 V and common ground; **PS0 and PS1 HIGH at reset** to select SPI mode. After reset
PS0 becomes the WAKE input, so it must also be wired to a GPIO for host writes — with it
tied HIGH the hub only accepts a write while it is asserting INT. SPI mode 3, 1 MHz
initial clock (3 MHz is the datasheet ceiling and is opt-in).

## Installing

This repository is the library root, so the folder must be named `BNO085Multi` when it
is placed in the Arduino libraries directory (Arduino derives the library name from the
folder). Then open **File → Examples → BNO085Multi → 01_SingleSensor**.

## Quick start

```cpp
#include <BNO085Multi.h>

bno085::SensorConfig cfg(10, 9, 8);          // CS, INT, RST
bno085::SpiBusManager bus;
bno085::Esp32SpiHal hal(bus, &SPI, managerCfg, cfg);
bno085::Sensor imu(hal);

void setup() {
  Serial.begin(115200);
  managerCfg.sckPin = 12; managerCfg.misoPin = 13; managerCfg.mosiPin = 11;
  if (!imu.begin(managerCfg, cfg).ok()) { /* inspect imu.lastError() */ }
}

void loop() {
  imu.update();                              // non-blocking, bounded
  bno085::Quaternion q;
  if (imu.latest(q)) { /* q.w/x/y/z, q.rawW..rawZ, q.accuracy, q.tMeasUs, q.tHostUs */ }
}
```

`begin()` never blocks on the device: it validates, opens the bus and starts the
initialisation state machine, which `update()` advances by at most a bounded budget per
call. Sensors reach `Streaming` asynchronously.

## Examples and hardware sketches

| Sketch | Purpose |
| --- | --- |
| `examples/01_SingleSensor` | Full Phase 1 acceptance sketch: init, streaming quaternions, `r`/`d`/`q`/`h` serial commands, optional binary output (`BNO085_BINARY_OUTPUT 1`) |
| `test/hardware/01_HeaderProbe` | Diagnostic stage 1–2 sketch (v4): measures boot time, prints raw SHTP headers and reads complete packets inside one CS assertion, without involving SH-2. Use this first when bringing up new wiring |
| `test/hardware/02_TransportDrain` | Regression harness for the continuous-CS read: drives the real `ShtpTransport` against the device and asserts the 276 B advertisement is delivered as one channel-0 packet, one CS assertion per packet, with the stream idle afterwards |

## Tests

Host tests (no hardware, no Arduino):

```
test\native\build_msvc.bat      # Windows / MSVC
sh test/native/build_gcc.sh     # any host with g++
```

They cover CRC vectors, frame encoding, output loss accounting (full sink, zero-byte,
partial and suppressed writes), SHTP header parsing, invalid length/channel,
continuous-CS framing (one assertion per packet, header and payload in the same
transaction, the 276 B advertisement read whole, mid-transfer failure releasing CS),
oversized-transfer drain, fragmentation, sequence handling, Rotation Vector decode, Q14
conversion, timestamp reconstruction and observation-time semantics, timeouts, the init
state machine, bounded automatic recovery (backoff, exhaustion, manual recovery,
transport fault and report silence) and observable output drops. See
[test/native/README.md](test/native/README.md) for what is simulated and what is not.

## Documents

| Document | Contents |
| --- | --- |
| [docs/PHASE1-NOTES.md](docs/PHASE1-NOTES.md) | What Phase 1 implements, deviations from the approved architecture, evidence gathered, and the remaining hardware checklist |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Directory layout, layer diagram, data flow, state ownership, SPI scheduling, reset/recovery, documented compromises |
| [docs/UPSTREAM-AUDIT.md](docs/UPSTREAM-AUDIT.md) | Audit of CEVA SH-2/SHTP and the Adafruit driver: every global/static that blocks multi-instance use, the probable cause of the `begin_SPI()` hang, and the exact patch set if upstream were forked |
| [docs/RESOURCE-ANALYSIS.md](docs/RESOURCE-ANALYSIS.md) | Memory tables for 1/5/10/15/20 sensors, SPI bandwidth and latency calculations, USB output budget, init-time budget |
| [docs/API.md](docs/API.md) | Public API, per-sensor report configuration, binary output protocol (framing, packet types, CRC) |
| [docs/PLAN.md](docs/PLAN.md) | Technical risks with mitigations, phased plan with exit criteria, test strategy, open decisions |

## Design in one paragraph

Four independent layers plus a scheduler: a **HAL** (SPI transactions, CS, INT, reset,
time), a **shared SPI bus manager** (one mutex, one transaction at a time, CS always
released on every exit path), a **per-sensor SHTP transport** (framing, channels,
sequence numbers, bounded reads) and a **per-sensor SH-2 client** (product ID, report
configuration, report decode, hub timestamping). Each `Sensor` owns its SHTP and SH-2
context, so nothing about a device lives in file-scope state. Changing the sensor count
is a runtime parameter, never a source edit.

Timestamps: `tHostUs` is ESP32 `micros()` when the polling loop first observes H_INTN
asserted; it is not the physical edge time (Phase 1 uses polling, not an ISR). `tMeasUs`
is the device's own measurement time, `tMeas = tObservation + (referenceDelta +
delayTicks) × 100 µs`, where
`referenceDelta` starts at 0 for each input payload, is replaced by `−timebase` when a `0xFB`
base reference arrives and accumulates a signed `0xFA` rebase (the SH-2 `sh2.c` convention). The
value is kept modulo 2³² and never merged into the `tHostUs` clock domain. Quaternion components
are reported as `x = i/16384, y = j/16384, z = k/16384, w = real/16384`, with the raw Q14 integers
preserved alongside.

## Phase 1 limitations

- One physical sensor. Multi-sensor scheduling, `Manager`, aggregation and ring buffers
  are Phase 2+; the per-instance design is what makes that a scheduling change rather
  than a rewrite.
- Rotation Vector only. A Game Rotation Vector request is rejected rather than
  silently substituted.
- `Sensor::get()` returns `false`; optional reports are not decoded yet.
- Fragmented payloads are drained and counted, not reassembled. The channel-0
  advertisement on this part is 276 B (272 B payload), which fits the 512 B transfer
  and 320 B payload buffers; larger payloads still take the bounded drain path inside
  the same CS assertion.
- **PS0/WAKE must be wired to a GPIO for SPI writes.** With PS0 tied HIGH the hub only
  accepts a host write while it is asserting INT; after the boot reads are consumed the
  line is idle, so Product ID and feature configuration time out. This currently blocks
  Stage 1.
- Binary output is synchronously offered to the sink during service. A sink that cannot
  accept a whole frame causes that frame to be dropped and counted; there is no output
  queue and no retry. A partial frame can only result from a sink that misreports
  capacity or returns a short count.
- Automatic recovery is bounded per sensor (default 5 attempts with 250–4000 ms
  backoff). Exhaustion enters terminal `Failed`; only `recover()` restarts it.
- No ESP32-side orientation maths of any kind.

Full detail, including why each deviation was taken, is in
[docs/PHASE1-NOTES.md](docs/PHASE1-NOTES.md).

## Not in scope for v1

Firmware update (DFU), I²C/UART transports, FRS/calibration-record access, tare, and
`sh2`-API compatibility. Rationale in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#9-documented-architectural-compromises).
