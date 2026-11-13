# BNO085Multi — modular multi-sensor BNO085 library for ESP32-S3

A ground-up Arduino library for **1–20 BNO085 IMUs on one shared SPI bus**, built for a
professional motion-capture system. The BNO085 performs its own sensor fusion; the
ESP32-S3 only moves bytes, keeps state separate per device, timestamps, aggregates and
forwards.

> **Status: Phase 0 — architecture review.**
> No driver source has been written yet, by design. The documents below are the
> deliverable; implementation starts after review. Nothing here has been validated on
> hardware, and no claim of hardware validation is made anywhere in this repository.

## Documents

| Document | Contents |
| --- | --- |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Directory layout, layer diagram, data flow, state ownership, SPI scheduling, reset/recovery, documented compromises |
| [docs/UPSTREAM-AUDIT.md](docs/UPSTREAM-AUDIT.md) | Audit of CEVA SH-2/SHTP and the Adafruit driver: every global/static that blocks multi-instance use, the probable cause of the `begin_SPI()` hang, and the exact patch set if upstream were forked |
| [docs/RESOURCE-ANALYSIS.md](docs/RESOURCE-ANALYSIS.md) | Memory tables for 1/5/10/15/20 sensors, SPI bandwidth and latency calculations, USB output budget, init-time budget |
| [docs/API.md](docs/API.md) | Proposed public API, per-sensor report configuration, binary output protocol (framing, packet types, CRC), JSON debug mode |
| [docs/PLAN.md](docs/PLAN.md) | Technical risks with mitigations, phased implementation plan with exit criteria, test strategy, open decisions for review |

## Target baseline

- **MCU:** ESP32-S3, Arduino framework (ESP32 Arduino core).
- **IMU:** Adafruit BNO085 breakout, SPI mode 3, 1 MHz bring-up / 3 MHz maximum.
- **Bus:** one shared SPI peripheral, one CS per sensor, one INT per sensor
  (optional), reset shared or per sensor.
- **Link to PC:** USB CDC serial, compact binary framing with CRC-16.

## Design in one paragraph

Four independent layers plus a scheduler: a **HAL** (SPI transactions, CS, INT, reset,
time), a **shared SPI bus manager** (one mutex, one transaction at a time, CS always
released), a **per-sensor SHTP transport** (framing, channels, sequence numbers,
fragment assembly, bounded reads) and a **per-sensor SH-2 client** (product ID, report
configuration, report decode, hub timestamping). A **manager** owns the sensor table
and runs a non-blocking round-robin scheduler over INT-driven service requests; an
**aggregator** stamps every record with sensor ID plus both measurement and reception
time; an **output encoder** writes a versioned binary frame with CRC-16. Changing the
sensor count is a runtime parameter — `begin(configs, n)` — never a source edit.

## Not in scope for v1

Firmware update (DFU), I²C/UART transports, FRS/calibration-record access, tare, and
`sh2`-API compatibility. All are listed with rationale in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#9-documented-architectural-compromises).
