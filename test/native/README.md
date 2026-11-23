# Host tests

These tests run the portable core on a PC with no Arduino and no hardware.
They are the only automated evidence for the protocol layers; they do not
replace a test against a real BNO085.

## What is simulated

`mock_hal.h` implements `bno085::IHal` and emulates the SPI side of the device as a
CS-driven bus, not a byte queue:

- `queuePacket()` enqueues one complete SHTP transfer (header + payload) as one
  device presentation;
- `beginRead()` asserts CS and opens a transaction; `read()` clocks bytes only while
  CS is low; `endRead()` releases CS. One queued transfer is presented per CS
  assertion, and exhausted transfers zero-fill until CS is released;
- releasing CS mid-transfer aborts it (`framingAborts`) and the unconsumed remainder is
  never presented as if it belonged to the next assertion;
- a read after `endRead()`, a second `beginRead()` inside an open read, or any other
  illegal sequence is recorded as a `csViolation`;
- `failTransfers` / `failReadAtByte` inject a bus failure, including mid-transfer, so
  the test can verify that CS is released and the bus lock dropped;
- `sent` captures every byte the driver wrote, so command framing is asserted
  byte-for-byte; `intState` forces the interrupt line, and `clockUs` is a manual clock
  so every timeout is tested without sleeping.

The packet bytes in `test_main.cpp` are constructed from the documented SHTP and
SH-2 layouts (datasheet 1000-3927). They are spec-derived vectors, not captures
from a physical part. Nothing here proves the real device behaves this way; that
is what the hardware bring-up checklist is for.

## Coverage

| Area | Cases |
| --- | --- |
| CRC | CCITT-FALSE check value `0x29B1`, incremental chaining |
| Frame encoder | 34-byte quaternion frame, field offsets, CRC range, corruption detection |
| Output loss | success, full sink (atomic drop), zero-byte write, partial write, no sink (suppressed, no sequence consumed), drop-then-recovery with a consumer-visible sequence gap |
| SHTP accept | valid rotation-vector transfer, reception timestamp propagation, sequence gap counting |
| SHTP reject | no data, zero length, short fragment, reserved length, bad channel, oversize transfer, payload larger than payload buffer, unexpected continuation, bus error |
| SHTP framing | one CS assertion per packet, header and payload in the same transaction, split read loses the payload, 276-byte advertisement read whole, mid-transfer failure releases CS and unlocks |
| SHTP oversize drain | oversized transfer consumed inside one CS assertion (nothing stranded, device idle afterwards), oversized continuation chain drained fragment by fragment, report arriving right after an oversize still decodes |
| SH-2 control | product id (4/5 entry forms), set-feature complete 21-byte frame and second-frame sequence, hub rejection, command timeout, reset-complete |
| SH-2 reports | RV decode (Q14 + accuracy), delay ticks and timestamp reconstruction, `0xFB`/`0xFA` per-payload delta (known example, zero fallback, host-counter wrap, negative-result low 32 bits), duplicate suppression, report gaps, unknown report id |
| Sensor FSM | full boot → identify → configure → get-feature → streaming, boot timeout, unsupported report rejection, recovery from a malformed transfer, recovery backoff |
| Sensor recovery | boot-timeout recovery success, bounded backoff sequence, attempt exhaustion → terminal `Failed`, no bus traffic in terminal state, manual recovery after terminal failure, rate limiting |
| Sensor fault triggers | consecutive SPI transfer failures, report silence → `Degraded` → recovery, output sink full/unavailable counters, polling observation timestamp shared across one drain |

## Running

Windows with the MSVC Build Tools:

```
test\native\build_msvc.bat
```

Any host with g++:

```
sh test/native/build_gcc.sh
```

Both print `RESULT: PASS` and exit non-zero on failure.

The current suite is 61 cases / 438 checks, 0 failures. Counts in this file are exact
only for the run recorded in `hardware_test_report.md`; add cases rather than trimming
them when extending coverage.

## Known non-goals

`build_gcc.sh` is provided for portability but has not been executed in this
environment (no g++ is installed here); the MSVC build is the one that ran. Address
Sanitizer could not be linked in this MSVC installation.

Oversized SHTP transfers are drained rather than reassembled. With
`kRxTransferBytes` = 512 and `kRxPayloadBytes` = 320, the 276-byte channel-0
advertisement (272-byte payload) is read and delivered normally; only genuinely larger
payloads take the bounded drain path, inside the same CS assertion.

The framing invariant the mock enforces: one device presentation per CS assertion. The
header and the whole payload must be clocked before CS is released. A split read (header
in one assertion, payload in another) does not continue the transfer, and the mock's
`shtp_split_read_loses_payload` case reproduces that failure. Draining fewer than
`length` bytes strands the remainder; `shtp_oversize_drain_complete` and
`shtp_oversize_continuation` cover the accounting and fail if the subtraction is
reintroduced.
