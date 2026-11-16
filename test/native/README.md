# Host tests

These tests run the portable core on a PC with no Arduino and no hardware.
They are the only automated evidence for the protocol layers; they do not
replace a test against a real BNO085.

## What is simulated

`mock_hal.h` implements `bno085::IHal` and emulates the SPI side of the device:

- `queuePacket()` enqueues one SHTP transfer as raw bytes;
- a 4-byte read is treated as a header peek and does **not** consume the queued
  item, matching the fact that the device re-presents its header on each new CS
  assertion;
- reads longer than the queued item zero-fill and consume it (a truncated
  transfer, which the transport must reject);
- `sent` captures every byte the driver wrote, so command framing is asserted
  byte-for-byte;
- `failTransfers` makes the bus fail, `intState` forces the interrupt line, and
  `clockUs` is a manual clock so every timeout is tested without sleeping.

The packet bytes in `test_main.cpp` are constructed from the documented SHTP and
SH-2 layouts (datasheet 1000-3927). They are spec-derived vectors, not captures
from a physical part. Nothing here proves the real device behaves this way; that
is what the hardware bring-up checklist is for.

## Coverage

| Area | Cases |
| --- | --- |
| CRC | CCITT-FALSE check value `0x29B1`, incremental chaining |
| Frame encoder | 34-byte quaternion frame, field offsets, CRC range, corruption detection |
| SHTP accept | valid rotation-vector transfer, reception timestamp propagation, sequence gap counting |
| SHTP reject | no data, zero length, short fragment, reserved length, bad channel, oversize transfer, payload larger than payload buffer, unexpected continuation, bus error |
| SHTP oversize drain | 552-byte advertisement consumed completely (nothing stranded, device idle afterwards), oversized continuation chain drained fragment by fragment, report arriving right after an oversize still decodes |
| SH-2 control | product id (4/5 entry forms), set-feature byte layout, hub rejection, command timeout, reset-complete |
| SH-2 reports | RV decode (Q14 + accuracy), delay ticks and timestamp reconstruction, duplicate suppression, report gaps, unknown report id |
| Sensor FSM | full boot → identify → configure → get-feature → streaming, boot timeout, unsupported report rejection, recovery from a malformed transfer, recovery backoff |

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

## Known non-goals

`build_gcc.sh` is provided for portability but has not been executed in this
environment (no g++ is installed here); the MSVC build is the one that ran.

Oversized SHTP transfers are drained rather than reassembled. With
`kRxTransferBytes` = 256 and `kRxPayloadBytes` = 192, every payload the device
can produce fits in one transfer, but the channel-0 advertisement (552 B on the
tested part) does not, so the drain path runs on every cold boot.

The drain accounting invariant, which the mock enforces and which the hardware
confirmed: the 4-byte header peek does not consume anything, so a read that
starts a transfer must clock out the full `length` bytes, header included.
Draining `length - 4` strands 4 payload bytes; the device then re-presents them
and the next `service()` parses payload as a header. Two regression cases cover
this (`shtp_oversize_drain_complete`, `shtp_oversize_continuation`) and both
fail if the subtraction is reintroduced.
