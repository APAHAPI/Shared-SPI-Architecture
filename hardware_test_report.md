# BNO085Multi - Physical Hardware Validation, Stage 1

**Initial run:** 2026-11-20
**Post-fix verification:** 2026-11-23
**Board:** ESP32-S3 (QFN56, rev v0.2, 8 MB PSRAM), MAC `94:a9:90:d3:5a:b8`
**Sensor:** one BNO085 on SPI, COM11 (CH343 USB-UART bridge)
**Repository:** `Shared SPI Architecture`, branch `main`, HEAD `7227135` ("Create hardware_test_report.md"). The SPI framing fix is present as uncommitted working-tree changes (nothing staged, committed or pushed).
**Firmware under test:** `examples/01_SingleSensor`, `esp32:esp32:esp32s3`, ESP32 core 3.3.12, SPI mode 3 at 1 MHz, pins SCK 12 / MISO 13 / MOSI 11 / CS 10 / INT 9 / RST 8.

---

## 1. Verdict

**Read-framing defect: FIXED and verified on hardware.**
**Stage 1 acceptance: still FAILED, at Product ID.**

| Result | Evidence |
|---|---|
| PASS - the first boot after power-on reads a 276-byte SHTP transfer (272-byte payload) inside one CS assertion | Trace log section 8.1: `hdr len=276`, payload contains `1.0.0`, `SHTP`, `control`, `executable`, `device`, `sensor` |
| PASS - no oversize, no drain, no fragments, no sequence gaps | `serial_final3.log`: `oversize=0 badFrag=0 seqGaps=0`; transport harness section 8.3 |
| PASS - reset-complete is received and boot waits for it | `reset complete (epoch 1)` then `identifying`; new host test `sensor_waits_for_reset_complete` |
| PASS - host suite and static analysis | 61 cases / 438 checks, 0 failures; `/analyze` 0 warnings |
| PASS - ESP32-S3 compilation | 326 806 B flash (24 %), 24 640 B static RAM (7 %), no warnings with `--warnings all` |
| FAIL - Product ID | `product=0x00000000`, `fw=0.0.0`, `lastError=sh2-timeout` |
| NOT REACHED - SH-2 init, RV at 100 Hz, quaternion, timestamps, rates | no RV report was ever produced |

The read-framing defect that invalidated the original run is gone. The remaining failure is a **hardware wiring/configuration issue, not a read-framing issue**: the BNO085 `PS0/WAKE` pin is tied HIGH (`wake=-1`), so after the corrected boot read consumes the pending transfers the hub stops accepting host writes. The F9 Product ID request is written, the hub does not answer, and the sensor enters bounded recovery.

This was proven, not inferred:

- A raw write of the F9 request with INT **idle** produces **zero** transfers, as does a software reset.
- The same F9 write issued while the hub is asserting INT (reset-complete still pending) is accepted and returns **two `0xF8` Product ID Response transfers** (16-byte and 48-byte payloads = four 16-byte records). The sensor, MOSI and the product-ID decoder are all healthy.

**Next action:** wire PS0/WAKE to a free GPIO, set `BNO085_WAKE_PIN` in the example, and re-run this procedure. PS0 must still be HIGH through reset (the HAL drives it HIGH before `pulseReset()`), then pulsed low to wake the hub. Do not claim Stage 1 until Product ID, SH-2 init and 100 Hz RV streaming succeed. No soak or fault injection was attempted.

The previous "552-byte drain PASS" is **invalidated** (section 10).

---

## 2. What was under test

| Item | Value |
|---|---|
| HEAD | `7227135` |
| Working tree | the framing fix, uncommitted (see the review checkpoint) |
| Tracked examples | `examples/01_SingleSensor/01_SingleSensor.ino` only |
| Library version | `0.1.0` |
| Build | `BNO085_BINARY_OUTPUT 0` (text diagnostics), `BNO085_LOG_LEVEL Info` for the final run; one Trace build from an identical temporary copy for first-boot proof |
| Modes | `SPI_MODE3`, MSBFIRST, 1 MHz, 512-byte RX transfer buffer, 320-byte payload buffer |
| Capture | `System.IO.Ports.SerialPort`, 115 200 8N1, one continuous session; RTS pulsed to reset the ESP32 for cold-boot captures |

The original report's §8 root cause was checked against the actual code before any change. It was correct that header and payload were read in two separate `SpiBusManager::transfer()` calls, i.e. two CS assertions. It was incomplete about the 552-byte length (section 3.2).

---

## 3. Original defect

### 3.1 Split read

`ShtpTransport::service()` read the 4-byte header with one `readRaw()` and then read the payload with a second `readRaw()`. Each HAL call was a complete, self-contained transaction:

```cpp
digitalWrite(csPin, LOW);
spi_->beginTransaction(settings_);
spi_->transferBytes(...);
spi_->endTransaction();
digitalWrite(csPin, HIGH);
```

The BNO085 presents one SHTP transfer starting at its header and drops the remainder when CS is released. A second CS assertion therefore does not continue the previous transfer: it presents either the next transfer or a re-presented continuation. The payload was never read from the position the header described, so every packet was misparsed.

### 3.2 The 552-byte length was a one-bit slip, with two contributors

The true advertisement is **276 bytes** (`0x0114`). `552 = 0x0228 = 0x0114 << 1`. There were two independent contributors:

1. **Split read (above).** Releasing CS between the header and the body makes the body read return something other than the payload.
2. **CS asserted before the SPI mode was applied.** On the first transfer after `SPI.begin()`, the core's clock idles low (mode 0) but the transaction uses `SPI_MODE3` (idle high). The old order was `CS low -> beginTransaction(mode 3) -> transfer`, so the clock transition happened while the device was selected and looked like one extra clock edge. The first header was sampled one bit late and `0x0114` was read as `0x0228` = 552. This is why the original run's very first boot reported `552 > 256` even though every raw probe measured 276.

The post-fix cold-boot capture reproduced the second cause exactly (`serial_final.log`: `oversize transfer 552 > 512`) before the ordering was corrected; the first-boot Trace after the correction reads `len=276` (section 8.1). Both causes are fixed.

### 3.3 Correction to the original report's Product ID claim

The original report said the packet `len=20 ch=2 seq=0` was the Product ID response header. It is not. It is the **boot-time unsolicited command response `F1 00 84`**, which the hub sends before any host write (and which the raw probes also see). The Product ID request was never answered in that run. This matters because it hid the write-path failure behind the read-framing failure.

---

## 4. Why the old host suite missed it

The old `test::MockHal` was a byte queue, not a bus:

- `transfer()` copied bytes out of one queue and never modelled CS at all.
- A 4-byte read was special-cased as a non-consuming "peek", so the mock *required* the split read to work.
- There was no transaction boundary, no `bytes clocked while CS asserted`, no abort when CS is released, and no illegal-continuation check.
- It could not represent a device that drops and re-presents a transfer, and it could not represent a clock edge caused by changing SPI mode.

The old `testShtpOversizeDrainConsumesWholeTransfer` therefore asserted the buggy behaviour. It also "verified" sequence tracking on a stream that could not occur.

---

## 5. Implemented correction

### 5.1 HAL: one continuous CS assertion

`IHal` now exposes:

```cpp
virtual bool beginRead();              // take the bus, set the mode, assert CS
virtual bool read(uint8_t* dst, size_t len);  // bytes while CS stays low
virtual void endRead();                // release CS and the bus; idempotent
```

`SpiBusManager` backs it with `beginTransaction()` / `transferInTransaction()` / `endTransaction()`:

- the bus lock and CS span the **entire** SHTP transfer;
- the lock is bounded (2 ms) and `beginTransaction()` fails fast on a nested read;
- `endTransaction()` is idempotent, releases CS before ending the SPI transaction, and unlocks;
- `end()` remembers the active CS pin, so tearing the bus down mid-read cannot leave CS asserted;
- the SPI transaction is configured **before** CS goes low and ended **after** CS goes high, so the mode-0 -> mode-3 transition cannot inject a clock edge;
- nothing but SPI byte transfers happens between `beginRead()` and `endRead()`.

### 5.2 Transport

`ShtpTransport::service()` now does exactly one `beginRead()`, reads the 4-byte header, reads `length - 4` payload bytes (or drains the remainder), and calls `endRead()` on every exit path. Logging and payload delivery are deferred until after `endRead()`, so no serial output occurs while CS is asserted. This applies to normal reads, malformed transfers, and oversized drains. Each service call reads one device presentation; a continuation fragment is left for the next call rather than clocked with CS still low.

### 5.3 Buffers

| Constant | Before | After | Reason |
|---|---|---|---|
| `BNO085_RX_TRANSFER_BYTES` | 256 | 512 | The 276-byte advertisement must be buffered whole. 512 gives room for the advertisement and control responses without per-byte splitting. Static asserts keep the payload buffer smaller than the transfer buffer and enforce a minimum. |
| `BNO085_RX_PAYLOAD_BYTES` | 192 | 320 | The advertisement payload is 272 bytes; 192 would return `Oversize` even with a correct read. 320 covers it; larger payloads still take the bounded drain path. |

No unrelated buffer was changed. The TX buffer and output buffers are untouched.

### 5.4 Boot ordering (hardware-discovered)

With the correct read, the advertisement is consumed and INT deasserts *before* the hub has finished booting. Sending the Product ID request on the advertisement makes the boot indistinguishable from a timeout. The sensor now waits for the executable-channel reset-complete (`Sh2Client::epoch()` change) before entering `Identifying`, then sends the request.

### 5.5 Mock and tests

`test/native/mock_hal.h` now models:

- CS assertion and deassertion counts;
- transaction boundaries and bytes clocked while CS is asserted;
- exactly one transfer presented per CS assertion;
- zero-fill after that transfer is exhausted, with the next transfer presented only on the next assertion;
- an aborted transfer when CS is released mid-packet (`framingAborts`);
- an error (`csViolations`) if a read is attempted after CS is released or if an independent transfer is started inside an open read;
- injectable mid-transfer failure (`failReadAtByte`).

New/updated regressions (all registered in `test/native/test_main.cpp`):

| # | Requirement | Test |
|---|---|---|
| 1 | one CS assertion per packet | `shtp_single_cs_per_packet` |
| 2 | header and payload in the same transaction | `shtp_header_payload_same_cs` |
| 3 | releasing CS between header and payload reproduces/rejects the failure | `shtp_split_read_loses_payload` |
| 4 | 276-byte advertisement consumed correctly | `shtp_advertisement_276` |
| 5 | oversized drain inside one CS | `shtp_oversize_drain_complete`, `shtp_oversize_continuation` |
| 6 | next packet after a drain | `shtp_oversize_drain_complete` (and `shtp_valid_rotation_vector`) |
| 7 | mid-transfer failure releases CS and unlocks | `shtp_mid_transfer_failure_releases_cs` |
| 8 | normal reception | `shtp_valid_rotation_vector`, `shtp_no_data`, error-path cases |
| 9 | fragmented/continued payloads remain correct | `shtp_unexpected_continuation`, `shtp_continuation_fragment_clean` |
| 10 | sequence tracking | `shtp_sequence_gap` |
| - | wait for reset-complete before writing | `sensor_waits_for_reset_complete` |

Continuation semantics are unchanged: the core does not assemble multi-transfer payloads, so an unexpected continuation is rejected per fragment and the next packet still reads cleanly.

### 5.6 Hardware harnesses

`test/hardware/01_HeaderProbe` (now v4) and `test/hardware/02_TransportDrain` use `beginRead()/read()/endRead()` and the corrected expectations (276-byte advertisement, 272-byte payload, no oversize). Both compile and pass on COM11 (section 8).

---

## 6. Host verification

Command:

```
test\native\build_msvc.bat
```

Result:

```
61 cases, 438 checks, 0 failures
RESULT: PASS
```

MSVC `/W4`: 0 warnings. Static analysis (`cl /analyze` over the portable core): 0 warnings, 0 errors, exit 0. AddressSanitizer could not be linked in this environment (`LNK1104: cannot open file 'clang_rt.asan_static_runtime_thunk-x86_64.lib'`); it is recorded as unavailable, not as a pass.

The suite includes all prior SHTP/SH-2/sensor/recovery/output cases plus the new framing regressions. The old claims in `test/native/README.md` that the mock treats a 4-byte read as a non-consuming peek are no longer true and have been updated.

---

## 7. ESP32-S3 compilation

```
arduino-cli compile --fqbn esp32:esp32:esp32s3 --library . \
  --build-path build/stage1-final3 --warnings all examples/01_SingleSensor
```

```
Sketch uses 326806 bytes (24%) of program storage space. Maximum is 1310720 bytes.
Global variables use 24640 bytes (7%) of dynamic memory, leaving 303040 bytes for local variables. Maximum is 327680 bytes.
```

Exit 0, **no compiler warnings**.

---

## 8. Physical verification (COM11)

Upload:

```
arduino-cli upload -p COM11 --fqbn esp32:esp32:esp32s3 \
  --input-dir build/stage1-final3 examples/01_SingleSensor
```

Result: exit 0, hash verified, hard reset.

### 8.1 First boot after power-on, Trace build (proof of framing)

The Trace build is an identical temporary copy of the example with only `BNO085_LOG_LEVEL` changed, so the repository example stays at `Info`. Capture: `build/stage1/serial_first_boot_trace.log`.

```
 0.245  [I] s0 begin cs=10 int=9 rst=8 wake=-1 rate=10000us
 0.250  [D] s0 state -> booting
 0.250  [I] s0 reset pulse on pin 8 (attempt 0)
 0.256  [I] s0 boot wait, budget 300ms
 0.336  [D] s0 INT asserted
 0.342  [T] s0 hdr len=276 cont=0 ch=0(command) seq=0
 0.353  [T] shtp-rx [276] 14 01 00 00 00 01 04 00 00 00 00 80 06 31 2E 30 2E 30 00 02 02 00 01 03 ...
 0.358  [D] advertisement [272] 00 01 04 00 00 00 00 80 06 31 2E 30 2E 30 00 02 02 00 01 03 02 FF 7F 04 ...
 0.404  [D] s0 INT asserted
 0.409  [T] s0 hdr len=20 cont=0 ch=2(control) seq=0
 0.415  [T] shtp-rx [20] 14 00 02 00 F1 00 84 00 00 00 01 00 00 00 00 00 00 00 00 00
 0.415  [D] s0 unsolicited command response 0x84
 0.421  [T] s0 hdr len=5 cont=0 ch=1(executable) seq=0
 0.426  [T] shtp-rx [5] 05 00 01 00 01
 0.426  [I] s0 reset complete (epoch 1)
 0.432  [D] s0 state -> identifying
 0.432  [I] s0 device reset complete, identifying
 0.436  [T] shtp-tx [6] 06 00 02 00 F9 00
 0.437  [D] s0 -> product id request
 0.527  [W] s0 error sh2-timeout (recover)
 0.530  [W] s0 recover attempt 1/5 in 250ms
```

This is the whole first boot. The 276-byte transfer is coherent, the 272-byte payload starts with the advertisement entry `00 01 04 00` and contains `31 2E 30 2E 30` (`1.0.0`), the following control and executable transfers are also coherent, and reset-complete is observed. No oversize/drain appears.

### 8.2 Final Info build, cold boot and diagnostics

Capture: `build/stage1/serial_final3.log` (RTS reset; `d` at 1.5 s).

```
 0.241  [I] s0 begin cs=10 int=9 rst=8 wake=-1 rate=10000us
 0.246  [I] s0 reset pulse on pin 8 (attempt 0)
 0.247  [I] s0 boot wait, budget 300ms
 0.401  [I] s0 reset complete (epoch 1)
 0.402  [I] s0 device reset complete, identifying
 0.501  [W] s0 error sh2-timeout (recover)
 0.501  [W] s0 recover attempt 1/5 in 250ms
 ...
 1.522  state=booting lastError=sh2-timeout stage=1
 1.528  product=0x00000000 fw=0.0.0  granted=0us
 1.533  packets=6 tx=2 bytes=602 spiErrors=0 busTimeouts=0
 1.539  short=0 badChan=0 badFrag=0 oversize=0 seqGaps=0 spurious=0
 1.539  sh2Timeouts=0 badResp=0 unknown=0 dups=0 gaps=0
 1.544  resets=3 ints=6 reports=0 lastReport=0us maxService=2367us
 1.550  recovery attempts=2 successes=0 failures=2 terminal=0 reason=sh2-timeout
```

`packets=6 bytes=602` is two complete boot bursts of 301 bytes each (276 + 20 + 5), read with no errors. `maxService=2367 us` is the 276-byte read plus overhead; the slipped 552-byte read measured 4704 us in the pre-ordering capture. `oversize=0` is the definitive change from the original run.

### 8.3 Transport framing harness (`test/hardware/02_TransportDrain`)

Capture: `build/stage1/hw02_framing.log`.

```
=== BNO085Multi transport framing validation ===
pins: SCK=12 MISO=13 MOSI=11 CS=10 INT=9 RST=8 clock=1000000Hz
hal.begin -> ok
boot: INT asserted after 93888 us (up)
servicing while INT asserted:
  payload ch=0(command) len=272 t=93964us head=00 01
  service[1] -> Ok
  INT deasserted: device idle
--- summary ---
service calls=1 oversize=0 ok=1 other=0
final service -> NoData (idle, no stranded tail)
stats: packets=1 bytes=276 oversizePayloads=0 badFragments=0 shortFragments=0 badChannels=0 seqGaps=0 rxErrors=0
payloads=1 firstCh=0 firstLen=272 lastCh=0 lastLen=272
bus transactions=1 lockTimeouts=0 busErrors=0
RESULT: PASS - 276-byte advertisement, one CS assertion per packet, stream idle
```

### 8.4 Header/framing probe (`test/hardware/01_HeaderProbe`, v4)

Capture: `build/stage1/hw01_framing.log`.

```
=== BNO085Multi framing probe v4 ===
boot: INT asserted after 93869 us (up)
reading complete packets:
HDR len=276 ch=0(command) seq=0 raw=14 01 00 00 t=93948us
  head(96): 00 01 04 00 00 00 00 80 06 31 2E 30 2E 30 00 02 02 00 01 03 02 FF 7F 04 ... |.........1.0.0...................SHTP......control.........executable......device.........sensor|
HDR len=20 ch=2(control) seq=0 raw=14 00 02 00 t=158234us
  head(16): F1 00 84 00 00 00 01 00 00 00 00 00 00 00 00 00
HDR len=5 ch=1(executable) seq=0 raw=05 00 01 00 t=170117us
  head(1): 01
--- summary ---
attempts=3 packets=3 cont=0 idle=0 drained=289 fail=0
bus transactions=3 lockTimeouts=0 busErrors=0
RESULT: PASS - complete SHTP packets consumed
```

Three packets, three CS assertions, zero errors. This independently confirms the single-CS framing and that the advertisement is 276 B, not 552 B.

### 8.5 Measured timings

| Measurement | Value |
|---|---|
| Reset release -> first INT | 93.9 ms (probe), 94-100 ms in the examples |
| 276-byte advertisement read | 2.2-2.4 ms at 1 MHz (`maxService=2367 us`) |
| Slipped 552-byte read (pre-ordering fix) | 4.7 ms (`maxService=4704 us`) |
| Bus errors / lock timeouts / spurious INT | 0 / 0 / 0 |
| Oversized transfers | **0** |
| RV reports | **0** |

---

## 9. Product ID investigation

### 9.1 Writes are ignored when INT is idle

Throw-away raw probes (outside the repository, in `%TEMP%\opencode`) reset the device, read the boot burst with one CS per packet, and then write the Product ID request. Captures: `probe_write_path.log`, `probe_pid_timing.log`.

```
--- A: product id, 5-byte frame ---
  raw=0114 len=276 ... payload=272
  raw=0014 len=20 ... F1 00 84 ...
  raw=0005 len=5 ... 01
  -> product-id 5B (5 bytes) at t=487323us INT=1
  tx: 05 00 02 00 F9
  transfers after=0

--- B: product id, 6-byte frame ---
  -> product-id 6B (6 bytes) ... INT=1
  tx: 06 00 02 00 F9 00
  transfers after=0

--- C: software reset ---
  -> software reset (5 bytes) ... INT=1
  tx: 05 00 01 00 01
  transfers after=0
```

The same result with a 0-delay, 50 ms or 150 ms delay after reset-complete. Reads work; writes do nothing. The 20-byte `F1 00 84` packet is the boot-time unsolicited command response, not a Product ID response.

### 9.2 Writes are accepted while the hub asserts INT

A second raw probe waits for the reset-complete transfer to be pending and writes F9 **without reading it first**. Capture: `probe_write_while_int.log`.

```
  boot packets read=2
  INT before write=0
  -> F9 request while INT=0 at t=486776us
  raw=0014 len=20 cont=0 ch=2 seq=1 payload=16  <-- PRODUCT ID RESPONSE INT=1
    head: F8 04 03 02 B4 A6 98 00 06 00 00 00 0D 00 00 00
  raw=0034 len=52 cont=0 ch=2 seq=2 payload=48  <-- PRODUCT ID RESPONSE INT=1
    head: F8 00 01 02 96 A4 98 00 E6 00 00 00 04 00 00 00 F8 00 04 04 36 A3 98 00 ...
  packets after write=2 productIdResponse=1
```

The hub answered immediately. The response is four 16-byte Product ID records split across two SHTP transfers (16 + 48 payload bytes); `Sh2Client` already accumulates them (`prodEntries_`, `prodExpected_ = 4`). This proves the sensor, MOSI, the F9 command and the parser are all functional.

### 9.3 Why the library still times out

The BNO08X protocol-select table is PS1/PS0 = 1/1 for SPI at reset. After reset, PS0 becomes the active-low WAKE input. The datasheet/migration guide states that PS0 **must be connected to a GPIO** for SPI writes; if it is tied HIGH, the host can only write while the hub is asserting INT (for example, while a read is pending). Once the corrected read consumes the boot burst, INT is idle, so the F9 write is dropped.

The board under test has PS0/PS1 tied HIGH and `BNO085_WAKE_PIN = -1`. `Esp32SpiHal` already implements the wake sequence correctly:

- `begin()` drives WAKE HIGH before `pulseReset()`, so PS0 is HIGH through reset (SPI selected);
- `prepareWrite()` pulses WAKE low for 200 us and waits up to 2 ms (bounded) for INT;
- a failed wake still attempts the write and logs a warning.

No firmware change can substitute for the missing GPIO connection. A "write only while INT is asserted" workaround cannot work for the first set-feature request because, after reset-complete, the hub has no reason to assert INT until a feature is enabled - which is exactly the write that is blocked.

### 9.4 Next action

1. Connect BNO085 PS0/WAKE to a free ESP32-S3 GPIO (PS0 must be HIGH at reset; the HAL drives it HIGH).
2. Set `BNO085_WAKE_PIN` to that GPIO in `examples/01_SingleSensor`.
3. Re-run the exact Stage 1 procedure. Expect `product=0x0098xxxx`, a non-zero firmware version, set-feature/get-feature at 10 000 us, and RV reports at 100 Hz.
4. Do not proceed to soak/fault injection until RV streaming is stable.

---

## 10. Invalidated evidence

**The 552-byte advertisement and the "oversized transfer fully drained" PASS are invalid.**

- The advertisement is **276 bytes** (`0x0114`); 552 is `0x0114 << 1`, a one-bit slip.
- The original `02_TransportDrain` result drained that slipped presentation with split reads. It did not prove that a payload was consumed coherently; the device can return to idle after a re-presentation without the host having read the packet correctly.
- After the fix, the same harness reads one 276-byte transfer, 272-byte payload, in **one** CS assertion, and passes (section 8.3).

**The original report's "Product ID response header len=20 ch=2" is invalid.** It is the unsolicited boot command response `0x84`, present before any host write.

`docs/PHASE1-NOTES.md` and `README.md` have been corrected accordingly.

---

## 11. Pass/fail by stage

| Stage | Requirement | Result |
|---|---|---|
| 1. Boot | ESP32-S3 boots, firmware starts | **PASS** |
| 2. HAL init | pins, mode 3, 1 MHz, reset pulse, INT | **PASS** |
| 3. Sensor presence / framing | coherent 276-byte advertisement in one CS | **PASS** |
| 4. Product ID | `product != 0` | **FAIL** - blocked by PS0/WAKE wiring |
| 5. SH-2 init | reach streaming | **NOT REACHED** - `sh2-timeout` |
| 6. RV configuration | set/get feature at 100 Hz | **NOT REACHED** - `granted=0us` |
| 7. RV reception | decoded quaternion | **NOT REACHED** - `reports=0` |
| 8. Timestamps / sequences | plausible `seq`, `tMeas`, `dt` | **NOT REACHED** |
| 9. Transport health | no bus errors/desync | **PASS** on the fixed path: `spiErrors=0`, `busTimeouts=0`, `badFrag=0`, `oversize=0`, `seqGaps=0` |
| 10. INT servicing | every assertion serviced | **PASS** - 0 spurious |
| 11. Report rate | 100 Hz measured | **NOT MEASURED** |

---

## 12. Remaining limitations

1. **PS0/WAKE wiring** is the only blocker to Stage 1. The library supports it; the board does not expose it to a GPIO yet.
2. **Continuation assembly is not implemented.** The core rejects an unexpected continuation fragment cleanly and reads the next packet, but a legitimate multi-transfer payload larger than `BNO085_RX_PAYLOAD_BYTES` is not reassembled. The 276-byte advertisement and all current control/report payloads fit.
3. **`/WX` is not enabled** in the host build; warnings are checked manually (`/W4` reports none).
4. **AddressSanitizer is unavailable** in this MSVC installation (link error), so the suite was not run under ASan here.
5. **RV behaviour, timestamps, report rate, output loss under load, recovery timing, soak and fault injection remain unverified** because streaming was never reached.
6. The product-ID response arrives as four 16-byte records across two transfers on this part; the accumulator handles it, but it has not yet been exercised end-to-end on hardware through `Sh2Client` (the probe used raw SPI).

---

## 13. Artifacts and commands

Tracked changes are listed in the final checkpoint. Untracked build/capture artifacts (all under the git-ignored `build/`):

| File | Contents |
|---|---|
| `build/stage1-final3/` | final Info build |
| `build/stage1/serial_first_boot_trace.log` | first boot at Trace (276 B proof) |
| `build/stage1/serial_final3.log` | final Info cold boot + `d` diagnostics |
| `build/stage1/hw01_framing.log` | header/framing probe PASS |
| `build/stage1/hw02_framing.log` | transport framing harness PASS |
| `build/stage1/probe_write_path.log` | F9/reset writes with INT idle: no response |
| `build/stage1/probe_write_while_int.log` | F9 while INT asserted: 0xF8 response |
| `build/stage1/serial_final.log` | pre-ordering cold boot: `552 > 512` (second cause) |

Commands used:

```
test\native\build_msvc.bat
cmd /c build\analyze_msvc.bat
arduino-cli compile --fqbn esp32:esp32:esp32s3 --library . --build-path build/stage1-final3 --warnings all examples/01_SingleSensor
arduino-cli upload -p COM11 --fqbn esp32:esp32:esp32s3 --input-dir build/stage1-final3 examples/01_SingleSensor
powershell -File %TEMP%\opencode\serial_capture.ps1 -Port COM11 -Seconds 8 -Out build\stage1\serial_final3.log -Reset -SendSchedule "1.5|d"
```
