# BNO085Multi — Physical Hardware Validation, Stage 1

**Date:** 2026-11-20
**Board:** ESP32-S3 (QFN56, rev v0.2, 8 MB PSRAM), MAC `94:a9:90:d3:5a:b8`
**Sensor:** one BNO085 on SPI, COM11 (CH343 USB-UART bridge)
**Repository:** `Shared SPI Architecture`, branch `main`, HEAD `330ef78` ("Add bounded recovery and output loss tracking"), working tree clean before and after the run.

---

## 1. Verdict

**Stage 1 FAILED.** The firmware never reached SH-2 operation:

- No Product ID (`product=0x00000000`).
- SH-2 initialisation timed out at the identify stage (`lastError=sh2-timeout`, `stage=2`).
- Rotation Vector was **never configured** and **no RV report was ever decoded** (`reports=0`), so there is no report rate, no quaternion, no timestamp and no sequence data to report.

The failure is **not** a dead sensor and **not** a wiring fault. The BNO085 answered correctly on the bus; a diagnostic probe proved the sensor emits a clean, coherent SHTP advertisement and deasserts INT when the packet is read properly. The failure is a **library defect in the SPI read framing** (§7), discovered during this run.

Because the failure occurred at bring-up, the run was stopped at that point. No soak testing, no fault injection, and no "try Game Rotation Vector instead" substitution were attempted.

---

## 2. Inspection before flashing

| Item | Finding |
|---|---|
| Branch / HEAD | `main` / `330ef78` |
| Working tree | clean (`git status --porcelain` empty) |
| Tracked examples | exactly one: `examples/01_SingleSensor/01_SingleSensor.ino` |
| Library version | `0.1.0` |
| Prior commit context | `d9fa757` timestamp-delta baseline, `330ef78` bounded recovery + output loss tracking |
| Build configuration on disk | `examples/01_SingleSensor/build/esp32.esp32.esp32s3/build.options.json`: `fqbn = esp32:esp32:esp32s3`, `-Os`, esp32 core 3.3.12 |

**Chosen example and why:** `examples/01_SingleSensor` is the only tracked example and is the repository's Phase 1 acceptance sketch. It already performs exactly the Stage 1 sequence:

- HAL/SPI bring-up with the required pin map (`SCK 12`, `MISO 13`, `MOSI 11`, `CS 10`, `INT 9`, `RST 8`), `SPI_MODE3`, 1 MHz (`BNO085_SPI_HZ 1000000`).
- Reset pulse, boot wait, Product ID request.
- SH-2 set-feature / get-feature for Rotation Vector at 10 000 µs (100 Hz).
- RV streaming with `seq`, `tMeas`, and signed `dt` per report.
- `d` diagnostics, including the Pass 6 recovery and output-loss counters.

**Build mode:** `BNO085_BINARY_OUTPUT 0` (text mode) so the RV stream and diagnostics are human-readable on COM11.

**Identification of the flashed firmware:** the startup banner is generic, so the Pass 6 identity check is the `d` output. Both `d` dumps below contain `recovery attempts=...` and `output enabled=...`, i.e. the flashed image really is the Pass 6 build, not an older transport-drain sketch.

---

## 3. Build

```
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" ^
  compile --fqbn esp32:esp32:esp32s3 --library . ^
  --build-path build/stage1 examples/01_SingleSensor
```

Result:

```
Sketch uses 326142 bytes (24%) of program storage space. Maximum is 1310720 bytes.
Global variables use 23968 bytes (7%) of dynamic memory, leaving 303712 bytes for local variables. Maximum is 327680 bytes.
```

Identical to the committed Pass 6 footprint (326 142 B flash / 23 968 B RAM), so the binary under test matches HEAD with no local modifications.

---

## 4. Flash

```
& "...\arduino-cli.exe" upload -p COM11 --fqbn esp32:esp32:esp32s3 ^
  --input-dir build/stage1 examples/01_SingleSensor
```

Observed:

```
esptool v5.3.1
Connected to ESP32-S3 on COM11:
Chip type: ESP32-S3 (QFN56) (revision v0.2)
Features:  Wi-Fi, BT 5 (LE), Dual Core + LP Core, 240MHz, Embedded PSRAM 8MB (AP_3v3)
Crystal frequency: 40MHz
MAC: 94:a9:90:d3:5a:b8
...
Writing 'build/stage1/01_SingleSensor.ino.bin' at 0x00010000...
Wrote 326288 bytes (189294 compressed) at 0x00010000 in 2.5 seconds (1049.1 kbit/s).
Verifying written data...
Hash of data verified.
Hard resetting via RTS pin...
```

---

## 5. COM11 configuration

| Setting | Value |
|---|---|
| Port | `COM11` (CH343 USB-UART, `Serial` = UART0, `ARDUINO_USB_CDC_ON_BOOT` not enabled) |
| Baud | 115200 |
| Framing | 8 data bits, no parity, 1 stop bit |
| Flow control | none |
| DTR / RTS | deasserted for normal run; RTS pulsed once to trigger a controlled reset for the boot capture |

Capture was done with `System.IO.Ports.SerialPort` (a small script in `%TEMP%\opencode\serial_capture.ps1`), holding the port open for one continuous session so that no extra resets were introduced.

---

## 6. Test sequence actually executed

1. Compiled HEAD to `build/stage1` (§3).
2. Flashed COM11 and let the board self-reset (§4).
3. Opened COM11 once and captured ~16 s of output, sending `d` after 6 s (`build/stage1/serial_stage1.log`).
   - The first sample already showed `state=failed`, i.e. the board had completed its whole init/recovery cycle before the port was opened.
4. To see the boot path, one controlled RTS reset was issued with the port already open, and the boot + two `d` dumps were captured (`build/stage1/serial_boot.log`). **This was the only deliberate reset of the run.**
5. On failure, the source path was inspected (§7) and the raw SPI stream was measured with two throw-away probes (§8). The probes were not retries of the failing firmware; they read bytes and printed them.
6. The committed Pass 6 build was flashed back so the board was left running the firmware under test.

---

## 7. Raw serial output

### 7.1 Boot and bring-up (Pass 6 build, text mode)

```
 0.247  [I] s0 begin cs=10 int=9 rst=8 wake=-1 rate=10000us
 0.252  [I] s0 reset pulse on pin 8 (attempt 0)
 0.252  [I] s0 boot wait, budget 300ms
 0.257  BNO085Multi phase-1 baseline started
 0.257
 0.261  commands: r = recover sensor, d = diagnostics, q = quaternion, h = help
 0.346  [W] s0 oversize transfer 552 > 256, drained
 0.351  [W] s0 error shtp-bad-frame (shtp-oversize)
 0.352  [W] s0 ch0 seq gap: expected 1 got 2
 0.355  [I] s0 device responded, identifying
 0.412  [W] s0 unknown control report 0x00 at 0/16
 0.446  [W] s0 error sh2-timeout (recover)
 0.447  [W] s0 recover attempt 1/5 in 250ms
 0.697  [I] s0 reset pulse on pin 8 (attempt 1)
 ...
 0.794  [W] s0 oversize transfer 276 > 256, drained
 0.799  [W] s0 error shtp-bad-frame (shtp-oversize)
 0.800  [W] s0 ch0 seq gap: expected 1 got 2
 0.805  [W] s0 unexpected continuation (ch=0, 35 bytes)
 0.810  [W] s0 error shtp-bad-frame (shtp-frame)
 0.811  [I] s0 device responded, identifying
 0.860  [W] s0 unknown control report 0x00 at 0/16
 0.906  [W] s0 error sh2-timeout (recover)
 0.906  [W] s0 recover attempt 2/5 in 500ms
 ...
 9.241  [W] s0 error sh2-timeout (terminal)
 9.242  [E] s0 terminal failure: sh2-timeout
10.037  [failed] reports=0 errors=0 maxService=11690us
```

(The five recovery attempts are all identical: reset pulse → oversize advertisement → sequence gap → a bogus control report → `sh2-timeout`.)

### 7.2 `d` diagnostics — first dump (mid-recovery)

```
state=booting lastError=sh2-timeout stage=1
product=0x00000000 fw=0.0.0  granted=0us
packets=10 tx=5 bytes=262 spiErrors=0 busTimeouts=0
short=0 badChan=0 badFrag=4 oversize=5 seqGaps=5 spurious=0
sh2Timeouts=0 badResp=0 unknown=5 dups=0 gaps=0
resets=6 ints=14 reports=0 lastReport=0us maxService=11683us
recovery attempts=5 successes=0 failures=5 terminal=0 reason=sh2-timeout
output enabled=0 written=0 suppressed=0 noCap=0 partial=0 failed=0 dropped=0 lastError=none
```

### 7.3 `d` diagnostics — final dump (terminal failure)

```
state=failed lastError=sh2-timeout stage=2
product=0x00000000 fw=0.0.0  granted=0us
packets=12 tx=6 bytes=321 spiErrors=0 busTimeouts=0
short=0 badChan=0 badFrag=5 oversize=6 seqGaps=6 spurious=0
sh2Timeouts=0 badResp=0 unknown=6 dups=0 gaps=0
resets=6 ints=17 reports=0 lastReport=0us maxService=11690us
recovery attempts=5 successes=0 failures=6 terminal=1 reason=sh2-timeout
output enabled=0 written=0 suppressed=0 noCap=0 partial=0 failed=0 dropped=0 lastError=none
```

### 7.4 Measured timings (hardware observations)

| Measurement | Value |
|---|---|
| Reset release → first INT assertion | 0.252 s → 0.346 s ≈ **94 ms** |
| Longest single `service()` call | **11 683–11 690 µs** (the oversized-advertisement drain) |
| INT assertions over the whole run | 17 (all serviced; `spurious=0`) |
| Bus-level errors | `spiErrors=0`, `busTimeouts=0`, lock timeouts 0 |
| SHTP packets accepted / sent | `packets=12`, `tx=6`, `bytes=321` |
| RV reports decoded | **0** |
| Observed RV report rate | **not measurable — no reports were produced** |
| Quaternion values / timestamps / sequence numbers | **not available — stream never started** |

---

## 8. Root cause

### 8.1 What the code does

`ShtpTransport::service()` (`src/shtp/ShtpTransport.cpp`):

```cpp
uint8_t peek[shtp::kHeaderLen];
if (!readRaw(peek, sizeof(peek))) { ... }      // line 81-82  -> one CS window
...
if (!readRaw(rxTransfer_, length)) { ... }     // line 207    -> a SECOND CS window
```

`Esp32SpiHal::transfer()` forwards to `SpiBusManager::transfer()` (`src/hal/SpiBusManager.cpp:107-112`):

```cpp
digitalWrite(csPin, LOW);
spi_->beginTransaction(settings_);
spi_->transferBytes(out, in, len);
spi_->endTransaction();
digitalWrite(csPin, HIGH);
```

So **each `transfer()` call is a complete, self-contained CS assertion**. The header peek and the body read are therefore two separate CS assertions.

### 8.2 Why that breaks

The BNO085 presents an SHTP transfer starting at its 4-byte header, and the whole packet must be clocked out inside **one** CS assertion. Splitting it means the body read does not return the payload of the packet that was peeked.

Evidence from the trace build (`build/stage1-trace/serial_trace.log`, `LogLevel::Trace`):

```
[T] s0 hdr len=20 cont=0 ch=2(control) seq=0        <- correct Product ID response header
[T] shtp-rx [20] 00 05 00 80 7C 40 21 00 ...       <- body read returns something else entirely
[W] s0 unknown control report 0x7C at 0/16
```

The header is *exactly right* for a Product ID response (20 bytes total = 16-byte SH-2 payload + 4-byte SHTP header, channel 2, first sequence). So **the sensor did answer the Product ID request**. The 16 payload bytes are never read from the right position, so the report ID is never seen as `0xF8` and `handleProdId()` is never called.

The same thing explains every other symptom:

| Symptom | Explanation |
|---|---|
| `oversize transfer 552/276 > 256` | header re-read or bit-slipped by the split read; the true packet is 276 B (§8.3) |
| `ch0 seq gap: expected 1 got 2` | the drained packet is never sequence-tracked, so the next parsed one looks like a gap |
| `unexpected continuation (ch=0, 35 bytes)` | the "header" is read from the middle of a payload |
| `unknown control report 0x00 at 0/16` | misaligned payload bytes |
| deterministic `badFrag=5`, `oversize=6` | exactly one bad parse per boot cycle (1 initial + 5 recovery resets) |

### 8.3 Confirmation with raw probes

Two throw-away sketches (untracked, in `%TEMP%\opencode`, same pins/mode/clock) read the bus with **no** library logic.

**Probe 1 — 4 bytes per CS assertion (i.e. what the library does for the peek):**

```
0128  14 01 00 00        <- header: len=0x0114=276, cont=0, ch=0, seq=0
0132  00 00 00 00
0136  14 81 00 01        <- a header again, seq 1
0140  14 81 00 02
0144  14 81 00 03
...
```
Every CS assertion re-presents a header; the sequence number keeps advancing and **INT stays asserted for the whole run** (`int=1` on every line). The packet is never consumed.

**Probe 2 — header + payload in a single CS assertion:**

```
INT after boot wait = 0
pkt0 raw=0114 len=276 cont=0 ch=0(cmd) seq=0 payload=272 dt=2900us INT=1
  head: 00 01 04 00 00 00 00 80 06 31 2E 30 2E 30 00 02 02 00 01 03 02 FF 7F 04 02 00 01 05 02 FF 7F 08
  tail: 27 00 28 0E 29 0C 2A 0E
INT idle, stop
```

With one CS assertion the packet is completely coherent: a 276-byte channel-0 advertisement whose payload contains `31 2E 30 2E 30` — the ASCII string **`1.0.0`**, the SH-2 version. INT deasserts immediately after the packet is read. **The sensor and the wiring are healthy.**

### 8.4 Correction to the earlier "552 B drain" evidence

`docs/PHASE1-NOTES.md` records the advertisement as **552 B** and the earlier transport-drain test as a PASS. Both probes read the header as `0x0114` = **276 B**; 552 = `0x0228` = `0x0114 << 1`, i.e. exactly a one-bit slip of the same 16-bit length field. So:

- The advertisement on this part is **276 B** (4 entries + version string), not 552 B.
- The earlier "PASS — oversized transfer fully drained, stream idle" is not evidence that the drain consumed the payload. With the split read the device re-presents the transfer, and INT can fall back to idle without the packet having been read coherently. That result should be treated as invalidated by this run.

### 8.5 Fix (identified, **not implemented**)

The read must be performed inside a single CS assertion:

1. Add a two-phase read to `IHal` (e.g. `beginRead()` / `read(dst, len)` / `endRead()`), implemented in `Esp32SpiHal`/`SpiBusManager` so CS is asserted once for the header **and** the `length - 4` payload bytes.
2. Use it in `ShtpTransport::service()` for both the normal path and the oversized-drain path (the drain must also stay inside one window, or be restructured to read `4 + payload` in one go).
3. Update `test/native/mock_hal.h` with the new HAL methods; the mock already models transfers, so the framing can be tested host-side (the existing 54-case suite cannot catch this, because the mock's `transfer()` does not model CS deassertion semantics).
4. Re-check `BNO085_RX_TRANSFER_BYTES`: a 276-byte advertisement exceeds the 256-byte buffer, so the buffer must grow (or the drain must stay correct) for a cold boot to be parsed at all. A 512-byte transfer buffer is the simplest correct choice.

Estimated change scope: `src/hal/IHal.h`, `src/hal/Esp32SpiHal.{h,cpp}`, `src/hal/SpiBusManager.{h,cpp}`, `src/shtp/ShtpTransport.cpp`, `test/native/mock_hal.h`, plus a new regression test. This is one subsystem (SPI read framing), but it is a real API change and was deliberately **not** started here.

---

## 9. Pass/fail by stage

| Stage | Requirement | Result |
|---|---|---|
| 1. Boot | ESP32-S3 boots, firmware starts, banner | **PASS** |
| 2. HAL init | SPI bus + pins configured, reset pulse, INT configured | **PASS** |
| 3. Sensor presence | BNO085 responds on SPI (INT asserts, SHTP headers) | **PASS** (confirmed by probe: coherent 276 B advertisement) |
| 4. Product ID | `product != 0x00000000` | **FAIL** — `0x00000000`, `fw=0.0.0` |
| 5. SH-2 init | reach streaming | **FAIL** — `sh2-timeout`, `stage=2` (identify) |
| 6. RV configuration | set/get feature at 100 Hz | **NOT REACHED** — `granted=0us` |
| 7. RV reception | reports decoded, plausible quaternion | **NOT REACHED** — `reports=0` |
| 8. Sequence / timestamps | `seq`, `tMeas`, `dt` plausible | **NOT REACHED** — no reports |
| 9. Counters / transport health | no bus errors, no desync | **FAIL (partial)** — `spiErrors=0`, `busTimeouts=0`, `spurious=0`, `short=0`, `badChan=0`, but `badFrag=5`, `oversize=6`, `seqGaps=6` |
| 10. INT servicing | INT consistently serviced | **PASS** — 17 assertions, 0 spurious; each serviced without a bus error |
| 11. Report rate | measured | **NOT MEASURED** — no reports |

---

## 10. Firmware changes made

**Library / example:** none. The working tree is clean at HEAD `330ef78` (`git status --porcelain` empty) and the library sources were not edited.

Temporary diagnostic actions, all reverted or kept outside the repository:

| Action | Status |
|---|---|
| `examples/01_SingleSensor/01_SingleSensor.ino`: `BNO085_LOG_LEVEL` Info → Trace, for one diagnostic build (`build/stage1-trace`) | **reverted**; file matches HEAD |
| `%TEMP%\opencode\ShtpProbe\ShtpProbe.ino` (4-byte-per-CS raw probe) | untracked, outside repo, discarded |
| `%TEMP%\opencode\ShtpProbe2\ShtpProbe2.ino` (single-CS whole-packet probe) | untracked, outside repo, discarded |
| Re-flashed committed Pass 6 build (`build/stage1`) after probing | board left on the firmware under test |

Artifacts (all under the git-ignored `build/`, nothing staged or committed):

- `build/stage1/serial_stage1.log` — first continuous capture
- `build/stage1/serial_boot.log` — boot capture + two `d` dumps
- `build/stage1-trace/serial_trace.log` — Trace-level header/hex capture
- `%TEMP%\opencode\probe_raw.log`, `%TEMP%\opencode\probe2.log`

---

## 11. Remaining unverified requirements

Everything downstream of the transport is still unverified on hardware:

1. Product ID retrieval and firmware version decode.
2. SH-2 initialisation to streaming state.
3. Rotation Vector set-feature/get-feature negotiation at 100 Hz and the granted interval.
4. Reception and decoding of valid RV quaternions (finiteness, plausibility, accuracy level).
5. `seq` monotonicity and `tMeas` / `dt` plausibility on live reports.
6. Observed report rate versus the granted 100 Hz (and jitter/service cadence).
7. Output counters (`framesWritten`, `framesPartial`, ...) under real streaming load (they are all zero because no output path was exercised).
8. Recovery behaviour on a real fault, soak stability, and any long-run drift.

---

## 12. Hardware observations vs software assumptions

**Observed on hardware (measured, repeatable):**

- ESP32-S3 boots the Pass 6 image; `d` confirms the Pass 6 recovery/output lines.
- Reset release → first INT ≈ 94 ms.
- BNO085 asserts INT and presents SHTP transfers on channel 0.
- A single-CS read of the first transfer returns a coherent 276-byte advertisement containing the ASCII string `1.0.0`, and INT deasserts afterwards.
- A 4-bytes-per-CS read loop receives only headers with advancing sequence numbers and INT never deasserts.
- The product ID *response header* (`len=20 ch=2 seq=0`) does arrive; its payload is never read from the correct position.
- Zero SPI/bus errors, zero lock timeouts, zero spurious interrupts.
- No RV report is ever decoded (`reports=0`).

**Software assumptions / inferences (not directly measured here):**

- The BNO085 requires the header and payload to be clocked inside one CS assertion. This is inferred from the contrast between the two probes; it is consistent with the single-CS packet being perfectly coherent and INT deasserting.
- `552 B` is a misread of `276 B` (exact 1-bit slip); the earlier drain "PASS" therefore did not prove payload consumption.
- PS0/PS1 were HIGH at reset and the wiring matches the stated known-good map; these were not electrically probed in this run — they are taken from the user's setup description.
- Feed-forward expectations about quaternion plausibility, report cadence and timestamp semantics remain untested here.

---

## 13. Recommended next step (not performed)

Fix the SPI read framing described in §8.5 (single-CS header+payload read, 512-byte transfer buffer, mock-HAL coverage), then re-run this exact Stage 1 procedure. Until that fix lands, no hardware result downstream of the advertisement can be trusted, including the previous transport-drain PASS.
