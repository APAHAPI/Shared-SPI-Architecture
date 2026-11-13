# Upstream audit — CEVA SH-2 / SHTP and the Adafruit driver

Phase 0 deliverable. Read-only inspection of public sources; nothing was built or run.

Sources inspected:

| Source | Revision inspected |
| --- | --- |
| `https://github.com/ceva-dsp/sh2` (`sh2.c`, `sh2.h`, `shtp.c`, `shtp.h`, `sh2_hal.h`, `sh2_SensorValue.c`, `sh2_util.c`, `euler.c`) | `main`, version 1.4.0 release notes |
| `https://github.com/adafruit/Adafruit_BNO08x` (`src/Adafruit_BNO08x.cpp`) | `master` |
| BNO08X datasheet 1000-3927 v1.8 (SparkFun DS-14686) | §1.2.4, §1.3.x, §5.2, §6.5 |
| SH-2 API User's Guide (Hillcrest, 2017) | HAL and API conventions |

License: CEVA's files are Apache-2.0; reuse is permitted with headers retained and
changes stated. Adafruit's driver is BSD; it was inspected for reference only, not
copied.

---

## 1. Executive summary

1. **CEVA's SH-2 is not multi-instance safe.** It is a singleton by construction:
   one global `sh2_t _sh2`, one shared async-event scratch object, file-static
   timestamp rollover state, and every public API hard-codes `&_sh2`. SHTP below it
   is also hard-limited to one device (`SHTP_INSTANCES (1)`).
2. **It contains two unbounded loops**, both reachable from a normal
   `begin_SPI()` sequence when a device does not answer. This is the most probable
   cause of the reported hang. See §5.
3. **Adafruit's driver cannot be made multi-instance by parameterising pins.** Its
   transport objects and callbacks are file-static and the HAL callbacks ignore the
   `self` pointer they are given, so every instance shares one SPI device, one INT
   pin, one reset pin and one output slot.
4. **One component is worth reusing verbatim:** `sh2_SensorValue.c` is stateless
   (its only global is `const float scaleRadToDeg`) and takes the destination struct
   as a parameter. It is a clean, audited report decoder.
5. **Recommendation:** own the SHTP transport and a minimal SH-2 control layer,
   vendor the stateless decoder, and keep the full fork patch set (§6) documented in
   case the fork route is preferred at review.

---

## 2. CEVA SHTP (`shtp.c`, `shtp.h`)

### 2.1 Global / static state

| Symbol | Declaration | Effect on multi-instance use |
| --- | --- | --- |
| `SHTP_INSTANCES` | `#define SHTP_INSTANCES (1)` | Compile-time cap of one device per process. |
| `instances` | `static shtp_t instances[SHTP_INSTANCES];` | All SHTP state lives in that one-element static array. |
| `shtp_initialized` | `static bool shtp_initialized = false;` | One-time module init flag; harmless in itself, symptomatic of the singleton design. |
| `getInstance()` | returns `&instances[n]` where `pHal == 0`, else `0` | Second `shtp_open()` returns `0` → `sh2_open()` returns `SH2_ERR`. |

Per-instance fields (good news: `shtp_t` itself is a proper context struct):

```c
uint8_t  outTransfer[SH2_HAL_MAX_TRANSFER_OUT];   // 128
uint16_t inRemaining; uint8_t inChan;
uint8_t  inPayload[SH2_HAL_MAX_PAYLOAD_IN];       // 1024
uint16_t inCursor; uint32_t inTimestamp;
uint8_t  inTransfer[SH2_HAL_MAX_TRANSFER_IN];     // 1024
shtp_Channel_t chan[SHTP_MAX_CHANS];              // nextOutSeq, nextInSeq, cb, cookie
uint32_t rxBadChan, rxShortFragments, ... ;       // stats
```

So the state is *struct-shaped* but *allocated as a static singleton*. That is the
whole problem: the struct is already suitable to be owned by a sensor object.

### 2.2 Behavioural hazards

| Symbol | Issue |
| --- | --- |
| `txProcess()` | `while (status == 0) { shtp_service(pShtp); status = pHal->write(...); }` — **unbounded retry.** `sh2_hal.h` explicitly permits a HAL to return 0 when the device is not ready, so this loop can spin forever if the device never asserts INT. No deadline, no counter, no escape. |
| `shtp_service()` | Calls `pHal->read()` unconditionally and trusts the HAL to be non-blocking. SHTP has no read deadline of its own; a blocking HAL (as in Adafruit) turns every service call into a potential 500 ms stall. |
| `shtp_open()` | Calls `pHal->open()` before storing the HAL pointer, so an `open()` failure returns a bare `0` with no error detail and no instance; `sh2_open()` can only report a generic `SH2_ERR`. |
| `rxAssemble()` | Fragment/sequence validation is good (short fragments, bad channel, interrupted payload, oversize payload are all counted and reported), but `chan` is indexed before the range check in the sequence comparison: `pShtp->chan[chan].nextInSeq` is evaluated before `if (chan >= SHTP_MAX_CHANS)`. A malformed channel byte indexes out of bounds. Must be fixed in any fork. |
| `shtp_listenChan()` | Rejects channel 0, so the advertisement cannot be observed through the channel API. |
| Buffer sizes | 1024 B in + 1024 B in-payload + 128 B out ≈ 2.2 KB per device; ×20 ≈ 45 KB of static buffers. Workable on an S3 but wasteful for 14-byte quaternion reports. |

---

## 3. CEVA SH-2 (`sh2.c`, `sh2.h`)

### 3.1 Global / static state

| Symbol | Declaration | Effect |
| --- | --- | --- |
| `_sh2` | `sh2_t _sh2;` (file scope, not `static`) | The single device session. Visible to the linker. |
| `sh2AsyncEvent` | `static sh2_AsyncEvent_t sh2AsyncEvent;` | One scratch object for all async events. `sensorhubControlHdlr()` fills it with `SH2_GET_FEATURE_RESP` data and then invokes the client callback with a pointer to it. With two devices, a caller waiting for device A's feature response can receive device B's. |
| `lastHostInt`, `rollovers` | `static uint32_t lastHostInt = 0; static uint32_t rollovers = 0;` **inside `touSTimestamp()`** | Shared 64-bit timestamp extrapolation. Interleaved reports from two hubs make each stream look like it wrapped, injecting ±2³² µs jumps into timestamps — silently corrupting the primary mocap output. |
| `sh2ReportLens[]` | `static const sh2_ReportLen_t[]` | Read-only; safe to share. |
| `sensorToRecordMap[]` | `static const struct {...}[]` | Read-only; safe to share. |
| `pSh2 = &_sh2` | the first statement of **every** public API function | No public entry point accepts a session handle: `sh2_open`, `sh2_service`, `sh2_setSensorConfig`, `sh2_getProdIds`, … all operate on the global. |

`sh2_t` itself is a real context struct (HAL pointer, SHTP pointer, pending op,
`opData` union, callbacks, `frsData[72]`, counters) — ≈ 400 B. It was designed to be
per-session, then instantiated once.

### 3.2 The unbounded operation loop

```c
static int opProcess(sh2_t *pSh2, const sh2_Op_t *pOp)
{
    ...
    while ((pSh2->pOp != 0) &&
           ((pOp->timeout_us == 0) ||
            ((now_us - start_us) < pOp->timeout_us))) {
        shtp_service(pSh2->pShtp);
        now_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    }
    ...
}
```

`timeout_us == 0` means **wait forever**. Only `sendWheelOp` sets a timeout
(`.timeout_us = 5000000`). The operations that `begin()` actually uses —
`getProdIdOp`, `setSensorConfigOp`, `getSensorConfigOp` — leave it at the zero
default. So: no response ⇒ endless loop ⇒ the sketch stops with no output. This is a
library bug reached through a perfectly normal call sequence, not a hardware fault.

`sh2_open()` itself is bounded — it waits for the advertisement in a loop bounded by
`ADVERT_TIMEOUT_US (200000)` — **but it returns `SH2_OK` even when the timeout
expires** and no advertisement ever arrived (`// No errors. return SH2_OK;`). The
caller therefore believes the device is open, proceeds to `sh2_getProdIds()`, and
enters the unbounded loop above. The false success is what makes the failure look like
a hang inside `begin_SPI()` rather than an open failure.

### 3.3 Other behavioural notes

- `opStart()` refuses a second operation with `SH2_ERR_OP_IN_PROGRESS`. Only one
  request in flight per device — consistent with the SH-2 protocol; our async design
  keeps that invariant.
- `sh2_Service()` is a global function with no handle and no return value; errors are
  only visible through the callback.
- `opData` holds **caller pointers** (`getProdIds.pProdIds`, `getFrS.pData`,
  `getSensorConfig.pConfig`). They are valid only while the call blocks. An async
  design must copy the request into instance storage instead.
- `sensorhubInputHdlr()` computes `delay` as `((report[2] & 0xFC) << 6) + report[3]`
  and the timestamp as `hostInt + (referenceDelta + delay) * 100`, matching the
  datasheet's "T − 12 ms + delay" model. This is the correct model to reimplement.
- `sensorhubInputGyroRvHdlr()` treats channel-5 payloads as 14-byte
  `SH2_GYRO_INTEGRATED_RV` records with no report ID. Relevant only if that report is
  ever enabled; v1 does not.
- `getReportLen()` returns 0 for unknown IDs, and the caller discards the remainder of
  the payload. Reasonable; we do the same, with a counter.

### 3.4 Components that are safe to reuse

| File | Verdict | Reason |
| --- | --- | --- |
| `sh2_SensorValue.c/.h` | **Reuse** | Stateless; only global is `const float scaleRadToDeg`. `sh2_decodeSensorEvent(value, event)` writes into a caller-supplied struct. |
| `sh2_util.c/.h` | Reuse (trivial) | `read16/read32/readu16/…`; pure functions. |
| `sh2_err.h` | Reuse | Error codes; no state. |
| `euler.c/.h` | Not needed | Euler conversion is not wanted: the ESP32 must not perform orientation maths. |
| `sh2.c`, `shtp.c` | Replace (or fork with §6 patches) | Singleton state, unbounded loops, blocking op model. |

---

## 4. Adafruit BNO08x driver

### 4.1 File-static state (`src/Adafruit_BNO08x.cpp`)

```c
static Adafruit_SPIDevice *spi_dev = NULL;
static int8_t _int_pin, _reset_pin;
static Adafruit_I2CDevice *i2c_dev = NULL;
static HardwareSerial *uart_dev = NULL;
static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;
```

| Symbol | Effect |
| --- | --- |
| `spi_dev` | `begin_SPI()` **deletes** the previous device and replaces it. A second instance silently destroys the first instance's transport. |
| `_int_pin`, `_reset_pin` | Written from the constructor and `begin_SPI()`. A second instance overwrites the pins used by the first. |
| `_sensor_value` | Set only inside `getSensorEvent()`. `sensorHandler()` dereferences it unconditionally, so any report arriving before the first `getSensorEvent()` call writes through a null pointer. |
| `_reset_occurred` | One flag for all devices; `wasReset()` on device A can consume device B's reset. |

`sh2_Hal_t _HAL` **is** a member of each object, which shows the intended design — but
none of the HAL callbacks use `self`:

```c
static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    if (!spihal_wait_for_int()) return 0;          // uses static _int_pin
    if (!spi_dev->read(pBuffer, 4, 0x00)) return 0; // uses static spi_dev
    ...
}
```

So per-object HAL structs all point at the same global transport. Instance isolation
is an illusion.

### 4.2 Blocking behaviour

| Function | Blocking |
| --- | --- |
| `spihal_wait_for_int()` | `for (i=0;i<500;i++) { if (!digitalRead(_int_pin)) return true; delay(1); }` then **`hal_hardwareReset()`** and return false |
| `spihal_read()` | Calls it **twice** per packet → up to 1 s, plus a reset on each timeout |
| `spihal_write()` | Calls it once → up to 500 ms, plus a reset on timeout |
| `hardwareReset()` | 10 ms high, 10 ms low, 10 ms high = 30 ms blocking |

Consequences:

- Service calls can stall ~1 s per sensor, per packet. With 20 sensors this alone
  makes the required service latency (≤ ~10 ms, datasheet §1.2.4.1) impossible.
- A timeout **inside a read** triggers a **hardware reset** using the shared static
  reset pin. With a shared NRST line, reading a momentarily busy sensor resets the
  entire rig. With independent lines, it resets whichever sensor the last
  `begin_SPI()` configured.

### 4.3 What it gets right (and we keep)

- `begin_SPI()` uses `SPI_MODE3`, MSB-first, **1 MHz** — the proven baseline clock.
- Read shape: 4-byte header, mask bit 15, then read `packet_size` bytes including the
  header. Matches the datasheet.
- Reset asserted low then released before `sh2_open`, and the boot wait happens in
  `open()`.
- Report enable defaults clear every optional flag (`changeSensitivity`, `wakeup`,
  `batchInterval`, `sensorSpecific`), which is the right baseline for a mocap stream.

---

## 5. Probable cause of the reported `begin_SPI()` hang

Stated as a hypothesis to confirm in Phase 1, with the exact mechanisms it rests on:

1. A second `begin_SPI()` deletes the static `spi_dev` and overwrites `_int_pin` /
   `_reset_pin`. The first object's HAL now drives the wrong CS and waits on the wrong
   INT line.
2. `sh2_open()` → `shtp_open()` → `getInstance()` fails on the second device because
   `SHTP_INSTANCES == 1`. `shtp_open` returns 0, `sh2_open` returns `SH2_ERR`.
3. If the sequence instead reaches `sh2_getProdIds()`, `opProcess()` is entered with
   `timeout_us == 0`. The product-ID request is written to whichever device the stale
   `spi_dev` selects; no response ever arrives; the `while` loop never exits.
4. The same shape occurs in `shtp_send()`: `spihal_write()` returns 0 while INT is
   never asserted, and `txProcess()` retries forever.
5. `spihal_open()`/`spihal_read()` additionally reset the (shared) static reset pin
   every 500 ms, so the device is repeatedly knocked back to boot while the host is
   waiting for a response that can never come.

Only one sensor physically connected does not help: the failure is in host-side state,
not in the device. This matches "hangs inside `begin_SPI()` even with one sensor".

**Confirmation plan (Phase 1, before writing the new driver):** instrument a
two-object sketch with a logic analyser on both CS lines and the INT line, log
`shtp_open()`'s return value and the `pOp`/`timeout_us` values, and confirm no
response arrives for the request. This is a 30-minute experiment and it decides
whether the new design needs anything beyond what is already in §6.

---

## 6. Required changes if upstream SH-2 is forked

Documented as required by the brief. These are the minimum edits for a clean
multi-instance fork; the recommended plan (§7) avoids them, but the list is the
fallback and the reference for anyone comparing the two designs.

| # | Change | Files | Notes |
| --- | --- | --- | --- |
| P1 | Add a session handle: `int sh2_openInstance(sh2_t *pSh2, sh2_Hal_t *, cb, cookie)`, and add `sh2_t *` as the first parameter to every public function. Delete `_sh2`. | `sh2.c`, `sh2.h` | Mechanical but touches ~35 signatures; callers must change. |
| P2 | Move `sh2AsyncEvent` into `sh2_t`. | `sh2.c` | Removes cross-device config-response contamination. |
| P3 | `shtpEventCallback()` must use its `cookie` instead of `&_sh2`. | `sh2.c` | The cookie is already passed; it is simply ignored. |
| P4 | Move `lastHostInt` / `rollovers` from function-static into `sh2_t`; add an explicit reset/epoch entry point for hub resets. | `sh2.c` | Otherwise timestamps jump by 2³² µs when streams interleave. |
| P5 | Replace `static shtp_t instances[1]` with a caller-provided context: `shtp_openInstance(shtp_t *, sh2_Hal_t *)`; keep `shtp_open()` as a deprecated wrapper. | `shtp.c`, `shtp.h` | The struct is already suitable; only allocation changes. |
| P6 | Bound `txProcess()`'s retry loop (deadline + attempt counter, return `SH2_ERR_TIMEOUT`). | `shtp.c` | Removes infinite loop #1. |
| P7 | Require a non-zero `timeout_us` for every `sh2_Op_t`, or convert `opProcess()` into a pumped state machine (`sh2_opBegin` / `sh2_opPoll`). | `sh2.c` | Removes infinite loop #2; the pump variant is what a non-blocking manager needs. |
| P8 | Range-check the channel before indexing `chan[chan]` in `rxAssemble()`; reject `0xFFFF` length and `chan == 0` writes. | `shtp.c` | One out-of-bounds read and one reserved-value case. |
| P9 | Right-size buffers: `SH2_HAL_MAX_TRANSFER_IN 256`, `MAX_PAYLOAD_IN 192`, `MAX_TRANSFER_OUT 64`; make them overridable macros. | `sh2_hal.h` | 2.2 KB → ~0.5 KB per device. |
| P10 | Expose per-instance counters (`shtp` stats, `unknownReportIds`, `emptyPayloads`) through an accessor. | `sh2.c`, `sh2.h` | Diagnostics requirement. |
| P11 | Retain Apache-2.0 headers in every modified file, add a `NOTICE` describing the modifications. | all | License compliance. |

Estimated effort: 1–2 days for P1–P11 plus regression testing against a single device,
and every future upstream change becomes a rebase. That maintenance cost is the main
reason the recommendation below does not fork.

---

## 7. Recommendation for review

| Layer | Decision | Rationale |
| --- | --- | --- |
| SPI HAL / bus | **Own implementation** | Must be per-sensor CS with one shared bus lock and bounded waits; upstream has no bus concept at all. |
| SHTP transport | **Own implementation** (≈350 lines) | The state struct is the right shape, but the singleton allocation, unbounded TX retry and HAL-blocking read contract are exactly what must not exist here. Writing it gives per-packet deadlines and per-sensor counters. |
| SH-2 control (prod ID, set/get feature) | **Own minimal implementation** (≈400 lines) | Only three request/response pairs are needed. Requests are matched by report ID and command sequence, one in flight per sensor, 100 ms deadline — non-blocking by construction. |
| SH-2 report decode | **Vendor `sh2_SensorValue.c` unchanged** | Stateless, audited, Apache-2.0, covers every optional report. Used as the typed-decode path; the hot path has a small fixed-point decoder of its own. |
| FRS / tare / calibration / DFU | **Out of scope v1** | Not needed for streaming; each would add a blocking multi-step operation. |
| Adafruit driver | **Reference only** | Not copied, not wrapped, not patched. |

If review prefers the fork instead, §6 is the complete work list and P1–P4 are the
ones that actually break multi-instance behaviour.
