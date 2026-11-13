# Public API and output protocol (proposal)

Phase 0 deliverable — interface design for review. Nothing is implemented yet; these
signatures are the contract the implementation will be written against.

Namespace: `bno085`. Umbrella header: `<BNO085Multi.h>`.

---

## 1. Design goals

- Changing the sensor count is a **runtime argument**, never a source edit.
- The common case is short: construct a manager, describe the sensors, call `update()`.
- Nothing about a device is hidden behind an implicit "current sensor".
- The API never blocks the sketch: `begin()` and `update()` both return promptly.
- Fixed-point values from the device are preserved alongside convenience floats.

---

## 2. Types

### 2.1 `Report`

The enumerator values **are** the SH-2 report IDs, so the same number identifies the
report on the wire, in configuration and in the API.

```cpp
enum class Report : uint8_t {
  RotationVector            = 0x05,  // default orientation source
  GameRotationVector        = 0x08,
  GeomagneticRotationVector = 0x09,
  Accelerometer             = 0x01,
  LinearAcceleration        = 0x04,
  Gravity                   = 0x06,
  GyroscopeCalibrated       = 0x02,
  GyroscopeUncalibrated     = 0x07,
  MagneticFieldCalibrated   = 0x03,
  MagneticFieldUncalibrated = 0x0F,
  RawAccelerometer          = 0x14,
  RawGyroscope              = 0x15,
  RawMagnetometer           = 0x16,
  StepCounter               = 0x11,
  StabilityClassifier       = 0x13,
  Temperature               = 0x0E,
  Pressure                  = 0x0A,
  AmbientLight              = 0x0B,
  Humidity                  = 0x0C,
};
```

### 2.2 Status enums

```cpp
enum class Accuracy : uint8_t { Unreliable = 0, Low = 1, Medium = 2, High = 3 };

enum class State : uint8_t {
  Unconfigured, Booting, Identifying, Configuring,
  Streaming, Degraded, Recovering, Failed, Disabled
};

struct Status {                    // return type of begin()/recover()
  enum Code : uint8_t {
    Ok = 0, InvalidConfig, DuplicatePin, NoSensors, NoMemory,
    BusUnavailable, OutputUnavailable, AlreadyStarted, SensorFailed
  };
  Code    code = Ok;
  uint8_t sensorId = 0xFF;         // first offending sensor, when applicable
  bool ok() const { return code == Ok; }
  const char* str() const;
};
```

### 2.3 `SensorConfig`

An aggregate, so the brief's original form still compiles:

```cpp
struct SensorConfig {
  int8_t  csPin      = -1;        // required, distinct per sensor
  int8_t  intPin     = -1;        // optional; -1 = polled
  int8_t  resetPin   = -1;        // optional; -1 = software reset only
  uint8_t resetGroup = 0;         // 0 = own line; 1..7 = physically shared NRST

  Report   report      = Report::RotationVector;
  uint32_t intervalUs  = 10000;   // 100 Hz
  bool     allowGrvFallback = false;

  // up to kMaxExtraReports (6) additional reports, off by default
  Report   extra[kMaxExtraReports] = {};
  uint32_t extraIntervalUs[kMaxExtraReports] = {};
  uint8_t  extraCount = 0;

  SensorConfig& enable(Report r, uint32_t intervalUs);
  SensorConfig& disable(Report r);
  bool reportsValid() const;
};
```

```cpp
SensorConfig sensors[] = {
  {10,  9,  8},     // CS, INT, RST
  { 7,  6,  5},
  { 4,  3, -1},     // no reset line
};
manager.begin(sensors, 3);
```

The same thing in named form, which is what the examples use:

```cpp
SensorConfig sensors[3];
sensors[0] = SensorConfig::spi(10, 9, 8);
sensors[1] = SensorConfig::spi(7, 6, 5);
sensors[2] = SensorConfig::spi(4, 3, -1);
sensors[2].intervalUs = 5000;                                  // 200 Hz
sensors[1].enable(Report::CalibratedGyroscope, 5000);          // optional
sensors[1].enable(Report::RawAccelerometer, 10000);
```

Rules enforced at `begin()`:

| Rule | Result on violation |
| --- | --- |
| `1 <= count <= 20` | `Status::NoSensors` / `Status::InvalidConfig` |
| `csPin >= 0` and distinct across sensors | `Status::DuplicatePin` |
| `csPin != intPin`, `csPin != resetPin` | `Status::DuplicatePin` |
| `intervalUs >= 1000` (device floor is report-dependent) | `InvalidConfig` |
| `extraCount <= kMaxExtraReports` | `InvalidConfig` |
| `allowGrvFallback` only if explicitly set | never enabled implicitly |

**Game Rotation Vector is never substituted silently.** `allowGrvFallback` defaults to
`false`; if a device does not support Rotation Vector and the user opted in, every
record carries flag bit 0 (source = GRV) and an `ERROR` frame is emitted once.

### 2.4 `ManagerConfig`

```cpp
enum class OutputMode : uint8_t { Binary, JsonDebug, None };

struct RecoveryPolicy {
  uint8_t  maxAttempts      = 5;
  uint16_t backoffMs[5]     = {250, 500, 1000, 2000, 4000};
  uint16_t bootTimeoutMs    = 300;   // datasheet: 94 ms typical
  uint16_t commandTimeoutMs = 100;   // per SH-2 request
  bool     resetOnBootTimeout = true;
};

struct ManagerConfig {
  SPIClass*  spi        = &SPI;
  int8_t     sckPin     = -1;        // -1 = core default for the board
  int8_t     misoPin    = -1;
  int8_t     mosiPin    = -1;
  uint32_t   spiHz      = 1000000;   // 1 MHz bring-up; 3 MHz max
  Stream*    output     = nullptr;
  OutputMode outputMode = OutputMode::Binary;

  uint32_t   statusIntervalMs    = 1000;  // 0 disables periodic status
  uint32_t   serviceBudgetUs     = 2000;  // max time inside update()
  uint8_t    maxTransfersPerVisit = 3;
  uint32_t   busTimeoutUs        = 2000;
  RecoveryPolicy recovery;
};
```

### 2.5 `Quaternion`

```cpp
struct Quaternion {
  // convenience values (device values / 16384)
  float w = 1, x = 0, y = 0, z = 0;

  // exact device values, unmodified
  int16_t  rawW = 0, rawX = 0, rawY = 0, rawZ = 0;   // Q14
  int16_t  rawAccuracy = 0;                          // Q12, radians

  Accuracy accuracy = Accuracy::Unreliable;          // status bits 1:0
  uint32_t tMeasUs  = 0;    // device measurement time (hub domain)
  uint32_t tHostUs  = 0;    // ESP32 micros() at INT assertion
  uint8_t  seq      = 0;    // per report ID
  uint8_t  epoch    = 0;    // increments on hub reset / timestamp rebase
  Report   source   = Report::RotationVector;
  bool     valid    = false;
};
```

Mapping from the SH-2 report (`i, j, k, real`): `x = i / 16384`, `y = j / 16384`,
`z = k / 16384`, `w = real / 16384`. This mapping is fixed and documented because
quaternion component-order mistakes are silent.

`tMeasUs` is reconstructed exactly as the datasheet specifies: host interrupt
timestamp, minus the timebase reference, plus the report's delay field. `tHostUs` is
the ESP32 clock. They are different clock domains and both are exported; see
`docs/ARCHITECTURE.md` §9.5.

### 2.6 Records and diagnostics

```cpp
enum class RecordKind : uint8_t { Quaternion, Imu, SensorStatus };

struct Record {
  uint8_t    sensorId;
  RecordKind kind;
  uint32_t   tHostUs;
  union {
    Quaternion   rv;
    ImuSample    imu;      // accel/gyro/mag, fixed point + scaled floats
    SensorStatus status;
  };
};

struct Diagnostics {
  State     state;
  uint16_t  lastError;          // ErrorCode
  uint32_t  productId;          // swPartNumber from the device
  uint8_t   fwMajor, fwMinor;
  uint16_t  fwPatch;
  uint32_t  grantedIntervalUs;  // what the device actually accepted

  uint32_t  rxPackets, txPackets, rxBytes;
  uint16_t  spiErrors, busBusy;
  uint16_t  shortPackets, badChannels, badFragments, oversizePayloads;
  uint16_t  sequenceGaps, duplicates;
  uint32_t  missedReports;
  uint16_t  resets, recoveryAttempts;
  uint32_t  lastReportUs, lastErrorUs;
  uint32_t  maxServiceUs;       // worst observed service time
};
```

`missedReports` counts `reportSeq` gaps; `duplicates` counts retransmissions detected
by an unchanged sequence **and** an unchanged `tMeasUs` (the BNO085 retries when the
host exceeds its ~10 ms host-response window).

---

## 3. Manager and sensor API

```cpp
class Sensor {
 public:
  uint8_t index() const;
  State   state() const;
  bool    streaming() const;

  // Latest-arrival access. Returns false if nothing new since the previous call.
  bool latest(Quaternion& out);
  const Quaternion& quaternion() const;      // snapshot, never a live reference

  // Optional reports: copy the most recent sample of one report type.
  bool get(Report r, ImuSample& out) const;

  const Diagnostics& diagnostics() const;

  bool recover();                            // reset + re-init this sensor only
  bool configure(Report r, uint32_t intervalUs);  // runtime change, bounded
  void disable();
};

class Manager {
 public:
  Status begin(const ManagerConfig& cfg, const SensorConfig* sensors, uint8_t count);
  Status begin(const SensorConfig* sensors, uint8_t count);   // default config
  void   update();           // call from loop(); non-blocking, bounded
  void   end();

  uint8_t count() const { return count_; }
  Sensor& sensor(uint8_t i);
  const Sensor& sensor(uint8_t i) const;

  // Aggregated stream (arrival order, sensor-tagged).
  bool next(Record& out);
  void onRecord(void (*cb)(void* user, const Record&), void* user = nullptr);

  bool busy() const;                          // any sensor not Streaming
  void setOutput(Stream* s);
  void setOutputMode(OutputMode m);
  const ManagerDiagnostics& diagnostics() const;
};
```

### 3.1 Usage contract

1. `begin()` validates, allocates, configures the bus and starts the init state
   machines. It returns in bounded time; sensors reach `Streaming` asynchronously.
2. `update()` must be called repeatedly (every `loop()`); a call takes at most
   `serviceBudgetUs`. Calling it less often than every ~5 ms risks the device's 10 ms
   host-response timeout at high rates.
3. `update()` is **not** thread-safe with itself. It may run in one task while
   `sensor(i).quaternion()` / `next()` are read from another **only** with the
   application's own synchronisation, because those accessors touch the queue.
   Recommended: call everything from one task.
4. `latest()` is edge-triggered (consumes the "new" flag); `quaternion()` is not.
5. `recover()` may be called from any state; it does not disturb other sensors unless
   resets are physically shared.

### 3.2 Sketch shape

```cpp
#include <BNO085Multi.h>

bno085::Manager imu;
bno085::SensorConfig sensors[] = { {10, 9, 8}, {7, 6, 5}, {4, 3, -1} };

void setup() {
  Serial.begin(115200);
  bno085::ManagerConfig cfg;
  cfg.spiHz  = 3000000;
  cfg.output = &Serial;
  auto st = imu.begin(cfg, sensors, 3);
  if (!st.ok()) {
    Serial.printf("begin failed: %s (sensor %u)\n", st.str(), st.sensorId);
  }
}

void loop() {
  imu.update();
  bno085::Record r;
  while (imu.next(r)) { /* binary mode already wrote the frame */ }
}
```

---

## 4. Binary output protocol

### 4.1 Frame

Little-endian, packed, no padding.

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | sync 0 = `0xA5` |
| 1 | 1 | sync 1 = `0x5A` |
| 2 | 1 | protocol version = `0x01` |
| 3 | 1 | packet type |
| 4 | 2 | payload length `N` (0..64) |
| 6 | 4 | packet sequence (u32, +1 per frame, wraps) |
| 8 | N | payload |
| 8+N | 2 | CRC-16 |

- **CRC:** CRC-16/CCITT-FALSE over bytes `[2, 8+N)` (version through last payload
  byte; sync bytes excluded so the receiver can resync). Init `0xFFFF`, polynomial
  `0x1021`, no reflection, xor-out `0`. Check value for `"123456789"` is `0x29B1`.
- **Maximum frame:** 8 + 64 + 2 = 74 bytes. A receiver that sees `N > 64` must discard
  to the next `A5 5A`.
- **Resynchronisation:** scan for `A5 5A`, then require a valid version, a legal type,
  `N <= 64`, and a matching CRC before accepting a frame.

### 4.2 Packet types

| Type | Name | Payload | Emitted |
| --- | --- | --- | --- |
| `0x01` | `QUATERNION` | 24 B | Every quaternion report (default stream) |
| `0x02` | `IMU_SAMPLE` | 28 B | Only for enabled raw/calibrated reports |
| `0x10` | `SENSOR_STATUS` | 24 B | Periodically (`statusIntervalMs`) and on state change |
| `0x11` | `MANAGER_DIAGNOSTICS` | 20 B | Periodically |
| `0x12` | `INIT_RESULT` | 16 B | Once per sensor when init completes or fails |
| `0x20` | `ERROR` | 8 B | On any error escalation |
| `0x7F` | `HELLO` | 10 B | Once at startup, before any other frame |

### 4.3 `QUATERNION` (0x01), 24 bytes

| Off | Type | Field | Notes |
| --- | --- | --- | --- |
| 0 | u8 | sensorId | 0..19 |
| 1 | u8 | flags | bit0 source=GRV, bit1 first-after-recovery, bit2 GRV fallback active, bit3 accuracy valid |
| 2 | u8 | accuracy | 0 unreliable … 3 high |
| 3 | u8 | reportSeq | device sequence for this report ID |
| 4 | u8 | epoch | increments on hub reset / timestamp rebase |
| 5 | u8 | qFormat | 14 (Q14 scale, `1/16384`) |
| 6 | i16 | qx (i) | raw Q14 |
| 8 | i16 | qy (j) | |
| 10 | i16 | qz (k) | |
| 12 | i16 | qw (real) | |
| 14 | i16 | radAccuracy | Q12, `1/4096` rad |
| 16 | u32 | tMeasUs | device measurement time, µs |
| 20 | u32 | tHostUs | ESP32 `micros()` at INT |

Frame on the wire: `A5 5A 01 01 18 00 <seq:4> <24 B> <crc:2>` = 34 bytes.

The device's quaternion is unit-norm; the receiver normalises only if it wants to
compensate for Q14 rounding (worst-case norm error ≈ 1/16384 per component).

### 4.4 `IMU_SAMPLE` (0x02), 28 bytes

| Off | Type | Field |
| --- | --- | --- |
| 0 | u8 | sensorId |
| 1 | u8 | reportId (0x01 accel, 0x02 gyro, 0x03 mag, 0x14/0x15/0x16 raw, …) |
| 2 | u8 | accuracy |
| 3 | u8 | reportSeq |
| 4 | u8 | epoch |
| 5 | u8 | scaleExponent (value = raw / 2^exp) |
| 6 | i16 | x |
| 8 | i16 | y |
| 10 | i16 | z |
| 12 | i32 | reserved/sensor-specific (e.g. temperature or bias) |
| 16 | u32 | tMeasUs |
| 20 | u32 | tHostUs |
| 24 | u32 | reserved |

Raw/calibrated reports have different scales; `scaleExponent` makes each frame
self-describing instead of relying on a fixed table per type.

### 4.5 `SENSOR_STATUS` (0x10), 24 bytes

| Off | Type | Field |
| --- | --- | --- |
| 0 | u8 | sensorId |
| 1 | u8 | state (`State` enum) |
| 2 | u8 | accuracy |
| 3 | u8 | lastError (`ErrorCode`) |
| 4 | u32 | missedReports |
| 8 | u32 | rxPackets |
| 12 | u16 | sequenceGaps |
| 14 | u16 | duplicates |
| 16 | u16 | spiErrors |
| 18 | u16 | resets |
| 20 | u32 | lastReportUs |

### 4.6 `MANAGER_DIAGNOSTICS` (0x11), 20 bytes

| Off | Type | Field |
| --- | --- | --- |
| 0 | u8 | sensorCount |
| 1 | u8 | streamingCount |
| 2 | u16 | busBusy events |
| 4 | u32 | framesOut |
| 8 | u32 | queueDrops |
| 12 | u32 | uptimeMs |
| 16 | u32 | maxUpdateUs (worst `update()` duration) |

### 4.7 `INIT_RESULT` (0x12), 16 bytes

| Off | Type | Field |
| --- | --- | --- |
| 0 | u8 | sensorId |
| 1 | u8 | result (0 = ok, else `ErrorCode`) |
| 2 | u8 | stage reached |
| 3 | u8 | fwMajor |
| 4 | u8 | fwMinor |
| 5 | u16 | fwPatch |
| 7 | u32 | productId (swPartNumber) |
| 11 | u32 | grantedIntervalUs |
| 15 | u8 | reserved |

### 4.8 `ERROR` (0x20), 8 bytes

| Off | Type | Field |
| --- | --- | --- |
| 0 | u8 | sensorId (0xFF = manager) |
| 1 | u8 | error code |
| 2 | u8 | context (state/stage) |
| 3 | u8 | reserved |
| 4 | u32 | tHostUs |

### 4.9 `HELLO` (0x7F), 10 bytes

| Off | Type | Field |
| --- | --- | --- |
| 0 | u8 | protocol version |
| 1 | u8 | sensorCount |
| 2 | u16 | capabilities: bit0 binary, bit1 json debug, bit2 GRV available, bit3 reception timestamps |
| 4 | u32 | tBootUs |
| 8 | u16 | library version (major<<8 | minor) |

### 4.10 `ErrorCode`

```
0  None
1  InvalidConfig
2  DuplicatePin
3  SpiRead
4  SpiWrite
5  ShtpBadFrame
6  ShtpBadChannel
7  ShtpSequenceGap
8  Sh2Timeout
9  Sh2BadResponse
10 ProductIdMismatch
11 BootTimeout
12 NoReports
13 BusTimeout
14 QueueOverflow
15 OutputOverflow
16 GrvFallbackUsed
17 UnsupportedReport
```

Values are stable; new codes append, never renumber.

---

## 5. JSON debug mode (optional)

`setOutputMode(OutputMode::JsonDebug)` emits one line per record at 115200 baud:

```json
{"v":1,"s":0,"k":"rv","q":[0.250000,-0.125000,0.000000,0.968750],
 "acc":3,"seq":42,"ep":0,"tm":50000,"th":50312}
```

Intended for bring-up and field diagnostics only. It is roughly 10× larger than the
binary frame and is never used for the mocap stream. `OutputMode::None` disables the
sink entirely (library used only through the C++ API).

---

## 6. Examples (to be created in Phase 1–3)

### 6.1 `01_SingleSensor`

One sensor, binary stream, prints the decoded quaternion on a second serial port or
the same port behind a compile switch. Demonstrates the minimum viable path and is
the Phase 1 acceptance sketch.

```cpp
#include <BNO085Multi.h>
bno085::Manager imu;
bno085::SensorConfig s[] = { {10, 9, 8} };

void setup() {
  Serial.begin(115200);
  bno085::ManagerConfig cfg; cfg.spiHz = 1000000; cfg.output = &Serial;
  if (!imu.begin(cfg, s, 1).ok()) { for (;;) delay(1000); }
}
void loop() { imu.update(); }
```

### 6.2 `02_DualSensor`

Two sensors, shared SPI, independent CS/INT/RST. Prints both quaternions side by
side so cross-contamination is visible by inspection (rotate one board, only its row
must change). Phase 2 acceptance sketch.

### 6.3 `03_MultiSensorConfigurable`

Count, pins and per-sensor report configuration come from a single table at the top of
the sketch. Changing 3 → 20 edits that table only; the sketch immediately streams 20
sensors, emits per-sensor status, and exposes `recover(i)` on a serial command.
Phase 5 acceptance sketch.

### 6.4 `extras/host/bno085_stream.py`

Reference PC parser: resynchronises on `A5 5A`, validates CRC, decodes packet types,
and prints/streams quaternions with per-sensor gap statistics. Used to prove the
protocol end to end and to keep the spec honest.
