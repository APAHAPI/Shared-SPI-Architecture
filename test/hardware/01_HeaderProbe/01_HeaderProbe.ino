// Phase 1 hardware stage 1 + 2: bus, CS and SHTP framing probe (v4).
//
// v1-v3 predate the continuous-CS read fix and are preserved only in the run
// history: they read the 4-byte header and the body in SEPARATE CS assertions.
// The BNO085 re-presents (and drops) the pending transfer when CS is released,
// so those probes measured a re-presented header, and the "552-byte"
// advertisement was a one-bit slip of 0x0114 (276) -> 0x0228 (552). The old
// 552-byte drain PASS is invalid.
//
// v4 uses IHal::beginRead()/read()/endRead(): the header and the body are
// clocked inside ONE CS assertion. The advertisement on this part is 276 bytes
// (4-byte header + 272-byte payload). No logging happens while CS is asserted.
//
// Wiring (Phase 1 baseline):
//   SCK 12, MISO 13, MOSI 11, CS 10, INT 9, RST 8, 3V3 + GND, PS0/PS1 HIGH.
// PS0/PS1 HIGH is enough for reads. Host writes additionally need PS0/WAKE on a
// GPIO; with PS0 tied HIGH the hub only accepts a write while it asserts INT.

#include <Arduino.h>
#include <SPI.h>

#include <BNO085Multi.h>

#define PIN_SCK 12
#define PIN_MISO 13
#define PIN_MOSI 11
#define PIN_CS 10
#define PIN_INT 9
#define PIN_RST 8

#define SPI_HZ 1000000UL
#define CHUNK 240                 // <= BNO085_RX_TRANSFER_BYTES
#define HEAD_SHOWN 96
#define MAX_PACKETS 8

bno085::ManagerConfig managerConfig;
bno085::SensorConfig sensorConfig(PIN_CS, PIN_INT, PIN_RST);
bno085::SpiBusManager bus;
bno085::Esp32SpiHal hal(bus, &SPI, managerConfig, sensorConfig);

static uint8_t chunkBuf[CHUNK];
static uint8_t head[HEAD_SHOWN];
static size_t headLen = 0;

static uint32_t attempts = 0;
static uint32_t packets = 0;
static uint32_t continuationSeen = 0;
static uint32_t idleReads = 0;
static uint32_t transferFailures = 0;
static uint32_t bytesDrained = 0;

static void printHead() {
  Serial.printf("  head(%u): ", static_cast<unsigned>(headLen));
  for (size_t i = 0; i < headLen; ++i) Serial.printf("%02X ", head[i]);
  Serial.print(" |");
  for (size_t i = 0; i < headLen; ++i) {
    const char ch = static_cast<char>(head[i]);
    Serial.print((ch >= 0x20 && ch < 0x7F) ? ch : '.');
  }
  Serial.println('|');
}

// Reads the remaining payload bytes in bounded chunks, staying inside the
// caller's CS assertion. No logging here.
static size_t drainBody(uint16_t length) {
  size_t remaining = length;
  headLen = 0;
  while (remaining > 0) {
    const size_t want = (remaining < CHUNK) ? remaining : CHUNK;
    if (!hal.read(chunkBuf, want)) {
      ++transferFailures;
      return length - remaining;
    }
    bytesDrained += want;
    for (size_t i = 0; i < want && headLen < HEAD_SHOWN; ++i) {
      head[headLen++] = chunkBuf[i];
    }
    remaining -= static_cast<uint16_t>(want);
  }
  return length;
}

static void oneTransfer(uint32_t tEdgeUs) {
  uint8_t header[4] = {0, 0, 0, 0};
  ++attempts;
  if (!hal.beginRead()) {
    ++transferFailures;
    Serial.println("FATAL: read start failed");
    return;
  }
  if (!hal.read(header, sizeof(header))) {
    hal.endRead();
    ++transferFailures;
    Serial.println("FATAL: header read failed");
    return;
  }

  const uint16_t raw = static_cast<uint16_t>(header[0] | (header[1] << 8));
  const bool cont = (raw & 0x8000u) != 0;
  const uint16_t length = static_cast<uint16_t>(raw & 0x7FFFu);

  if (raw == 0xFFFFu) {
    hal.endRead();
    ++idleReads;
    return;
  }
  if (length == 0) {
    hal.endRead();
    Serial.println("HDR len=0 (spurious)");
    return;
  }
  if (length < 4) {
    hal.endRead();
    ++transferFailures;
    Serial.println("HDR len<4 (malformed)");
    return;
  }

  // The header is already consumed, so only length - 4 payload bytes remain in
  // this same CS assertion.
  const size_t want = static_cast<size_t>(length - 4);
  const size_t got = drainBody(static_cast<uint16_t>(want));
  hal.endRead();  // release CS before any logging

  if (cont) ++continuationSeen;
  Serial.printf("HDR len=%u%s ch=%u(%s) seq=%u raw=%02X %02X %02X %02X t=%luus\r\n",
                static_cast<unsigned>(length), cont ? " CONT" : "",
                static_cast<unsigned>(header[2]),
                bno085::shtp::channelName(header[2]),
                static_cast<unsigned>(header[3]), header[0], header[1], header[2],
                header[3], static_cast<unsigned long>(tEdgeUs));

  if (got != want) {
    Serial.printf("  INCOMPLETE: %u/%u payload bytes\r\n", static_cast<unsigned>(got),
                  static_cast<unsigned>(want));
    return;
  }
  ++packets;
  printHead();
}

void setup() {
  Serial.begin(115200);
  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart) < 3000) delay(10);
  delay(100);

  managerConfig.sckPin = PIN_SCK;
  managerConfig.misoPin = PIN_MISO;
  managerConfig.mosiPin = PIN_MOSI;
  managerConfig.spiHz = SPI_HZ;
  managerConfig.busTimeoutUs = 2000;
  managerConfig.log = nullptr;

  Serial.println();
  Serial.println("=== BNO085Multi framing probe v4 ===");
  Serial.printf("pins: SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d clock=%luHz\r\n",
                PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_INT, PIN_RST,
                static_cast<unsigned long>(SPI_HZ));

  const bno085::Status st = hal.begin();
  Serial.printf("hal.begin -> %s\r\n", st.str());
  if (!st.ok()) {
    Serial.println("FATAL: bus/HAL did not open.");
    return;
  }

  Serial.printf("INT before reset=%d\r\n", digitalRead(PIN_INT));
  hal.pulseReset();

  const uint32_t resetAt = micros();
  bool asserted = false;
  while (static_cast<uint32_t>(micros() - resetAt) < 3000000UL) {
    if (digitalRead(PIN_INT) == LOW) {
      asserted = true;
      break;
    }
    delayMicroseconds(100);
  }
  Serial.printf("boot: INT asserted after %lu us (%s)\r\n",
                static_cast<unsigned long>(micros() - resetAt),
                asserted ? "up" : "TIMEOUT 3 s");

  Serial.println("reading complete packets:");
  const uint32_t start = micros();
  while (static_cast<uint32_t>(micros() - start) < 3000000UL) {
    if (packets >= MAX_PACKETS) break;
    if (digitalRead(PIN_INT) != LOW) {
      delayMicroseconds(200);
      continue;
    }
    oneTransfer(static_cast<uint32_t>(micros() - resetAt));
  }

  Serial.println("--- summary ---");
  Serial.printf("attempts=%lu packets=%lu cont=%lu idle=%lu drained=%lu fail=%lu\r\n",
                static_cast<unsigned long>(attempts),
                static_cast<unsigned long>(packets),
                static_cast<unsigned long>(continuationSeen),
                static_cast<unsigned long>(idleReads),
                static_cast<unsigned long>(bytesDrained),
                static_cast<unsigned long>(transferFailures));
  Serial.printf("bus transactions=%lu lockTimeouts=%lu busErrors=%lu\r\n",
                static_cast<unsigned long>(bus.transactions()),
                static_cast<unsigned long>(bus.lockTimeouts()),
                static_cast<unsigned long>(bus.busErrors()));
  Serial.printf("sizeof(Quaternion)=%u sizeof(Diagnostics)=%u\r\n",
                static_cast<unsigned>(sizeof(bno085::Quaternion)),
                static_cast<unsigned>(sizeof(bno085::Diagnostics)));
  Serial.println(packets > 0 ? "RESULT: PASS - complete SHTP packets consumed"
                             : "RESULT: FAIL - no complete packet");
}

void loop() { delay(1000); }
