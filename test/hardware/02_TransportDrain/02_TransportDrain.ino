// Phase 1 transport framing validation against a physical BNO085.
//
// Drives bno085::ShtpTransport::service() directly after a reset and verifies the
// corrected continuous-CS read: the 276-byte channel-0 advertisement (272-byte
// payload) is delivered as one Ok packet, with no oversized transfer, no
// fragments, no bus errors, one bus transaction per packet, and the device idle
// afterwards.
//
// The old expectation that the advertisement is 552 B and must be drained is
// invalid: 552 was a one-bit slip of 0x0114 caused by releasing CS between the
// header and the body (and, on the first boot, by applying SPI_MODE3 after CS
// was already low). The drain path is covered by the host suite.
//
// This is NOT stage 3: no SH-2 traffic, no product ID, no feature configuration.
//
// Wiring (Phase 1 baseline):
//   SCK 12, MISO 13, MOSI 11, CS 10, INT 9, RST 8, 3V3 + GND, PS0/PS1 HIGH.

#include <Arduino.h>
#include <SPI.h>

#include <BNO085Multi.h>
#include "shtp/ShtpTransport.h"

#define PIN_SCK 12
#define PIN_MISO 13
#define PIN_MOSI 11
#define PIN_CS 10
#define PIN_INT 9
#define PIN_RST 8

#define SPI_HZ 1000000UL
#define SERVICE_LIMIT 64
#define SERVICE_WINDOW_US 3000000UL

bno085::ManagerConfig managerConfig;
bno085::SensorConfig sensorConfig(PIN_CS, PIN_INT, PIN_RST);
bno085::SpiBusManager bus;
bno085::Esp32SpiHal hal(bus, &SPI, managerConfig, sensorConfig);
bno085::ShtpTransport transport;

static uint32_t handled = 0;
static uint8_t firstHandledChannel = 0xFF;
static uint16_t firstHandledLength = 0;
static uint8_t lastHandledChannel = 0xFF;
static uint16_t lastHandledLength = 0;

static const char* resultName(bno085::ShtpResult r) {
  switch (r) {
    case bno085::ShtpResult::NoData:    return "NoData";
    case bno085::ShtpResult::Ok:        return "Ok";
    case bno085::ShtpResult::Fragment:  return "Fragment";
    case bno085::ShtpResult::Spurious:  return "Spurious";
    case bno085::ShtpResult::Malformed: return "Malformed";
    case bno085::ShtpResult::Oversize:  return "Oversize";
    case bno085::ShtpResult::BusError:  return "BusError";
  }
  return "?";
}

static void onPayload(void*, uint8_t channel, const uint8_t* payload, uint16_t len,
                      uint32_t tUs) {
  if (handled == 0) {
    firstHandledChannel = channel;
    firstHandledLength = len;
  }
  ++handled;
  lastHandledChannel = channel;
  lastHandledLength = len;
  Serial.printf("  payload ch=%u(%s) len=%u t=%luus head=%02X %02X\r\n",
                static_cast<unsigned>(channel), bno085::shtp::channelName(channel),
                static_cast<unsigned>(len), static_cast<unsigned long>(tUs),
                len > 0 ? payload[0] : 0u, len > 1 ? payload[1] : 0u);
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
  Serial.println("=== BNO085Multi transport framing validation ===");
  Serial.printf("pins: SCK=%d MISO=%d MOSI=%d CS=%d INT=%d RST=%d clock=%luHz\r\n",
                PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PIN_INT, PIN_RST,
                static_cast<unsigned long>(SPI_HZ));

  const bno085::Status st = hal.begin();
  Serial.printf("hal.begin -> %s\r\n", st.str());
  if (!st.ok()) {
    Serial.println("RESULT: FAIL - bus/HAL did not open");
    return;
  }

  transport.attach(&hal, nullptr, 0);
  transport.setHandler(onPayload, nullptr);

  hal.pulseReset();
  const uint32_t resetAt = micros();

  bool asserted = false;
  while (static_cast<uint32_t>(micros() - resetAt) < SERVICE_WINDOW_US) {
    if (digitalRead(PIN_INT) == LOW) {
      asserted = true;
      break;
    }
    delayMicroseconds(100);
  }
  Serial.printf("boot: INT asserted after %lu us (%s)\r\n",
                static_cast<unsigned long>(micros() - resetAt),
                asserted ? "up" : "TIMEOUT 3 s");
  if (!asserted) {
    Serial.println("RESULT: FAIL - device never asserted INT after reset");
    return;
  }

  Serial.println("servicing while INT asserted:");
  uint32_t oversizeCalls = 0;
  uint32_t okCalls = 0;
  uint32_t otherCalls = 0;
  uint32_t calls = 0;
  const uint32_t start = micros();

  while (static_cast<uint32_t>(micros() - start) < SERVICE_WINDOW_US) {
    if (calls >= SERVICE_LIMIT) {
      Serial.println("  (service limit reached)");
      break;
    }
    if (digitalRead(PIN_INT) != LOW) {
      Serial.println("  INT deasserted: device idle");
      break;
    }
    const bno085::ShtpResult r =
        transport.service(static_cast<uint32_t>(micros() - resetAt));
    ++calls;
    if (r == bno085::ShtpResult::Oversize) ++oversizeCalls;
    else if (r == bno085::ShtpResult::Ok) ++okCalls;
    else ++otherCalls;
    Serial.printf("  service[%lu] -> %s\r\n", static_cast<unsigned long>(calls),
                  resultName(r));
  }

  // A drained transfer leaves nothing behind, so this must report NoData.
  const bno085::ShtpResult finalResult = transport.service(micros());
  const bool idle = (finalResult == bno085::ShtpResult::NoData);

  const bno085::ShtpStats& s = transport.stats();
  Serial.println("--- summary ---");
  Serial.printf("service calls=%lu oversize=%lu ok=%lu other=%lu\r\n",
                static_cast<unsigned long>(calls),
                static_cast<unsigned long>(oversizeCalls),
                static_cast<unsigned long>(okCalls),
                static_cast<unsigned long>(otherCalls));
  Serial.printf("final service -> %s (%s)\r\n", resultName(finalResult),
                idle ? "idle, no stranded tail" : "TAIL PRESENT");
  Serial.printf("stats: packets=%lu bytes=%lu oversizePayloads=%u badFragments=%u "
                "shortFragments=%u badChannels=%u seqGaps=%u rxErrors=%lu\r\n",
                static_cast<unsigned long>(s.packets),
                static_cast<unsigned long>(s.bytes),
                static_cast<unsigned>(s.oversizePayloads),
                static_cast<unsigned>(s.badFragments),
                static_cast<unsigned>(s.shortFragments),
                static_cast<unsigned>(s.badChannels),
                static_cast<unsigned>(s.sequenceGaps),
                static_cast<unsigned long>(s.rxErrors));
  Serial.printf("payloads=%lu firstCh=%u firstLen=%u lastCh=%u lastLen=%u\r\n",
                static_cast<unsigned long>(handled),
                static_cast<unsigned>(firstHandledChannel),
                static_cast<unsigned>(firstHandledLength),
                static_cast<unsigned>(lastHandledChannel),
                static_cast<unsigned>(lastHandledLength));
  Serial.printf("bus transactions=%lu lockTimeouts=%lu busErrors=%lu\r\n",
                static_cast<unsigned long>(bus.transactions()),
                static_cast<unsigned long>(bus.lockTimeouts()),
                static_cast<unsigned long>(bus.busErrors()));

  // The corrected read presents the 276-byte advertisement as an ordinary
  // packet: one Ok, channel 0, 272-byte payload. Nothing is oversized.
  const bool framed = (okCalls >= 1 && firstHandledChannel == 0 &&
                       firstHandledLength == 272);
  const bool noOversize = (s.oversizePayloads == 0 && oversizeCalls == 0);
  const bool clean = (s.badFragments == 0 && s.shortFragments == 0 &&
                      s.badChannels == 0 && s.rxErrors == 0);
  const bool oneCsPerPacket = (bus.transactions() == calls);
  const bool pass = framed && noOversize && clean && idle && oneCsPerPacket;

  if (!framed) Serial.println("  MISSING: first payload is not the 272-byte channel-0 advertisement");
  if (!noOversize) Serial.println("  UNEXPECTED: an oversized transfer was seen");
  if (!clean) Serial.println("  DESYNC: fragment/channel/bus error counters non-zero");
  if (!idle) Serial.println("  DESYNC: device still presenting data after the boot burst");
  if (!oneCsPerPacket) Serial.println("  FRAMING: bus transactions do not match packets read");

  Serial.println(pass ? "RESULT: PASS - 276-byte advertisement, one CS assertion per packet, stream idle"
                      : "RESULT: FAIL");
}

void loop() { delay(1000); }
