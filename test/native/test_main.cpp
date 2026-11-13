// Host tests for the portable core. No Arduino, no hardware.
//
// The packet bytes below are constructed from the documented SHTP/SH-2 layouts
// (datasheet 1000-3927 and the SH-2 report definitions). They are spec-derived
// golden vectors, not captures from a physical device.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "BNO085Sensor.h"
#include "diag/Logger.h"
#include "output/BinaryEncoder.h"
#include "sh2/Sh2Client.h"
#include "sh2/Sh2Codes.h"
#include "sh2/Sh2Reports.h"
#include "shtp/ShtpTransport.h"
#include "util/Crc16.h"

#include "mock_hal.h"

namespace {

int g_checks = 0;
int g_failures = 0;

void checkImpl(bool condition, const char* expression, const char* file, int line) {
  ++g_checks;
  if (!condition) {
    ++g_failures;
    std::printf("    FAIL %s:%d  %s\n", file, line, expression);
  }
}

#define CHECK(cond) checkImpl((cond), #cond, __FILE__, __LINE__)

bool nearly(float a, float b, float epsilon = 1e-6f) {
  return std::fabs(a - b) <= epsilon;
}

// ---------------------------------------------------------------------------
// Packet construction helpers (all little-endian, matching the wire format)
// ---------------------------------------------------------------------------
void put16(std::vector<uint8_t>& v, size_t offset, uint16_t value) {
  v[offset] = static_cast<uint8_t>(value & 0xFF);
  v[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void put32(std::vector<uint8_t>& v, size_t offset, uint32_t value) {
  v[offset] = static_cast<uint8_t>(value & 0xFF);
  v[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  v[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  v[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

std::vector<uint8_t> makePacket(uint8_t channel, uint8_t sequence,
                                const std::vector<uint8_t>& payload,
                                bool continuation = false) {
  const uint16_t total = static_cast<uint16_t>(payload.size() + 4);
  uint16_t field = static_cast<uint16_t>(total & 0x7FFF);
  if (continuation) field = static_cast<uint16_t>(field | 0x8000);
  std::vector<uint8_t> packet;
  packet.push_back(static_cast<uint8_t>(field & 0xFF));
  packet.push_back(static_cast<uint8_t>((field >> 8) & 0xFF));
  packet.push_back(channel);
  packet.push_back(sequence);
  packet.insert(packet.end(), payload.begin(), payload.end());
  return packet;
}

// Raw 4-byte header, used to build malformed transfers.
std::vector<uint8_t> makeHeader(uint16_t lengthField, uint8_t channel,
                                uint8_t sequence) {
  std::vector<uint8_t> packet;
  packet.push_back(static_cast<uint8_t>(lengthField & 0xFF));
  packet.push_back(static_cast<uint8_t>((lengthField >> 8) & 0xFF));
  packet.push_back(channel);
  packet.push_back(sequence);
  return packet;
}

// Rotation vector report (ID 0x05, 14 bytes).
std::vector<uint8_t> makeRotationVector(uint8_t sequence, int16_t i, int16_t j,
                                        int16_t k, int16_t real, uint8_t accuracy,
                                        uint16_t delayTicks, uint16_t accuracyQ12) {
  std::vector<uint8_t> r(14, 0);
  r[0] = 0x05;
  r[1] = sequence;
  const uint8_t delayHigh = static_cast<uint8_t>((delayTicks >> 8) & 0x3F);
  r[2] = static_cast<uint8_t>((accuracy & 0x03) | static_cast<uint8_t>(delayHigh << 2));
  r[3] = static_cast<uint8_t>(delayTicks & 0xFF);
  put16(r, 4, static_cast<uint16_t>(i));
  put16(r, 6, static_cast<uint16_t>(j));
  put16(r, 8, static_cast<uint16_t>(k));
  put16(r, 10, static_cast<uint16_t>(real));
  put16(r, 12, accuracyQ12);
  return r;
}

std::vector<uint8_t> makeProductId(uint32_t partNumber, uint32_t buildNumber,
                                   uint16_t patch, uint8_t major, uint8_t minor) {
  std::vector<uint8_t> r(16, 0);
  r[0] = 0xF8;
  r[1] = 1;  // reset cause
  r[2] = major;
  r[3] = minor;
  put32(r, 4, partNumber);
  put32(r, 8, buildNumber);
  put16(r, 12, patch);
  return r;
}

std::vector<uint8_t> makeCommandResponse(uint8_t command, uint8_t commandSeq,
                                         uint8_t hubStatus) {
  std::vector<uint8_t> r(16, 0);
  r[0] = 0xF1;
  r[1] = 0;
  r[2] = command;
  r[3] = commandSeq;
  r[4] = 0;
  r[5] = hubStatus;
  return r;
}

std::vector<uint8_t> makeGetFeatureResponse(uint8_t reportId, uint32_t intervalUs) {
  std::vector<uint8_t> r(17, 0);
  r[0] = 0xFC;
  r[1] = reportId;
  put32(r, 5, intervalUs);
  return r;
}

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------
struct TransportFixture {
  test::MockHal hal;
  bno085::Logger log;
  bno085::ShtpTransport shtp;
  int calls;
  uint8_t lastChannel;
  uint16_t lastLength;
  uint32_t lastTime;
  std::vector<uint8_t> lastPayload;

  TransportFixture()
      : hal(), log(), shtp(), calls(0), lastChannel(0xFF), lastLength(0),
        lastTime(0), lastPayload() {
    shtp.attach(&hal, &log, 0);
    shtp.setHandler(&TransportFixture::onChannel, this);
  }

  static void onChannel(void* context, uint8_t channel, const uint8_t* payload,
                        uint16_t len, uint32_t tUs) {
    TransportFixture* self = static_cast<TransportFixture*>(context);
    ++self->calls;
    self->lastChannel = channel;
    self->lastLength = len;
    self->lastTime = tUs;
    self->lastPayload.assign(payload, payload + len);
  }
};

struct Sh2Fixture {
  test::MockHal hal;
  bno085::Logger log;
  bno085::ShtpTransport shtp;
  bno085::Sh2Client sh2;
  int quaternionCalls;
  int resetCalls;
  bno085::Quaternion lastQuaternion;

  Sh2Fixture()
      : hal(), log(), shtp(), sh2(), quaternionCalls(0), resetCalls(0),
        lastQuaternion() {
    shtp.attach(&hal, &log, 0);
    sh2.attach(&shtp, &log, 0);
    shtp.setHandler(&Sh2Fixture::onChannel, this);
    bno085::Sh2Client::Handlers handlers;
    handlers.context = this;
    handlers.quaternion = &Sh2Fixture::onQuaternion;
    handlers.resetComplete = &Sh2Fixture::onReset;
    sh2.setHandlers(handlers);
  }

  static void onChannel(void* context, uint8_t channel, const uint8_t* payload,
                        uint16_t len, uint32_t tUs) {
    static_cast<Sh2Fixture*>(context)->sh2.handlePayload(channel, payload, len, tUs);
  }

  static void onQuaternion(void* context, const bno085::Quaternion& q) {
    Sh2Fixture* self = static_cast<Sh2Fixture*>(context);
    ++self->quaternionCalls;
    self->lastQuaternion = q;
  }

  static void onReset(void* context) {
    ++static_cast<Sh2Fixture*>(context)->resetCalls;
  }
};

struct SensorFixture {
  test::MockHal hal;
  test::BufferSink sink;
  bno085::ManagerConfig manager;
  bno085::SensorConfig config;
  bno085::Sensor sensor;

  SensorFixture()
      : hal(), sink(), manager(), config(10, -1, -1), sensor(hal) {
    manager.output = &sink;
    manager.emitBinaryFrames = true;
    manager.statusIntervalMs = 0;
    manager.log = nullptr;
    manager.spiHz = 1000000;
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
void testCrc16() {
  const char* text = "123456789";
  CHECK(bno085::crc16CcittFalse(reinterpret_cast<const uint8_t*>(text), 9) == 0x29B1);

  const uint16_t partial =
      bno085::crc16CcittFalse(reinterpret_cast<const uint8_t*>(text), 4);
  const uint16_t rest =
      bno085::crc16CcittFalse(reinterpret_cast<const uint8_t*>(text) + 4, 5, partial);
  CHECK(rest == 0x29B1);
}

void testEncoderQuaternionFrame() {
  bno085::BinaryEncoder encoder;
  test::BufferSink sink;
  encoder.attach(&sink, nullptr);

  bno085::Quaternion q;
  q.rawX = 4096;    // 0.25
  q.rawY = -2048;   // -0.125
  q.rawZ = 0;
  q.rawW = 15872;   // 0.96875
  q.rawAccuracy = 0x0080;
  q.accuracy = bno085::Accuracy::High;
  q.tMeasUs = 1000;
  q.tHostUs = 2000;
  q.seq = 7;
  q.epoch = 2;

  CHECK(encoder.writeQuaternion(3, q));
  CHECK(sink.bytes.size() == 34);
  CHECK(sink.bytes[0] == 0xA5);
  CHECK(sink.bytes[1] == 0x5A);
  CHECK(sink.bytes[2] == 1);
  CHECK(sink.bytes[3] == 0x01);
  CHECK(bno085::sh2::readU16(&sink.bytes[4]) == 24);
  CHECK(bno085::sh2::readU16(&sink.bytes[6]) == 0);  // frame sequence
  CHECK(sink.bytes[8] == 3);
  CHECK(sink.bytes[11] == 7);    // sequence
  CHECK(sink.bytes[12] == 2);    // epoch
  CHECK(sink.bytes[13] == 14);   // Q14
  CHECK(bno085::sh2::readI16(&sink.bytes[14]) == 4096);
  CHECK(bno085::sh2::readI16(&sink.bytes[16]) == -2048);
  CHECK(bno085::sh2::readI16(&sink.bytes[22]) == 0x0080);
  CHECK(bno085::sh2::readU32(&sink.bytes[24]) == 1000);
  CHECK(bno085::sh2::readU32(&sink.bytes[28]) == 2000);

  const uint16_t expected = bno085::crc16CcittFalse(&sink.bytes[2], 6 + 24);
  CHECK(bno085::sh2::readU16(&sink.bytes[32]) == expected);
  CHECK(encoder.framesWritten() == 1);

  // A single flipped byte must invalidate the frame.
  std::vector<uint8_t> corrupted = sink.bytes;
  corrupted[20] ^= 0xFF;
  CHECK(bno085::crc16CcittFalse(&corrupted[2], 6 + 24) !=
        bno085::sh2::readU16(&corrupted[32]));
}

void testShtpValidRotationVector() {
  TransportFixture fx;
  const std::vector<uint8_t> report =
      makeRotationVector(42, 4096, -2048, 0, 15872, 3, 0, 0x0080);
  fx.hal.queuePacket(makePacket(3, 7, report));

  CHECK(fx.shtp.service(5000) == bno085::ShtpResult::Ok);
  CHECK(fx.calls == 1);
  CHECK(fx.lastChannel == 3);
  CHECK(fx.lastLength == 14);
  CHECK(fx.lastTime == 5000);
  CHECK(fx.lastPayload.size() == 14);
  CHECK(fx.lastPayload[0] == 0x05);
  CHECK(fx.shtp.stats().packets == 1);
  CHECK(fx.shtp.stats().sequenceGaps == 0);
}

void testShtpNoData() {
  TransportFixture fx;
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::NoData);
  CHECK(fx.calls == 0);
}

void testShtpZeroLengthIsSpurious() {
  TransportFixture fx;
  fx.hal.queuePacket(makeHeader(0, 3, 0));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Spurious);
  CHECK(fx.shtp.stats().spurious == 1);
  CHECK(fx.calls == 0);
}

void testShtpShortFragment() {
  TransportFixture fx;
  fx.hal.queuePacket(makeHeader(3, 3, 0));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Malformed);
  CHECK(fx.shtp.stats().shortFragments == 1);
  CHECK(fx.calls == 0);
}

void testShtpReservedLength() {
  TransportFixture fx;
  fx.hal.queuePacket(makeHeader(0xFFFF, 3, 0));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Malformed);
  CHECK(fx.shtp.stats().badFragments == 1);
}

void testShtpBadChannel() {
  TransportFixture fx;
  const std::vector<uint8_t> payload(6, 0xAA);
  fx.hal.queuePacket(makePacket(9, 0, payload));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Malformed);
  CHECK(fx.shtp.stats().badChannels == 1);
  CHECK(fx.calls == 0);
}

void testShtpOversizeTransfer() {
  TransportFixture fx;
  std::vector<uint8_t> payload(296, 0x55);  // declared length 300 > transfer buffer
  fx.hal.queuePacket(makePacket(3, 0, payload));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Oversize);
  CHECK(fx.shtp.stats().oversizePayloads == 1);
  CHECK(fx.calls == 0);
}

void testShtpPayloadExceedsPayloadBuffer() {
  TransportFixture fx;
  // Fits the transfer buffer (204 <= 256) but not the payload buffer (200 > 192).
  std::vector<uint8_t> payload(200, 0x11);
  fx.hal.queuePacket(makePacket(3, 0, payload));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Oversize);
  CHECK(fx.calls == 0);
  CHECK(fx.shtp.stats().oversizePayloads == 1);
}

void testShtpUnexpectedContinuation() {
  TransportFixture fx;
  const std::vector<uint8_t> payload(6, 0x22);
  fx.hal.queuePacket(makePacket(3, 0, payload, true));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Malformed);
  CHECK(fx.shtp.stats().badFragments == 1);
  CHECK(fx.calls == 0);
}

void testShtpBusError() {
  TransportFixture fx;
  fx.hal.intState = true;
  fx.hal.failTransfers = true;
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::BusError);
  CHECK(fx.shtp.stats().rxErrors == 1);
}

void testShtpSequenceGapIsCounted() {
  TransportFixture fx;
  const std::vector<uint8_t> report =
      makeRotationVector(1, 0, 0, 0, 16384, 3, 0, 0);
  fx.hal.queuePacket(makePacket(3, 0, report));
  CHECK(fx.shtp.service(1000) == bno085::ShtpResult::Ok);
  fx.hal.queuePacket(makePacket(3, 3, report));
  CHECK(fx.shtp.service(2000) == bno085::ShtpResult::Ok);
  CHECK(fx.shtp.stats().sequenceGaps == 1);
}

void testSh2ProductId() {
  Sh2Fixture fx;
  fx.sh2.requestProductIds(1000);
  fx.hal.queuePacket(makePacket(2, 0, makeProductId(10004158u, 1234u, 5, 3, 2)));
  CHECK(fx.shtp.service(2000) == bno085::ShtpResult::Ok);

  const bno085::ProductInfo& p = fx.sh2.productInfo();
  CHECK(p.valid);
  CHECK(p.partNumber == 10004158u);
  CHECK(p.buildNumber == 1234u);
  CHECK(p.patch == 5);
  CHECK(p.major == 3);
  CHECK(p.minor == 2);
  CHECK(p.entries == 1);
}

void testSh2SetFeatureBytes() {
  Sh2Fixture fx;
  CHECK(fx.sh2.setFeature(0x05, 60000, 1000));
  CHECK(fx.hal.sent.size() == 1);

  const std::vector<uint8_t>& frame = fx.hal.sent.back();
  CHECK(frame.size() == 21);
  CHECK(frame[0] == 0x15);  // length 21
  CHECK(frame[1] == 0x00);
  CHECK(frame[2] == 0x02);  // SH-2 control channel
  CHECK(frame[3] == 0x00);  // SHTP sequence
  CHECK(frame[4] == 0xFD);  // set feature
  CHECK(frame[5] == 0x00);  // command sequence
  CHECK(frame[6] == 0x00);  // flags
  CHECK(frame[9] == 0x60);  // 60000 = 0x0000EA60
  CHECK(frame[10] == 0xEA);
  CHECK(frame[11] == 0x00);
  CHECK(frame[12] == 0x00);
  CHECK(fx.sh2.cmdState() == bno085::Sh2Client::CmdState::Pending);
}

void testSh2RotationVectorDecode() {
  Sh2Fixture fx;
  const std::vector<uint8_t> report =
      makeRotationVector(42, 4096, -2048, 0, 15872, 3, 0, 0x0080);
  fx.sh2.handlePayload(3, report.data(), static_cast<uint16_t>(report.size()), 100000);

  CHECK(fx.quaternionCalls == 1);
  const bno085::Quaternion& q = fx.lastQuaternion;
  CHECK(q.valid);
  CHECK(q.rawX == 4096);
  CHECK(q.rawY == -2048);
  CHECK(q.rawZ == 0);
  CHECK(q.rawW == 15872);
  CHECK(nearly(q.x, 0.25f));
  CHECK(nearly(q.y, -0.125f));
  CHECK(nearly(q.z, 0.0f));
  CHECK(nearly(q.w, 0.96875f));
  CHECK(q.accuracy == bno085::Accuracy::High);
  CHECK(q.rawAccuracy == 0x0080);
  CHECK(q.seq == 42);
  CHECK(q.tHostUs == 100000);
  CHECK(q.tMeasUs == 88000);  // 100000 - 12000 reference, no delay
  CHECK(q.source == bno085::Report::RotationVector);
}

void testSh2DelayField() {
  Sh2Fixture fx;
  const std::vector<uint8_t> report =
      makeRotationVector(1, 0, 0, 0, 16384, 0, 300, 0);
  fx.sh2.handlePayload(3, report.data(), static_cast<uint16_t>(report.size()), 100000);
  CHECK(fx.quaternionCalls == 1);
  CHECK(fx.lastQuaternion.delayTicks == 300);
  CHECK(fx.lastQuaternion.tMeasUs == 118000);  // 100000 - 12000 + 30000
  CHECK(fx.lastQuaternion.accuracy == bno085::Accuracy::Unreliable);
}

void testSh2DuplicateSuppression() {
  Sh2Fixture fx;
  const std::vector<uint8_t> report =
      makeRotationVector(9, 0, 0, 0, 16384, 3, 0, 0);
  fx.sh2.handlePayload(3, report.data(), static_cast<uint16_t>(report.size()), 50000);
  fx.sh2.handlePayload(3, report.data(), static_cast<uint16_t>(report.size()), 50000);
  CHECK(fx.quaternionCalls == 1);
  CHECK(fx.sh2.stats().duplicates == 1);
  CHECK(fx.sh2.stats().reports == 1);
}

void testSh2ReportGap() {
  Sh2Fixture fx;
  std::vector<uint8_t> first = makeRotationVector(10, 0, 0, 0, 16384, 3, 0, 0);
  std::vector<uint8_t> second = makeRotationVector(13, 0, 0, 0, 16384, 3, 0, 0);
  fx.sh2.handlePayload(3, first.data(), static_cast<uint16_t>(first.size()), 10000);
  fx.sh2.handlePayload(3, second.data(), static_cast<uint16_t>(second.size()), 20000);
  CHECK(fx.quaternionCalls == 2);
  CHECK(fx.sh2.stats().reportGaps == 2);  // 11 and 12 missing
}

void testSh2UnknownReportStopsCleanly() {
  Sh2Fixture fx;
  std::vector<uint8_t> payload;
  payload.push_back(0x7E);  // unknown report id
  payload.push_back(0x00);
  fx.sh2.handlePayload(3, payload.data(), static_cast<uint16_t>(payload.size()), 1000);
  CHECK(fx.quaternionCalls == 0);
  CHECK(fx.sh2.stats().unknownReports == 1);
}

void testSh2CommandTimeout() {
  Sh2Fixture fx;
  fx.sh2.setTimeoutMs(100);
  CHECK(fx.sh2.setFeature(0x05, 10000, 1000));
  CHECK(fx.sh2.busy());
  fx.sh2.pollTimeouts(1000 + 100000);
  CHECK(!fx.sh2.busy());
  CHECK(fx.sh2.cmdState() == bno085::Sh2Client::CmdState::Timeout);
  CHECK(fx.sh2.stats().timeouts == 1);
  CHECK(fx.sh2.lastError() == bno085::ErrorCode::Sh2Timeout);
}

void testSh2HubRejection() {
  Sh2Fixture fx;
  CHECK(fx.sh2.setFeature(0x05, 10000, 1000));
  fx.hal.queuePacket(makePacket(2, 0, makeCommandResponse(0xFD, 0, 1)));
  CHECK(fx.shtp.service(2000) == bno085::ShtpResult::Ok);
  CHECK(fx.sh2.cmdState() == bno085::Sh2Client::CmdState::HubError);
  CHECK(fx.sh2.stats().badResponses == 1);
}

void testSh2ResetComplete() {
  Sh2Fixture fx;
  fx.sh2.handlePayload(1, reinterpret_cast<const uint8_t*>("\x01"), 1, 1000);
  CHECK(fx.resetCalls == 1);
  CHECK(fx.sh2.epoch() == 1);
}

void testSensorInitializationSequence() {
  SensorFixture fx;
  CHECK(fx.sensor.begin(fx.manager, fx.config, 0).ok());

  // Advertisement on channel 0 and reset-complete on channel 1.
  fx.hal.queuePacket(makePacket(0, 0, std::vector<uint8_t>(8, 0x42)));
  fx.hal.queuePacket(makePacket(1, 0, std::vector<uint8_t>(1, 0x01)));
  fx.sensor.update();
  CHECK(fx.sensor.state() == bno085::State::Identifying);
  CHECK(!fx.hal.sent.empty());

  fx.hal.queuePacket(makePacket(2, 0, makeProductId(10004158u, 1u, 2, 3, 1)));
  fx.sensor.update();
  CHECK(fx.sensor.state() == bno085::State::Configuring);
  CHECK(fx.sensor.productInfo().valid);

  fx.sensor.update();  // sends set-feature
  CHECK(fx.hal.sent.size() >= 2);

  fx.hal.queuePacket(makePacket(2, 1, makeCommandResponse(0xFD, 0, 0)));
  fx.sensor.update();

  fx.sensor.update();  // sends get-feature
  fx.hal.queuePacket(makePacket(2, 2, makeGetFeatureResponse(0x05, 10000)));
  fx.sensor.update();
  CHECK(fx.sensor.state() == bno085::State::Streaming);
  CHECK(fx.sensor.diagnostics().grantedIntervalUs == 10000);

  fx.hal.queuePacket(
      makePacket(3, 0, makeRotationVector(1, 4096, 0, 0, 15872, 3, 0, 0)));
  fx.sensor.update();

  bno085::Quaternion q;
  CHECK(fx.sensor.latest(q));
  CHECK(q.valid);
  CHECK(nearly(q.x, 0.25f));
  CHECK(fx.sensor.diagnostics().reportsDecoded == 1);
  CHECK(!fx.sensor.latest(q));  // edge-triggered
}

void testSensorBootTimeout() {
  SensorFixture fx;
  CHECK(fx.sensor.begin(fx.manager, fx.config, 0).ok());
  CHECK(fx.sensor.state() == bno085::State::Booting);
  fx.hal.advance(400000);  // beyond the 300 ms boot budget
  fx.sensor.update();
  CHECK(fx.sensor.state() == bno085::State::Failed);
  CHECK(fx.sensor.lastError() == bno085::ErrorCode::BootTimeout);
}

void testSensorRejectsUnsupportedReport() {
  SensorFixture fx;
  fx.config.report = bno085::Report::GameRotationVector;
  const bno085::Status status = fx.sensor.begin(fx.manager, fx.config, 0);
  CHECK(!status.ok());
  CHECK(status.code == bno085::Status::InvalidConfig);
  CHECK(fx.sensor.lastError() == bno085::ErrorCode::UnsupportedReport);
}

void testSensorSurvivesMalformedPacket() {
  SensorFixture fx;
  CHECK(fx.sensor.begin(fx.manager, fx.config, 0).ok());
  fx.hal.queuePacket(makePacket(0, 0, std::vector<uint8_t>(4, 0)));
  fx.sensor.update();
  fx.hal.queuePacket(makePacket(2, 0, makeProductId(10004158u, 1u, 2, 3, 1)));
  fx.sensor.update();
  fx.sensor.update();
  fx.hal.queuePacket(makePacket(2, 1, makeCommandResponse(0xFD, 0, 0)));
  fx.sensor.update();
  fx.sensor.update();
  fx.hal.queuePacket(makePacket(2, 2, makeGetFeatureResponse(0x05, 10000)));
  fx.sensor.update();
  CHECK(fx.sensor.state() == bno085::State::Streaming);

  // Malformed transfer: short fragment.
  fx.hal.queuePacket(makeHeader(3, 3, 0));
  fx.sensor.update();

  // The next valid report must still be decoded.
  fx.hal.queuePacket(
      makePacket(3, 1, makeRotationVector(1, 0, 4096, 0, 15872, 2, 0, 0)));
  fx.sensor.update();

  bno085::Quaternion q;
  CHECK(fx.sensor.latest(q));
  CHECK(q.valid);
  CHECK(q.accuracy == bno085::Accuracy::Medium);
}

void testSensorRecoverBackoff() {
  SensorFixture fx;
  CHECK(fx.sensor.begin(fx.manager, fx.config, 0).ok());
  CHECK(fx.sensor.recover().ok());
  CHECK(fx.sensor.state() == bno085::State::Recovering);
  fx.sensor.update();
  CHECK(fx.sensor.state() == bno085::State::Recovering);  // still in backoff
  fx.hal.advance(300000);
  fx.sensor.update();
  CHECK(fx.sensor.state() == bno085::State::Booting);
}

} // namespace

int main() {
  struct TestCase {
    const char* name;
    void (*fn)();
  };

  const TestCase tests[] = {
      {"crc16_known_vector", testCrc16},
      {"encoder_quaternion_frame", testEncoderQuaternionFrame},
      {"shtp_valid_rotation_vector", testShtpValidRotationVector},
      {"shtp_no_data", testShtpNoData},
      {"shtp_zero_length", testShtpZeroLengthIsSpurious},
      {"shtp_short_fragment", testShtpShortFragment},
      {"shtp_reserved_length", testShtpReservedLength},
      {"shtp_bad_channel", testShtpBadChannel},
      {"shtp_oversize_transfer", testShtpOversizeTransfer},
      {"shtp_payload_too_large", testShtpPayloadExceedsPayloadBuffer},
      {"shtp_unexpected_continuation", testShtpUnexpectedContinuation},
      {"shtp_bus_error", testShtpBusError},
      {"shtp_sequence_gap", testShtpSequenceGapIsCounted},
      {"sh2_product_id", testSh2ProductId},
      {"sh2_set_feature_bytes", testSh2SetFeatureBytes},
      {"sh2_rotation_vector_decode", testSh2RotationVectorDecode},
      {"sh2_delay_field", testSh2DelayField},
      {"sh2_duplicate_suppression", testSh2DuplicateSuppression},
      {"sh2_report_gap", testSh2ReportGap},
      {"sh2_unknown_report", testSh2UnknownReportStopsCleanly},
      {"sh2_command_timeout", testSh2CommandTimeout},
      {"sh2_hub_rejection", testSh2HubRejection},
      {"sh2_reset_complete", testSh2ResetComplete},
      {"sensor_init_sequence", testSensorInitializationSequence},
      {"sensor_boot_timeout", testSensorBootTimeout},
      {"sensor_rejects_grv", testSensorRejectsUnsupportedReport},
      {"sensor_survives_malformed", testSensorSurvivesMalformedPacket},
      {"sensor_recover_backoff", testSensorRecoverBackoff},
  };

  std::printf("BNO085Multi host tests (%u cases)\n", static_cast<unsigned>(sizeof(tests) / sizeof(tests[0])));
  for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
    std::printf("[test] %s\n", tests[i].name);
    tests[i].fn();
  }

  std::printf("\n%u checks, %u failures\n", static_cast<unsigned>(g_checks),
              static_cast<unsigned>(g_failures));
  if (g_failures == 0) {
    std::printf("RESULT: PASS\n");
    return 0;
  }
  std::printf("RESULT: FAIL\n");
  return 1;
}
