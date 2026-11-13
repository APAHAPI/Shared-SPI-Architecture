#ifndef BNO085_TEST_MOCK_HAL_H
#define BNO085_TEST_MOCK_HAL_H

#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#include "hal/IHal.h"
#include "output/IOutputSink.h"

namespace test {

// Emulates the SPI side of a BNO085 closely enough to exercise framing:
//  - each queued item is one SHTP transfer, delivered as raw bytes;
//  - a 4-byte read is treated as a header peek and does not consume the item,
//    because the device re-presents the header on every new CS assertion;
//  - writes are recorded, never delivered.
class MockHal : public bno085::IHal {
 public:
  std::deque<std::vector<uint8_t> > queue;
  std::vector<std::vector<uint8_t> > sent;
  bool intState = false;
  bool failTransfers = false;
  uint32_t clockUs = 1000;
  uint32_t transferCalls = 0;
  uint32_t resets = 0;
  size_t partialPos = 0;

  void queuePacket(const std::vector<uint8_t>& packet) { queue.push_back(packet); }

  void clear() {
    queue.clear();
    sent.clear();
    partialPos = 0;
    intState = false;
    failTransfers = false;
  }

  bno085::Status begin() override { return bno085::Status(); }
  void end() override {}

  bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) override {
    ++transferCalls;
    if (tx != nullptr) {
      sent.push_back(std::vector<uint8_t>(tx, tx + len));
    }
    if (failTransfers) return false;
    if (rx == nullptr) return true;

    if (queue.empty()) {
      std::memset(rx, 0, len);
      return true;
    }

    std::vector<uint8_t>& packet = queue.front();

    // Header peek: nothing is consumed.
    if (len == 4 && partialPos == 0 && packet.size() > 4) {
      std::memcpy(rx, packet.data(), 4);
      return true;
    }

    const size_t available = packet.size() - partialPos;
    const size_t copied = (len < available) ? len : available;
    std::memcpy(rx, packet.data() + partialPos, copied);
    if (copied < len) std::memset(rx + copied, 0, len - copied);

    partialPos += copied;
    if (partialPos >= packet.size()) {
      queue.pop_front();
      partialPos = 0;
    }
    return true;
  }

  bool intAsserted() override { return intState || !queue.empty(); }
  bool prepareWrite() override { return true; }
  void pulseReset() override { ++resets; }
  uint32_t timeUs() override { return clockUs; }
  uint32_t transferCount() const override { return transferCalls; }

  void advance(uint32_t deltaUs) { clockUs += deltaUs; }
};

// Captures encoded frames for inspection.
class BufferSink : public bno085::IOutputSink {
 public:
  std::vector<uint8_t> bytes;

  size_t write(const uint8_t* data, size_t len) override {
    bytes.insert(bytes.end(), data, data + len);
    return len;
  }

  void clear() { bytes.clear(); }
};

} // namespace test

#endif // BNO085_TEST_MOCK_HAL_H
