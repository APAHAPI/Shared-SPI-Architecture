#ifndef BNO085_TEST_MOCK_HAL_H
#define BNO085_TEST_MOCK_HAL_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#include "hal/IHal.h"
#include "output/IOutputSink.h"

namespace test {

// Emulates the SPI side of a BNO085, including the chip-select transaction
// boundary that the real device enforces:
//
//  - each queued item is one SHTP transfer, delivered as raw bytes;
//  - a CS assertion presents exactly one pending transfer, from its first byte;
//  - bytes clocked while CS is asserted consume that transfer in order;
//  - after that transfer is exhausted, further bytes in the SAME assertion are
//    zeros: the device presents the next transfer only on the next CS assertion;
//  - releasing CS before the transfer is complete aborts it. The device drops
//    the remainder, so a header read and a payload read in two separate CS
//    assertions can never reassemble a packet;
//  - continuing a read after CS was released is a violation and fails;
//  - a one-shot transfer() is exactly one CS assertion, so it can only consume
//    a transfer if that single call clocks the whole thing.
//
// The previous mock treated a 4-byte read as a free "peek" that did not consume
// anything and re-presented the header to the next read. That made the split
// header/payload read look correct, which is why the host suite missed the
// physical defect.
class MockHal : public bno085::IHal {
 public:
  std::deque<std::vector<uint8_t> > queue;
  std::vector<std::vector<uint8_t> > sent;
  bool intState = false;
  bool failTransfers = false;
  uint32_t clockUs = 1000;
  uint32_t transferCalls = 0;
  uint32_t resets = 0;

  // Bytes of queue.front() already clocked inside the current CS assertion.
  size_t partialPos = 0;

  // True while the transfer presented at the current CS assertion still has
  // bytes left. Once it is exhausted, the rest of this assertion reads zeros.
  bool presenting = false;

  // Chip-select / transaction instrumentation.
  bool csAsserted = false;
  uint32_t csAssertions = 0;
  uint32_t csDeassertions = 0;
  uint32_t csViolations = 0;       // read/transfer attempted with CS released
  uint32_t framingAborts = 0;      // CS released before the transfer completed
  uint32_t bytesWhileCs = 0;       // total bytes clocked with CS asserted
  uint32_t maxBytesPerAssertion = 0;
  uint32_t assertionBytes = 0;
  // Fails read() once this many bytes of the current transfer were clocked.
  int32_t failReadAtByte = -1;

  void queuePacket(const std::vector<uint8_t>& packet) { queue.push_back(packet); }

  void clear() {
    queue.clear();
    sent.clear();
    partialPos = 0;
    presenting = false;
    intState = false;
    failTransfers = false;
    failReadAtByte = -1;
    csAsserted = false;
    csAssertions = 0;
    csDeassertions = 0;
    csViolations = 0;
    framingAborts = 0;
    bytesWhileCs = 0;
    maxBytesPerAssertion = 0;
    assertionBytes = 0;
  }

  bno085::Status begin() override { return bno085::Status(); }
  void end() override { endRead(); }

  // One CS assertion covering one complete call.
  bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) override {
    ++transferCalls;

    if (csAsserted) {
      // Cannot start an independent transfer in the middle of an open read: it
      // would release CS and abort the other transfer.
      ++csViolations;
      return false;
    }

    if (tx != nullptr) {
      sent.push_back(std::vector<uint8_t>(tx, tx + len));
    }
    if (failTransfers) return false;

    csAsserted = true;
    ++csAssertions;
    partialPos = 0;
    assertionBytes = 0;
    presenting = true;  // this assertion presents exactly one transfer

    if (rx != nullptr && len > 0) {
      clockBytes(rx, len);
      assertionBytes = static_cast<uint32_t>(len);
      bytesWhileCs += static_cast<uint32_t>(len);
    }

    endRead();
    return true;
  }

  bool beginRead() override {
    if (csAsserted) {
      ++csViolations;
      return false;
    }
    if (failTransfers) return false;

    csAsserted = true;
    ++csAssertions;
    partialPos = 0;
    assertionBytes = 0;
    presenting = true;  // this assertion presents exactly one transfer
    return true;
  }

  bool read(uint8_t* dst, size_t len) override {
    if (!csAsserted) {
      // Attempt to continue a transfer after CS was released.
      ++csViolations;
      return false;
    }
    if (failTransfers) return false;
    if (failReadAtByte >= 0 &&
        static_cast<int32_t>(partialPos) >= failReadAtByte) {
      return false;
    }
    if (dst == nullptr || len == 0) return false;

    clockBytes(dst, len);
    assertionBytes += static_cast<uint32_t>(len);
    bytesWhileCs += static_cast<uint32_t>(len);
    return true;
  }

  void endRead() override {
    if (!csAsserted) return;  // idempotent

    csAsserted = false;
    ++csDeassertions;

    if (presenting && partialPos != 0) {
      // CS released mid-transfer: the device discards the pending remainder.
      ++framingAborts;
      if (!queue.empty()) queue.pop_front();
      partialPos = 0;
    }
    presenting = false;

    if (assertionBytes > maxBytesPerAssertion) {
      maxBytesPerAssertion = assertionBytes;
    }
    assertionBytes = 0;
  }

  bool intAsserted() override { return intState || !queue.empty(); }
  bool prepareWrite() override { return true; }
  void pulseReset() override { ++resets; }
  uint32_t timeUs() override { return clockUs; }
  uint32_t transferCount() const override { return transferCalls; }

  void advance(uint32_t deltaUs) { clockUs += deltaUs; }

 private:
  // Clocks `len` bytes out of the transfer presented at this CS assertion. Once
  // that transfer is exhausted this assertion yields zeros: the next queued
  // transfer is presented only after CS is released and asserted again.
  void clockBytes(uint8_t* dst, size_t len) {
    if (!presenting || queue.empty()) {
      std::memset(dst, 0, len);
      return;
    }

    std::vector<uint8_t>& packet = queue.front();
    size_t copied = 0;
    while (copied < len && partialPos < packet.size()) {
      dst[copied++] = packet[partialPos++];
    }
    if (copied < len) std::memset(dst + copied, 0, len - copied);

    if (partialPos >= packet.size()) {
      queue.pop_front();
      partialPos = 0;
      presenting = false;
    }
  }
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

// Sink with controllable capacity and write behaviour. It is the host-side
// abstraction boundary for the Arduino Stream sink: the real HardwareSerial
// reports free TX-ring space and does not report short writes, so these knobs
// test the encoder contract rather than the UART driver.
class CapacitySink : public bno085::IOutputSink {
 public:
  size_t capacity = npos;       // npos = report "unlimited"
  size_t accept = npos;         // npos = accept the whole frame
  size_t forcedReturn = npos;   // npos = return the accepted count
  size_t writeCalls = 0;
  size_t lastLen = 0;
  std::vector<uint8_t> bytes;

  size_t availableForWrite() const override { return capacity; }

  size_t write(const uint8_t* data, size_t len) override {
    ++writeCalls;
    lastLen = len;
    const size_t n = (accept == npos) ? len : ((accept < len) ? accept : len);
    bytes.insert(bytes.end(), data, data + n);
    if (forcedReturn != npos) return forcedReturn;
    return n;
  }

  void clear() {
    bytes.clear();
    writeCalls = 0;
    lastLen = 0;
  }
};

} // namespace test

#endif // BNO085_TEST_MOCK_HAL_H
