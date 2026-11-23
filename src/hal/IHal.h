#ifndef BNO085_HAL_IHAL_H
#define BNO085_HAL_IHAL_H

#include <cstddef>
#include <cstdint>

#include "BNO085Types.h"

namespace bno085 {

// Hardware abstraction for one BNO085.
//
// The HAL owns no protocol knowledge: it performs one chip-select-asserted
// transfer at a time, reports the interrupt line, pulses reset and provides a
// microsecond clock. Everything above it is platform independent.
class IHal {
 public:
  virtual ~IHal() {}

  // Configure pins and the shared bus. Must leave every CS deasserted.
  virtual Status begin() = 0;

  // Release pins/bus state owned by this sensor. Must not deassert another
  // sensor's CS.
  virtual void end() = 0;

  // One SPI transaction with this sensor's CS asserted for the whole call.
  // tx may be null (send 0x00), rx may be null (discard).
  // Returns false on a bus-level failure; never blocks indefinitely.
  virtual bool transfer(const uint8_t* tx, uint8_t* rx, size_t len) = 0;

  // Continuous chip-select read.
  //
  // A BNO085 restarts its transfer presentation when CS is released, so an
  // SHTP transfer must be clocked out inside ONE CS assertion: the 4-byte
  // header first, then the remaining payload bytes. Splitting the header and
  // the payload into two transfer() calls is not equivalent and silently
  // corrupts every packet.
  //
  // beginRead() asserts CS, takes the bus and begins the transaction.
  // read() clocks bytes while CS stays asserted. endRead() releases CS and the
  // bus and is idempotent, so it is safe to call on every exit path, including
  // after beginRead() failed. Nothing but SPI byte transfers may happen between
  // beginRead() and endRead().
  virtual bool beginRead() = 0;
  virtual bool read(uint8_t* dst, size_t len) = 0;
  virtual void endRead() = 0;

  // True while H_INTN is asserted (active low). With no INT pin configured,
  // returns true so the caller polls.
  virtual bool intAsserted() = 0;

  // Called before a host-to-device write. Implementations may pulse WAKE and
  // wait (bounded) for the device to assert INT. Returning false aborts the
  // write attempt for this pass.
  virtual bool prepareWrite() { return true; }

  // Drive NRST low, then release it.
  virtual void pulseReset() = 0;

  // Monotonic microsecond counter, wraps at 2^32.
  virtual uint32_t timeUs() = 0;

  // Number of CS-asserted transactions performed (diagnostics).
  virtual uint32_t transferCount() const { return 0; }
};

} // namespace bno085

#endif // BNO085_HAL_IHAL_H
