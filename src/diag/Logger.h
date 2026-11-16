#ifndef BNO085_DIAG_LOGGER_H
#define BNO085_DIAG_LOGGER_H

#include <cstddef>
#include <cstdint>

#include "BNO085Config.h"
#include "diag/LogTypes.h"

namespace bno085 {

// Formats into a fixed stack buffer and hands the line to an ILogSink.
// No dynamic allocation, no blocking, and every message is filtered before
// formatting so disabled categories cost one branch.
class Logger {
 public:
  Logger();

  void attach(ILogSink* sink) { sink_ = sink; }
  void setLevel(LogLevel level) { level_ = level; }
  LogLevel level() const { return level_; }
  void setCategories(uint16_t categories) { categories_ = categories; }
  uint16_t categories() const { return categories_; }

  bool enabled(LogLevel level, uint16_t category) const {
    return sink_ != nullptr && level_ != LogLevel::Off && level <= level_ &&
           (categories_ & category) != 0;
  }

  void log(LogLevel level, uint16_t category, const char* fmt, ...);
  void hex(LogLevel level, uint16_t category, const char* tag,
           const uint8_t* data, size_t len);

  // Lines dropped because the formatted message exceeded the buffer.
  uint32_t dropped() const { return dropped_; }

 private:
  ILogSink* sink_;
  LogLevel level_;
  uint16_t categories_;
  uint32_t dropped_;
};

} // namespace bno085

#endif // BNO085_DIAG_LOGGER_H
