#ifndef BNO085_DIAG_LOGTYPES_H
#define BNO085_DIAG_LOGTYPES_H

#include <cstdint>

namespace bno085 {

enum class LogLevel : uint8_t {
  Off = 0,
  Error,
  Warn,
  Info,
  Debug,
  Trace,
};

enum LogCategory : uint16_t {
  LogReset  = 1u << 0,
  LogInt    = 1u << 1,
  LogCs     = 1u << 2,
  LogSpi    = 1u << 3,
  LogShtp   = 1u << 4,
  LogSh2    = 1u << 5,
  LogInit   = 1u << 6,
  LogData   = 1u << 7,
};

constexpr uint16_t logAllCategories = 0xFFFF;

const char* logLevelName(LogLevel level);

// Sink for formatted log lines. Implementations must not block indefinitely.
class ILogSink {
 public:
  virtual ~ILogSink() {}
  virtual void write(const char* line) = 0;
};

// Discards everything; used when logging is disabled.
class NullLogSink : public ILogSink {
 public:
  void write(const char*) override {}
};

} // namespace bno085

#endif // BNO085_DIAG_LOGTYPES_H
