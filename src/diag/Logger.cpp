#include "diag/Logger.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace bno085 {

namespace {
constexpr size_t kLineBuffer = 192;

const char* levelTag(LogLevel level) {
  switch (level) {
    case LogLevel::Error: return "E";
    case LogLevel::Warn:  return "W";
    case LogLevel::Info:  return "I";
    case LogLevel::Debug: return "D";
    case LogLevel::Trace: return "T";
    default:              return "-";
  }
}
} // namespace

const char* logLevelName(LogLevel level) {
  switch (level) {
    case LogLevel::Off:   return "off";
    case LogLevel::Error: return "error";
    case LogLevel::Warn:  return "warn";
    case LogLevel::Info:  return "info";
    case LogLevel::Debug: return "debug";
    case LogLevel::Trace: return "trace";
  }
  return "unknown";
}

Logger::Logger()
    : sink_(nullptr), level_(LogLevel::Off), categories_(logAllCategories),
      dropped_(0) {}

void Logger::log(LogLevel level, uint16_t category, const char* fmt, ...) {
#if BNO085_ENABLE_LOGGING
  if (!enabled(level, category)) return;

  char buffer[kLineBuffer];
  int n = std::snprintf(buffer, sizeof(buffer), "[%s] ", levelTag(level));
  if (n < 0) {
    ++dropped_;
    return;
  }
  size_t used = static_cast<size_t>(n);
  if (used >= sizeof(buffer)) {
    ++dropped_;
    return;
  }

  va_list args;
  va_start(args, fmt);
  int m = std::vsnprintf(buffer + used, sizeof(buffer) - used, fmt, args);
  va_end(args);

  if (m < 0 || used + static_cast<size_t>(m) >= sizeof(buffer)) {
    // Formatting was truncated: keep the partial line but count it.
    buffer[sizeof(buffer) - 1] = '\0';
    ++dropped_;
  }
  sink_->write(buffer);
#else
  (void)level;
  (void)category;
  (void)fmt;
#endif
}

void Logger::hex(LogLevel level, uint16_t category, const char* tag,
                 const uint8_t* data, size_t len) {
#if BNO085_ENABLE_LOGGING
  if (!enabled(level, category)) return;

  // Bounded dump: first 24 bytes, then a summary. Avoids long blocking writes.
  constexpr size_t kMaxDump = 24;
  char buffer[kLineBuffer];
  int n = std::snprintf(buffer, sizeof(buffer), "[%s] %s [%u]",
                        levelTag(level), tag, static_cast<unsigned>(len));
  if (n < 0) {
    ++dropped_;
    return;
  }
  size_t used = static_cast<size_t>(n);
  const size_t dump = (len < kMaxDump) ? len : kMaxDump;
  for (size_t i = 0; i < dump && used + 4 < sizeof(buffer); ++i) {
    int w = std::snprintf(buffer + used, sizeof(buffer) - used, " %02X",
                          static_cast<unsigned>(data[i]));
    if (w <= 0) break;
    used += static_cast<size_t>(w);
  }
  if (dump < len && used + 5 < sizeof(buffer)) {
    std::snprintf(buffer + used, sizeof(buffer) - used, " ...");
  }
  sink_->write(buffer);
#else
  (void)level;
  (void)category;
  (void)tag;
  (void)data;
  (void)len;
#endif
}

} // namespace bno085
