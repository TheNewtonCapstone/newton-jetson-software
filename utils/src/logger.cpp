#include "logger.h"
#include "time.h"

std::string Logger::get_timestamp() {
  auto ts = timer::apptime_ns();
  if (ts.has_error()) {
      // use cpp std's timer as fallback
    return get_chrono_timestamp();
  }
  uint32_t time = (uint32_t)ts.get_value();
  uint32_t seconds = time / 1000000000;
  uint32_t milliseconds = (time % 1000000000) / 1000000;
  uint32_t microseconds = (time % 1000000) / 1000;
  uint32_t nanoseconds = time % 1000;

  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u:%02u", seconds, milliseconds, microseconds, nanoseconds);
  return std::string(buffer);
}