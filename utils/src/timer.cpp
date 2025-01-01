#include "timer.h"
#include <errno.h>

// Initialize the static application start time
const timespec __app_start_time = []() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts;
  }();


result<void> timer::start() {
  if (running) {
    return error<void>("Timer already running");
  }

  auto ts = timestamp();
  if (ts.has_error()) {
    return error<void>(ts.get_error_msg());
  }

  start_time = ts.get_value();
  running = true;
  return success();
}

result<void> timer::stop() {
  if (!running) {
    return error<void>("Timer not running");
  }

  auto ts = timestamp();
  if (ts.has_error()) {
    return error<void>(ts.get_error_msg());
  }

  end_time = ts.get_value();
  time_diff(start_time, end_time, elapsed_time);
  running = false;
  return success();
}

result<void> timer::reset() {
  running = false;
  start_time = { 0, 0 };
  end_time = { 0, 0 };
  elapsed_time = { 0, 0 };
  return success();
}

void timer::time_diff(const timespec& start, const timespec& end, timespec& result) {
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    result.tv_sec = end.tv_sec - start.tv_sec - 1;
    result.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else {
    result.tv_sec = end.tv_sec - start.tv_sec;
    result.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
}

result<double> timer::elapsed_ms() const {
  if (running) {
    return error<double>("Timer still running");
  }
  return success(elapsed_time.tv_sec * 1000.0 + elapsed_time.tv_nsec / 1000000.0);
}

result<double> timer::elapsed_us() const {
  if (running) {
    return error<double>("Timer still running");
  }
  return success(elapsed_time.tv_sec * 1000000.0 + elapsed_time.tv_nsec / 1000.0);
}

result<double> timer::elapsed_ns() const {
  if (running) {
    return error<double>("Timer still running");
  }
  return success(elapsed_time.tv_sec * 1000000000.0 + elapsed_time.tv_nsec);
}

result<double> timer::elapsed_s() const {
  if (running) {
    return error<double>("Timer still running");
  }
  return success(elapsed_time.tv_sec + elapsed_time.tv_nsec / 1000000000.0);
}

result<void> timer::timespec_sleep(const timespec& duration) {
  int ret = nanosleep(&duration, nullptr);
  if (ret != 0) {
    return error<void>("Sleep interrupted: " + std::string(strerror(errno)));
  }
  return success();
}

result<void> timer::sleep_ms(uint32_t milliseconds) {
  timespec ts{
      .tv_sec = milliseconds / 1000,
      .tv_nsec = (milliseconds % 1000) * 1000000
  };
  return timespec_sleep(ts);
}

result<void> timer::sleep_us(uint32_t microseconds) {
  timespec ts{
      .tv_sec = microseconds / 1000000,
      .tv_nsec = (microseconds % 1000000) * 1000
  };
  return timespec_sleep(ts);
}

result<void> timer::sleep_ns(uint32_t nanoseconds) {
  timespec ts{
      .tv_sec = nanoseconds / 1000000000,
      .tv_nsec = nanoseconds % 1000000000
  };
  return timespec_sleep(ts);
}
