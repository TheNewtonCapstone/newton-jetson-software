#include "timer.h"
#include <errno.h>

// Initialize the static application start time
const timespec __app_start_time = []() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts;
  }();

timer::timer(const std::string& _name)
  : running(false)
  , timer_name(_name) {
  // Initialize all timespec structs to zero
  start_time = { 0, 0 };
  end_time = { 0, 0 };
  elapsed_time = { 0, 0 };
}

result<void> timer::start() {
  if (running) {
    return error("Timer already running");
  }

  auto ts = timestamp();
  if (ts.has_error()) {
    return error(ts.get_error_msg());
  }

  start_time = ts.get_value();
  running = true;
  return success();
}

result<void> timer::stop() {
  if (!running) {
    return error("Timer not running");
  }

  auto ts = timestamp();
  if (ts.has_error()) {
    return error(ts.get_error_msg());
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
    return error("Sleep interrupted: " + std::string(strerror(errno)));
  }
  return success();
}
