#pragma once
#include <time.h>
#include <stdint.h>
#include "result.h"
#include "logger.h"

static const timespec __app_start_time;

class timer {
public:
  explicit timer(const std::string& _name);

  result<void> start();
  result<void> stop();

  result<void> reset();

  result<double> elapsed_ms() const;
  result<double> elapsed_us() const;
  result<double> elapsed_ns() const;
  result<double> elapsed_s() const;

  // Check if timer is running
  bool is_running() const { return running; } // not used for now but can be useful for thread safety

  static result<void> sleep_ms(uint32_t milliseconds);
  static result<void> sleep_us(uint32_t microseconds);

  // Get current time from an inline call 
  static inline result<timespec> timestamp() {
    timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
      return error<timespec>("Failed to get timestamp");
    }
    return success(ts);
  }

  static inline result<double> timestamp_ms() {
    auto ts = timestamp();
    if (ts.has_error()) {
      return error<double>(ts.get_error_msg());
    }
    const timespec& time = ts.get_value();
    return success<double>(time.tv_sec * 1000.0 + time.tv_nsec / 1000000.0);
  }

  static result<void> sleep_ns(uint32_t nanoseconds) {
    timespec ts;
    ts.tv_sec = nanoseconds / 1000000000;
    ts.tv_nsec = nanoseconds % 1000000000;
    return timespec_sleep(ts);
  }

  // Get time since application start
  static result<double> apptime_s() {
    auto ts = timestamp();
    if (ts.has_error()) {
      return error<double>(ts.get_error_msg());
    }

    timespec diff;
    time_diff(__app_start_time, ts.get_value(), diff);
    return success(diff.tv_sec + diff.tv_nsec / 1000000000.0);
  }
  static result<double> apptime_ms() {
    auto ts = timestamp();
    if (ts.has_error()) {
      return error<double>(ts.get_error_msg());
    }

    timespec diff;
    time_diff(__app_start_time, ts.get_value(), diff);
    return success(diff.tv_sec * 1000.0 + diff.tv_nsec / 1000000.0);
  }
  static result<double> apptime_us() {
    auto ts = timestamp();
    if (ts.has_error()) {
      return error<double>(ts.get_error_msg());
    }

    timespec diff;
    time_diff(__app_start_time, ts.get_value(), diff);
    return success(diff.tv_sec * 1000000.0 + diff.tv_nsec / 1000.0);
  }
  static result<long> apptime_ns() {
    auto ts = timestamp();
    if (ts.has_error()) {
      return error<long>(ts.get_error_msg());
    }

    timespec diff;
    time_diff(__app_start_time, ts.get_value(), diff);
    return success(diff.tv_sec * 1000000000 + diff.tv_nsec);
  }

private:
  /*This is a little HACK that im using cause im too lazy to refactor the time_diff implementationt below which has result as a pass by refernce in
  instead of a return if time allows it will be refactored*/
  static result<timespec> time_diff(const timespec& start, const timespec& end) {
    timespec result;
    time_diff(start, end, result);
    return success(result);
  }
  static void time_diff(const timespec& start, const timespec& end, timespec& result) {
    if ((end.tv_nsec - start.tv_nsec) < 0) {
      result.tv_sec = end.tv_sec - start.tv_sec - 1;
      result.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    }
    else {
      result.tv_sec = end.tv_sec - start.tv_sec;
      result.tv_nsec = end.tv_nsec - start.tv_nsec;
    }


  }
  static result<void> timespec_sleep(const timespec& duration);

  timespec start_time;
  timespec end_time;
  timespec elapsed_time;
  bool running;
  std::string timer_name;
  static constexpr const char* TAG = "Timer";
};