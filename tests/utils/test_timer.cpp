#include <catch2/catch_test_macros.hpp>
#include "timer.h"
#include "logger.h"
#include <thread>
#include <chrono>

void sleep_for_ms(int milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// Test initialization
struct TestInit {
  TestInit() {
      // Enable all log levels for testing
    Logger::get_instance().enable_all();
    Logger::LOG_INFO("TimerTest", "Timer test suite starting");
  }

  ~TestInit() {
    Logger::LOG_INFO("TimerTest", "Timer test suite completed");
  }
};

static TestInit testInit;

TEST_CASE("Timer basic functionality", "[utils]") {
  Logger::LOG_INFO("TimerTest", "Starting basic functionality test case");

  SECTION("timer construction") {
    Logger::LOG_DEBUG("TimerTest", "Testing timer construction");
    timer t("test_timer");
    REQUIRE_FALSE(t.is_running());
    Logger::LOG_DEBUG("TimerTest", "Timer construction test passed");
  }

  SECTION("start and stop") {
    Logger::LOG_DEBUG("TimerTest", "Testing start and stop operations");
    timer t("test_timer");

    // Test successful start
    Logger::LOG_DEBUG("TimerTest", "Attempting to start timer");
    auto start_result = t.start();
    REQUIRE_FALSE(start_result.has_error());
    REQUIRE(t.is_running());
    Logger::LOG_DEBUG("TimerTest", "Timer started successfully");

    // Test starting an already running timer
    Logger::LOG_DEBUG("TimerTest", "Testing double start scenario");
    auto double_start = t.start();
    REQUIRE(double_start.has_error());
    REQUIRE(double_start.get_error_msg() == "Timer already running");
    Logger::LOG_DEBUG("TimerTest", "Double start properly rejected");

    // Test successful stop
    Logger::LOG_DEBUG("TimerTest", "Sleeping for 10ms before stopping timer");
    sleep_for_ms(10);
    auto stop_result = t.stop();
    REQUIRE_FALSE(stop_result.has_error());
    REQUIRE_FALSE(t.is_running());
    Logger::LOG_DEBUG("TimerTest", "Timer stopped successfully");

    // Test stopping an already stopped timer
    Logger::LOG_DEBUG("TimerTest", "Testing double stop scenario");
    auto double_stop = t.stop();
    REQUIRE(double_stop.has_error());
    REQUIRE(double_stop.get_error_msg() == "Timer not running");
    Logger::LOG_DEBUG("TimerTest", "Double stop properly rejected");
  }

  SECTION("elapsed time measurements") {
    Logger::LOG_DEBUG("TimerTest", "Testing elapsed time measurements");
    timer t("test_timer");

    // Test elapsed time on non-started timer
    auto elapsed = t.elapsed_ms();
    REQUIRE_FALSE(elapsed.has_error());
    REQUIRE(elapsed.get_value() == 0.0);
    Logger::LOG_DEBUG("TimerTest", "Initial elapsed time is zero as expected");

    // Test measuring a short duration
    Logger::LOG_DEBUG("TimerTest", "Starting timed section (100ms sleep)");
    t.start();
    sleep_for_ms(100);
    t.stop();
    Logger::LOG_DEBUG("TimerTest", "Timed section completed");

    // Check different time units
    SECTION("milliseconds") {
      auto ms = t.elapsed_ms();
      REQUIRE_FALSE(ms.has_error());
      REQUIRE(ms.get_value() >= 95.0);
      REQUIRE(ms.get_value() <= 150.0);
      Logger::LOG_INFO("TimerTest", "Measured time in ms: %.2f", ms.get_value());
    }

    SECTION("microseconds") {
      auto us = t.elapsed_us();
      REQUIRE_FALSE(us.has_error());
      REQUIRE(us.get_value() >= 95000.0);
      REQUIRE(us.get_value() <= 150000.0);
      Logger::LOG_INFO("TimerTest", "Measured time in us: %.2f", us.get_value());
    }

    SECTION("nanoseconds") {
      auto ns = t.elapsed_ns();
      REQUIRE_FALSE(ns.has_error());
      REQUIRE(ns.get_value() >= 95000000.0);
      REQUIRE(ns.get_value() <= 150000000.0);
      Logger::LOG_INFO("TimerTest", "Measured time in ns: %.2f", ns.get_value());
    }

    SECTION("seconds") {
      auto s = t.elapsed_s();
      REQUIRE_FALSE(s.has_error());
      REQUIRE(s.get_value() >= 0.095);
      REQUIRE(s.get_value() <= 0.150);
      Logger::LOG_INFO("TimerTest", "Measured time in s: %.3f", s.get_value());
    }
  }

  SECTION("timer reset") {
    Logger::LOG_DEBUG("TimerTest", "Testing timer reset functionality");
    timer t("test_timer");

    Logger::LOG_DEBUG("TimerTest", "Starting pre-reset measurement");
    t.start();
    sleep_for_ms(10);
    t.stop();

    auto pre_reset = t.elapsed_ms();
    REQUIRE_FALSE(pre_reset.has_error());
    REQUIRE(pre_reset.get_value() > 0.0);
    Logger::LOG_INFO("TimerTest", "Pre-reset measurement: %.2f ms", pre_reset.get_value());

    Logger::LOG_DEBUG("TimerTest", "Performing timer reset");
    auto reset_result = t.reset();
    REQUIRE_FALSE(reset_result.has_error());
    REQUIRE_FALSE(t.is_running());

    auto post_reset = t.elapsed_ms();
    REQUIRE_FALSE(post_reset.has_error());
    REQUIRE(post_reset.get_value() == 0.0);
    Logger::LOG_DEBUG("TimerTest", "Post-reset values verified");
  }
}

TEST_CASE("Timer sleep functions", "[utils]") {
  Logger::LOG_INFO("TimerTest", "Starting sleep functions test case");

  SECTION("sleep_ms accuracy") {
    Logger::LOG_DEBUG("TimerTest", "Testing millisecond sleep accuracy");
    auto start_time = std::chrono::steady_clock::now();
    auto result = timer::sleep_ms(100);
    auto end_time = std::chrono::steady_clock::now();

    REQUIRE_FALSE(result.has_error());

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count();
    Logger::LOG_INFO("TimerTest", "Requested sleep: 100ms, Actual sleep: %ldms", duration);

    REQUIRE(duration >= 95);
    REQUIRE(duration <= 150);
  }

  SECTION("sleep_us accuracy") {
    Logger::LOG_DEBUG("TimerTest", "Testing microsecond sleep accuracy");
    auto start_time = std::chrono::steady_clock::now();
    auto result = timer::sleep_us(100000);
    auto end_time = std::chrono::steady_clock::now();

    REQUIRE_FALSE(result.has_error());

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time).count();
    Logger::LOG_INFO("TimerTest", "Requested sleep: 100000us, Actual sleep: %ldus", duration);

    REQUIRE(duration >= 95000);
    REQUIRE(duration <= 150000);
  }

  SECTION("sleep_ns accuracy") {
    Logger::LOG_DEBUG("TimerTest", "Testing nanosecond sleep accuracy");
    auto start_time = std::chrono::steady_clock::now();
    auto result = timer::sleep_ns(100000000);
    auto end_time = std::chrono::steady_clock::now();

    REQUIRE_FALSE(result.has_error());

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      end_time - start_time).count();
    Logger::LOG_INFO("TimerTest", "Requested sleep: 100000000ns, Actual sleep: %ldns", duration);

    REQUIRE(duration >= 95000000);
    REQUIRE(duration <= 150000000);
  }
}

TEST_CASE("Timer application time functions", "[utils]") {
  Logger::LOG_INFO("TimerTest", "Starting application time functions test case");

  SECTION("app time increases monotonically") {
    Logger::LOG_DEBUG("TimerTest", "Testing monotonic increase of app time");
    auto time1 = timer::apptime_ms();
    sleep_for_ms(10);
    auto time2 = timer::apptime_ms();

    REQUIRE_FALSE(time1.has_error());
    REQUIRE_FALSE(time2.has_error());
    REQUIRE(time2.get_value() > time1.get_value());
    Logger::LOG_INFO("TimerTest", "Time difference: %.2f ms",
      time2.get_value() - time1.get_value());
  }

  SECTION("different time units consistency") {
    Logger::LOG_DEBUG("TimerTest", "Testing consistency across time units");
    auto s = timer::apptime_s();
    auto ms = timer::apptime_ms();
    auto us = timer::apptime_us();
    auto ns = timer::apptime_ns();

    REQUIRE_FALSE(s.has_error());
    REQUIRE_FALSE(ms.has_error());
    REQUIRE_FALSE(us.has_error());
    REQUIRE_FALSE(ns.has_error());

    Logger::LOG_INFO("TimerTest", "App time in different units - s: %.3f, ms: %.3f, us: %.3f, ns: %ld",
      s.get_value(), ms.get_value(), us.get_value(), ns.get_value());

    REQUIRE(ms.get_value() >= s.get_value() * 1000.0);
    REQUIRE(us.get_value() >= ms.get_value() * 1000.0);
    REQUIRE(ns.get_value() >= us.get_value() * 1000.0);
    Logger::LOG_DEBUG("TimerTest", "Time unit consistency verified");
  }
}