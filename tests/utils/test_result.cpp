#include <catch2/catch_test_macros.hpp>
#include "result.h"
#include "logger.h"
#include <string>

TEST_CASE("Result wrapper basic functionality", "[utils]") {
  SECTION("success with value") {
    auto res = success(42);
    REQUIRE_FALSE(res.has_error());
    REQUIRE(res.get_value() == 42);
    REQUIRE(res.get_error_msg().empty());
    REQUIRE(res.value_or(0) == 42);
  }

  SECTION("error state") {
    auto res = error<int>("test error");
    REQUIRE(res.has_error());
    REQUIRE(res.get_value() == 0);  // default constructed
    REQUIRE(res.get_error_msg() == "test error");
    REQUIRE(res.value_or(42) == 42);
  }

  SECTION("void result") {
    SECTION("success case") {
      auto res = success();
      REQUIRE_FALSE(res.has_error());
      REQUIRE(res.get_error_msg().empty());
    }

    SECTION("error case") {
      auto res = error("void error");
      REQUIRE(res.has_error());
      REQUIRE(res.get_error_msg() == "void error");
    }
  }

  SECTION("string result") {
    SECTION("success case") {
      auto res = success<std::string>("test string");
      REQUIRE_FALSE(res.has_error());
      REQUIRE(res.get_value() == "test string");
    }

    SECTION("error case") {
      auto res = error<std::string>("string error");
      REQUIRE(res.has_error());
      REQUIRE(res.get_value().empty());
      REQUIRE(res.value_or("default") == "default");
    }
  }
}

TEST_CASE("Result wrapper move semantics", "[utils]") {
  SECTION("moving large string") {
    std::string large_string(1000, 'a');
    auto original_size = large_string.size();

    auto res = success(std::move(large_string));
    REQUIRE(large_string.empty());  // moved from
    REQUIRE(res.get_value().size() == original_size);
  }
}

TEST_CASE("Result wrapper practical usage", "[utils]") {
  SECTION("division function") {
    auto divide = [](int a, int b) -> result<double> {
      if (b == 0) {
        Logger::LOG_ERROR("Test result", "Division by zero");
        return error<double>("Division by zero");
      }
      return success(static_cast<double>(a) / b);
      };

    SECTION("valid division") {
      auto res = divide(10, 2);
      REQUIRE_FALSE(res.has_error());
      REQUIRE(res.get_value() == 5.0);
    }

    SECTION("division by zero") {
      auto res = divide(10, 0);
      REQUIRE(res.has_error());
      REQUIRE(res.get_error_msg() == "Division by zero");
    }
  }

  SECTION("file operations") {
    auto open_file = [](const std::string& path) -> result<std::string> {
      std::ifstream file(path);
      if (!file) {
        return error<std::string>("Failed to open file");
      }
      return success<std::string>("File content");
      };

    SECTION("non-existent file") {
      auto res = open_file("non_existent.txt");
      REQUIRE(res.has_error());
      REQUIRE(res.get_error_msg() == "Failed to open file");
    }
  }
}