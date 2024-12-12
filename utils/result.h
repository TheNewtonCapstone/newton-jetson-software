// result.h
#pragma once
#include <string>
#include "utils/logger.hpp"

// Forward declaration of the template
template<typename T>
class result;

// Main template declaration
template<typename T>
class result {
public:
  static result<T> success(T value) {
    return result<T>(value, "");
  }

  static result<T> error(std::string error_msg) {
    return result<T>(T(), error_msg);
  }

  bool has_error() const {
    return !error_msg.empty();
  }

  bool is_success() const {
    return error_msg.empty();
  }

  T get_value() const {
    if (has_error()) {
      Logger::ERROR("result", "Error Accessing value in error state %s", error_msg.c_str());
    }
    return value;
  }

  std::string get_error_msg() const {
    return error_msg;
  }

private:
  result(T value, std::string error_msg)
    : value(std::move(value)), error_msg(std::move(error_msg)) {
  }

private:
  T value;
  std::string error_msg;
};

// Void specialization declaration
template<>
class result<void> {
public:
  static result<void> success() {
    return result<void>("");
  }
  static result<void> error(std::string error_msg) {
    return result<void>(error_msg);
  }

  bool has_error() const {
    return !error_msg.empty();
  };
  bool is_success() const {
    return error_msg.empty();
  }
  std::string get_error_msg() const {
    return error_msg;
  }

private:
  explicit result(std::string error_msg) : error_msg(std::move(error_msg)) {}

private:
  std::string error_msg;
};