#pragma once
#include <string>
#include "utils/logger.hpp"

template<typename T>
class result {
public:
  static result<T> success(T value) {
    return result<T>(value, "");
  }
  static result<T> error(std::string tag, std::string error_msg) {
    Logger::ERROR(tag, error_msg);
    return result<T>(T(), error_msg);
  }

  bool has_error() const;
  bool is_success() const;
  T get_value() const;
  std::string get_error_msg() const;

private:
  result(T value, std::string error_msg);
private:
  T value;
  std::string error_msg;
};

template<>
class result<void> {
public:
  static result<void> success() {
    return result<void>("");
  }

  static result<void> error(std::string tag, std::string error_msg) {
    Logger::ERROR(tag.c_str(), error_msg.c_str());
    return result<void>(error_msg);
  }

  bool has_error() const;

  bool is_success() const;

  std::string get_error_msg() const;

private:
  explicit result(std::string error_msg) : error_msg(std::move(error_msg)) {}

private:
  std::string error_msg;
};