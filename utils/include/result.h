// result.h
#pragma once
#include <string>
#include "logger.h"

/**
 * A utility class for handling operation results and errors in a type-safe manner.
 * Without using exceptions.
 * */

template<typename T>
class result;


template<typename T>
class result {
public:

  static result<T> success(T value){
    return result<T>(value, "");
  }

  static result<T> error(std::string error_msg) {
    return result<T>(error_msg);
  }

  bool has_error() const {
    return !error_msg.empty();
  }

  T get_value() {
    return value;
  }

  std::string get_error_msg() const {
    return error_msg;
  }

  T value_or(T default_value) const {
    return has_error() ? default_value : value;
  }

private:
  result(T value, std::string error_msg)
    : value(std::move(value)), error_msg(std::move(error_msg)) {
  }
  result(std::string error_msg) : error_msg(std::move(error_msg)) {}

private:
  T value;
  std::string error_msg;
};



/* Void returning functions need a sepecial declaration  */
template<>
class result<void> {
public:
  static result<void> success() {
    return result<void>("");
  }
  static result<void> error(std::string tag, std::string error_msg) {
    N_LOG_ERROR(tag, error_msg.c_str());
    return result<void>(error_msg);
  }
  
  bool has_error() const {
    return !error_msg.empty();
  };

  std::string get_error_msg() const {
    return error_msg;
  }

private:
  explicit result(std::string error_msg) : error_msg(std::move(error_msg)) {}

private:
  std::string error_msg;
};

