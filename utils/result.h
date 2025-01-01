// result.h
#pragma once
#include <string>
#include "utils/logger.hpp"

/**
 * A utility class for handling operation results and errors in a type-safe manner.
 * Without using exceptions.
 * */

template<typename T>
class result;

template<typename T>
result<T> sucess(T value);

template<typename T>
result<T> error(std::string error_msg);

inline result<void> success();
inline result<void> error(std::string error_msg);


template<typename T>
class result {
public:
  friend result<T> success<T>(T value);
  friend result<T> error<T>(std::string error_msg);

  bool has_error() const {
    return !error_msg.empty();
  }

  T get_value() const {
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

private:
  T value;
  std::string error_msg;
};

template<typename T>
result<T> success(T value) {
  return result<T>(value, "");
}

template <typename T>
result<T> error(std::string error_msg) {
  return result<T>(T(), error_msg);
}



/* Void returning functions need a sepecial declaration  */
template<>
class result<void> {
public:
  friend result<void> success();
  friend result<void> error(std::string error_msg);

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

inline result<void> success() {
  return result<void>("");
}

inline result<void> error(std::string error_msg) {
  return result<void>(error_msg);
}