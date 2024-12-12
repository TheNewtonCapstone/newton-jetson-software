#include "utils/result.h"

template<typename T>
result<T>::result(T _value, std::string _error_msg)
  :value(std::move(_value)), error_msg(_error_msg) {
};


template<typename T>
bool result<T>::has_error() const {
  return !error_msg.empty();
}

template<typename T>
bool result<T>::is_success() const {
  return error_msg.empty();
}


template<typename T>
T result<T>::get_value() const {
  if (has_error()) {
    Logger::ERROR("result", "Error Accessing value in error state %s", error_msg.c_str());
  }
  return value;
}

template<typename T>
std::string result<T>::get_error_msg() const {
  return error_msg;
}

// void specialization
result<void> result<void>::success() {
  return result<void>("");
}

result<void> result<void>::error(std::string tag, std::string error_msg) {
  Logger::ERROR(tag.c_str(), error_msg.c_str());
  return result<void>(error_msg);
}

bool result<void>::has_error() const {
  return !error_msg.empty();
}

bool result<void>::is_success() const {
  return error_msg.empty();
}

std::string result<void>::get_error_msg() const {
  return error_msg;
}

result<void>::result(std::string error_msg) : error_msg(std::move(error_msg)) {
}