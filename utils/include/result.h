/**
 * @file result.h
 * @brief Provides a type-safe error handling mechanism without exceptions
 * 
 * This header defines the result<T> template class which implements the Result pattern.
 * It provides a utility class for handling operation results and errors in a type-safe manner.
 * The class returns either a success value or an error message. 
 *This provides explicit error handling without the overhead of exception handling.
 */


#pragma once
#include <string>
#include "logger.h"

 using newton::Logger;

template<typename T> class result;
/**
 * @brief A template class for handling operation results with error checking
 * 
 * @tparam T The type of the success value
 * 
 * This class implements the Result pattern, providing a way to return either
 * a success value of type T or an error message from functions. It enforces
 * explicit error handling while avoiding the performance costs and complexity
 * of exception handling.
 */

template<typename T> class result {
public:
  /**
    * @brief Creates a successful result containing the given value
    * 
    * @param value The success value to store
    * @return result<T> A result object containing the success value 
  */
  static result<T> success(T value){
    return result<T>(value, "");
  }

/**
  * @brief Creates a failed result and logs the error
  * 
  * @param tag Logger tag for categorizing the error
  * @param format Printf-style format string for the error message
  * @param args Variable arguments for the format string
  * @return result<T> A result object containing the formatted error message
  */
  template<typename... Args>
  static result<T> error(std::string tag, const char* format, Args... args) {
    N_LOG_ERROR(tag, format, args...);
    return result<T>(std::string(format));
  }

  /**
    * @brief Creates a failed result containing the given error message
    * 
    * @param error_msg The error message to store
    * @return result<T> A result object containing the error message
  */
  static result<T> error(std::string error_msg) {
    return result<T>(error_msg);
  }


  /**
    * @brief Checks if the result contains an error
    * 
    * @return true if the result contains an error, false otherwise
  */
  bool has_error() const {
    return !error_msg.empty();
  }

  /**
    * @brief Retrieves the success value or returns a default if there is an error
    *  
    * @param default_value Value to return if result contains an error
    * @return T The success value if present, otherwise the default value
  */
  T get_value() {
    return value;
  }

  
  /**
    * @brief Retrieves the error message if the result contains an error
    * 
    * @return std::string The error message
  */
  std::string get_error_msg() const {
    return error_msg;
  }

  /**
    * @brief Retrieves the success value or returns a default if there is an error
    * 
    * @param default_value Value to return if result contains an error
    * @return T The success value if present, otherwise the default value
  */

  T value_or(T default_value) const {
    return has_error() ? default_value : value;
  }

private:
  /**
    * @brief Constructs a successful result containing the given value
    * 
    * @param value The success value to store
    * @param error_msg The error message to store
  */
  result(T value, std::string error_msg)
    : value(std::move(value)), error_msg(std::move(error_msg)) {
  }

  /**
    * @brief Constructs a failed result containing the given error message
    * 
    * @param error_msg The error message to store
  */
  result(std::string error_msg) : error_msg(std::move(error_msg)) {}

private:
  T value;
  std::string error_msg;
};



/* Void returning functions need a sepecial declaration  */
/**
 * @brief A template class for handling operation results with error checking
 * 
 * This specialization of the result class is used for functions that do not return a value. (void functions)
 * It provides a way to return either a success value or an error message from functions.
 */
template<>
class result<void> {
public:


  /**
    * @brief Creates a successful result
    * 
    * @return result<void> A result object indicating success
  */
  static result<void> success() {
    return result<void>("");
  }
  /**
    * @brief Creates a failed result and logs the error
    * 
    * @param tag Logger tag for categorizing the error
    * @param format Printf-style format string for the error message
    * @param args Variable arguments for the format string
    * @return result<void> A result object containing the formatted error message
  */
  template<typename... Args>
  static result<void> error(std::string tag, const char* format, Args... args) {
    Logger::ERROR(tag, format, args...);
    return result<void>(std::string(format));
  }

  /**
    * @brief Creates a failed result containing the given error message, without logging it
    * 
    * @param error_msg The error message to store
    * @return result<void> A result object containing the error message
  */
  static result<void> error(std::string error_msg) {
    return result<void>(error_msg);
  }

  /**
    * @brief Checks if the result contains an error
    * 
    * @return true if the result contains an error, false otherwise
  */

  bool has_error() const {
    return !error_msg.empty();
  };

  /**
    * @brief Retrieves the error message if the result contains an error
    * 
    * @return std::string The error message
  */
  std::string get_error_msg() const {
    return error_msg;
  }

private:
 /**
    * @brief Constructs a successful result
    * 
    * @param error_msg The error message to store
  */
  explicit result(std::string error_msg) : error_msg(std::move(error_msg)) {}

private:
  std::string error_msg;
};

