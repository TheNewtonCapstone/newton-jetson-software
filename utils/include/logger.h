#pragma once
#include <cstring>
#include <string>
#include <iostream>
#include <mutex>
#include <bitset>
#include <sstream>
#include <fstream>

namespace newton
{
class Logger
{
public:
  enum class Level
  {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    COUNT
  };

  template <typename... Args>
  static void INFO(const std::string& tag, const char* format, Args... args)
  {
    auto& logger= newton::Logger::get_instance();
    auto level = newton::Logger::Level::INFO;
    if (logger.is_enabled(level))
    {
      logger.log_formatted(level, tag, format, std::forward<Args>(args)...);
    }
  }

  template <typename... Args>
  static void DEBUG(const std::string& tag, const char* format, Args... args)
  {
    auto& logger= newton::Logger::get_instance();
    auto level = newton::Logger::Level::DEBUG;
    if (logger.is_enabled(level))
    {
      logger.log_formatted(level, tag, format, std::forward<Args>(args)...);
    }
  }

  template <typename... Args>
  static void WARN(const std::string& tag, const char* format, Args... args)
  {
    auto& logger= newton::Logger::get_instance();
    auto level = newton::Logger::Level::WARN;
    if (logger.is_enabled(level))
    {
      logger.log_formatted(level, tag, format, std::forward<Args>(args)...);
    }
  } 
  template<typename... Args>
  static void ERROR(const std::string& tag, const char* msg, Args... args)
  {
    auto& logger= newton::Logger::get_instance();
    auto level = newton::Logger::Level::ERROR;
    if (logger.is_enabled(level))
    {
      logger.log_formatted(level, tag, msg, std::forward<Args>(args)...);
    }
  }

  // Configuration methods
  void set_logfile(const std::string& filename)
  {
    std::lock_guard<std::mutex> lock(mutex);
    logFile_ = filename;
  }

  void enable(Level level)
  {
    enabled_levels.set(static_cast<size_t>(level), true);
  }

  void disable(Level level)
  {
    enabled_levels.set(static_cast<size_t>(level), false);
  }

  bool is_enabled(Level level) const
  {
    return enabled_levels[static_cast<size_t>(level)];
  }

  void enable_all()
  {
    enabled_levels.set();
  }

  void disable_all()
  {
    enabled_levels.reset();
  }

  static Logger& get_instance()
  {
    static Logger instance;
    return instance;
  }

private:
  Logger()
  {
    enable_all();
    disable(Level::DEBUG);
  }

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  template <typename... Args>
  void log_formatted(Level level, const std::string& tag, const char* format, Args... args)
  {
    std::lock_guard<std::mutex> lock(mutex);

    // Format the message
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format, std::forward<Args>(args)...);

    std::stringstream ss;
    ss << get_timestamp();
    ss << " [ " << to_string(level) << " ]";
    ss << " [ " << tag << " ] ";
    ss << buffer;

    // Output to console
    std::cout << ss.str() << std::endl;

    // Output to file if configured
    if (!logFile_.empty())
    {
      std::ofstream file(logFile_, std::ios::app);
      if (file.is_open())
      {
        file << ss.str() << std::endl;
        file.flush();
      }
    }
  }

  std::string get_timestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    struct tm* timeinfo = localtime(&now_c);
    char buffer[24];  // Enough for "YYYY-MM-DD HH:MM:SS.mmm"
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);

    sprintf(buffer + strlen(buffer), ".%03ld", now_ms.count());

    return std::string(buffer);
  }
  std::string to_string(Level level)
  {
    switch (level)
    {
      case Level::DEBUG:
        return "DEBUG";
      case Level::INFO:
        return "INFO ";
      case Level::WARN:
        return "WARN ";
      case Level::ERROR:
        return "ERROR";
      default:
        return "UNKNOWN";
    }
  }

  std::bitset<static_cast<size_t>(Level::COUNT)> enabled_levels;
  std::mutex mutex;
  std::string logFile_;
};
}  // namespace newton
