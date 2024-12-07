// include/utils/Logger.h
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <iostream>
#include <mutex>
#include <bitset>
#include <sstream>
#include <fstream>
#include <time.h>
#include <Config.h>



// Convenience macros - now with enabled checks
#define LOG_DEBUG(tag, msg)   \
  if(Logger::getInstance().isEnabled(Logger::Level::DEBUG)) \
    Logger::getInstance().log(Logger::Level::DEBUG, tag, msg)
#define LOG_INFO(tag, msg) \
    if(Logger::getInstance().isEnabled(Logger::Level::INFO)) \
        Logger::getInstance().log(Logger::Level::INFO, tag, msg)
#define LOG_WARNING(tag, msg) \
    if(Logger::getInstance().isEnabled(Logger::Level::WARNING)) \
        Logger::getInstance().log(Logger::Level::WARNING, tag, msg)
#define LOG_ERROR(tag, msg) \
    if(Logger::getInstance().isEnabled(Logger::Level::ERROR)) \
        Logger::getInstance().log(Logger::Level::ERROR, tag, msg)

#define LOG_TO_FILE(tag, msg) \
  Logger::getInstance().logToFile(tag, msg)\

class Logger {
public:
    enum class Level {
        DEBUG,    // Detailed debugging
        INFO,     // Normal events
        WARNING,  // Issues that need attention
        ERROR,    // Critical issues
        COUNT     // Used to size the bitset
    };

    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    // Enable specific log levels
    void enable(Level level) {
        enabledLevels.set(static_cast<size_t>(level), true);
    }

    // Disable specific log levels
    void disable(Level level) {
        enabledLevels.set(static_cast<size_t>(level), false);
    }

    // Check if a level is enabled
    bool isEnabled(Level level) const {
        return enabledLevels[static_cast<size_t>(level)];
    }

    // Enable all levels
    void enableAll() {
        enabledLevels.set();
    }

    // Disable all levels
    void disableAll() {
        enabledLevels.reset();
    }

    void log(Level level, const std::string tag, const std::string& message) {
        if (isEnabled(level)) {
            std::lock_guard<std::mutex> lock(mutex);
            std::string str = (level == Logger::Level::ERROR) ? "[ " + levelToString(level) + " ]": "";
            std::cout << getTimestamp() << str << "[ " << tag << "] " << message << std::endl;
        }
    }

    std::string getTimestamp() {
        time_t now = time(nullptr);
        struct tm* timeinfo = localtime(&now);

        char buffer[9];  // HH:MM:SS\0
        strftime(buffer, sizeof(buffer), "%H:%M:%S", timeinfo);

        std::stringstream ss;
        ss << buffer;

        // Get milliseconds using a simpler approach
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        int ms = ts.tv_nsec / 1000000;  // Convert nanoseconds to milliseconds

        // Format milliseconds with leading zeros
        char ms_buffer[5];
        snprintf(ms_buffer, sizeof(ms_buffer), ".%03d", ms);
        ss << ms_buffer;

        return ss.str();
    }
    Status logToFile(const std::string tag, const std::string& message) {
      std::lock_guard<std::mutex> lock(mutex);
      std::ofstream file("log.txt", std::ios::app);
      LOG_INFO("Logger", "Writing to log file");
      if (!file.is_open()) {
        LOG_ERROR("Logger", "Failed to open log file");
        return Status::ERROR;
      }

      file << getTimestamp() << "[" << tag << "] " << message << std::endl;
      file.flush();
      file.close();
      return Status::OK;
    }

private:
    Logger() {
        // By default, enable all except DEBUG
        enableAll();
        disable(Level::DEBUG);
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::bitset<static_cast<size_t>(Level::COUNT)> enabledLevels;
    std::mutex mutex;


//    const char* levelToString(Level level) {
//        switch (level) {
//            case Level::DEBUG:   return "DEBUG";
//            case Level::INFO:    return "INFO";
//            case Level::WARNING: return "WARNING";
//            case Level::ERROR:   return "ERROR";
//            default:            return "UNKNOWN";
//        }
//    }

    std::string levelToString(Level level) {
        switch (level) {
            case Level::DEBUG:   return "DEBUG";
            case Level::INFO:    return "INFO";
            case Level::WARNING: return "WARNING";
            case Level::ERROR:   return "ERROR";
            default:            return "UNKNOWN";
        }
    }

};




#endif // LOGGER_H