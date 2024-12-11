#pragma once
#include "communication/message.h"
#include "utils/logger.hpp"
#include "communication/transport.h"
#include <sstream>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include "utils/logger.hpp"
#include <thread>

namespace com {
  enum class uart_baudrate {
    B_9600 = B9600,
    B_57600 = B57600,
    B_115200 = B115200,
    B_230400 = B230400,
    B_460800 = B460800,
    B_500000 = B500000,
    B_576000 = B576000,
  };

  struct uart_settings {
    // Generic connection address/path
    std::string address;
    uart_baudrate baudrate;
    uint16_t timeout_ms;
  };

  class uart : public Transport {
  public:
    uart();
    ~uart() override {
      disconnect();
    }
    status connect() override {
      return connect(settings);
    }
    status connect(const uart_settings& settings);
    status disconnect() override;
    status send(const std::vector<uint8_t>& data) override;
    status receive(std::vector<uint8_t>& data) override;
  private:
    void read_thread();
  private:
    int file_desc;
    uart_settings settings;
    // struct that holds the configuration
    struct termios tty;
    std::mutex mutex;

  };
};