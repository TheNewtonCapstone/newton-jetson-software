#pragma once
#include "logger.h"
#include "base_msg.h"
#include <vector>
#include <sstream>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <thread>
#include "serial_config.h"
#include "result.h"
namespace com {
  static constexpr size_t MAX_MSG_SIZE = 255;
  class serial {
  public:
    serial(const std::string& device_path, baudrate baudrate);
    serial();
    ~serial();

    result<void> set_read_timeout(int32_t timeout_ms);

    result<int> connect(const std::string& device_path);
    result<void> disconnect();

    result<void> send(const std::array<uint8_t, MAX_MSG_SIZE>& data);
    result<std::array<uint8_t, MAX_MSG_SIZE>> receive();

    bool is_connected() const;

  private:
    // TODO : implement  a thread that reads and writes to the uart
    result <void> configure_port_settings();
  private:
    std::string device_path;
    com::baudrate baudrate;
    com::port_state port_state = port_state::DISCONNECTED;
    int8_t port_fd;

    // struct that holds the configuration
    struct termios tty;
    serial_config config;
    const std::string tag = "UART";

  };
};