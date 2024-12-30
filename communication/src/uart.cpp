#include "uart.h"
#include "logger.h"
#include "result.h"
#include <filesystem>
#include "types.h"

namespace com {
  serial::serial(const std::string& _device_path, com::baudrate _baudrate)
    : device_path(_device_path), baudrate(_baudrate) {
    // default settings, only change this if you know what you are doing
    // auto result = open_port(device_path);
    // if (result.has_error()) {
    //   Logger::LOG_ERROR(tag, "Failed to open port with error %s for device %s",
    //     result.get_error_msg().c_str(), device_path.c_str());

    //     // check if file exist
    //   if (!std::filesystem::exists(device_path)) {
    //     Logger::LOG_ERROR(tag, "Device path does not exist %s", device_path.c_str());
    //     return;
    //   }
    //   // TODO: add retry logic
    //   return;
    // }

    // port_fd = result.get_value();
    // port_state = com::port_state::CONNECTED;

    // config.baudrate = baudrate;
    // // configure port settings
    // auto config_result = configure_port_settings();
    // if (config_result.has_error()) {
    //   Logger::LOG_ERROR("Failed to configure port settings with error %s", config_result.get_error_msg().c_str());
    //   return;
    // }

  }

  result<void> com::serial::configure_port_settings() {

    if (tcgetattr(port_fd, &tty) != 0) {
      return result<void>::error("Failed to get port settings : " + std::string(strerror(errno)));
    }

    tty.c_cflag &= ~PARENB; // clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // clear all the size bits
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // disable echo
    tty.c_lflag &= ~ECHOE; // disable erasure
    tty.c_lflag &= ~ECHONL; // disable new-line echo
    tty.c_lflag &= ~ISIG; // disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // disable any special handling of received bytes
    tty.c_iflag &= ~(ICRNL | INLCR); // disable translating carriage return to newline on input.
    tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return/line feed

    // set read timeout
    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 10; // 10 decisecond read timeout

    switch (baudrate) {
    case com::baudrate::B_9600:
      cfsetospeed(&tty, B9600);
      cfsetispeed(&tty, B9600);
      break;
    case com::baudrate::B_57600:
      cfsetospeed(&tty, B57600);
      cfsetispeed(&tty, B57600);
      break;
    case com::baudrate::B_115200:
      cfsetospeed(&tty, B115200);
      cfsetispeed(&tty, B115200);
      break;
    case com::baudrate::B_230400:
      cfsetospeed(&tty, B230400);
      cfsetispeed(&tty, B230400);
      break;
    case com::baudrate::B_460800:
      cfsetospeed(&tty, B460800);
      cfsetispeed(&tty, B460800);
      break;
    case com::baudrate::B_500000:
      cfsetospeed(&tty, B500000);
      cfsetispeed(&tty, B500000);
      break;
    case com::baudrate::B_576000:
      cfsetospeed(&tty, B576000);
      cfsetispeed(&tty, B576000);
    default:
      return result<void>::error("Invalid baudrate");
      break;
    }

    // save tty settings, also check for error
    if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
      return result<void>::error("Failed to set port settings : " + std::string(strerror(errno)));
    }
    return result<void>::success();

  }

  result<int> com::serial::open_port(const std::string& device_path) {

    int fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      // check if file exist
      return result<int>::error("Failed to open port, unknown error %s" + std::string(strerror(errno)));
    }
    return result<int>::success(fd);
  }

  serial::~serial() {
    if (com::port_state::CONNECTED == port_state) {
      disconnect();
    }

    if (port_fd > 0) {
      close(port_fd);
    }
  }
  result<void> serial::disconnect() {
    if (com::port_state::CONNECTED != port_state) {
      return result<void>::error("Port is not connected");
    }

    if (close(port_fd) < 0) {
      return result<void>::error("Failed to close port with error %s" + std::string(strerror(errno)));
    }

    port_state = com::port_state::DISCONNECTED;
    return result<void>::success();
  }

  result<void> com::serial::set_read_timeout(int32_t timeout_ms) {
    if (com::port_state::CONNECTED != port_state) {
      return result<void>::error("Port is not connected");
    }

    if (timeout_ms < 0) {
      return result<void>::error("Invalid timeout value");
    }

    tty.c_cc[VTIME] = timeout_ms / 100;
    tty.c_cc[VMIN] = 0;
    if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
      return result<void>::error("Failed to set read timeout with error %s" + std::string(strerror(errno)));
    }
    return result<void>::success();
  }

  result<void> com::serial::send(const std::array<uint8_t, MAX_MSG_SIZE>& data) {
    if (com::port_state::CONNECTED != port_state) {
      return result<void>::error("Port is not connected");
    }

    ssize_t bytes_written = write(port_fd, data.data(), data.size());
    if (bytes_written < 0) {
      return result<void>::error("Failed to write to port with error %s" + std::string(strerror(errno)));
    }

    return result<void>::success();
  }

  result<std::array<uint8_t, MAX_MSG_SIZE>> com::serial::receive() {
    if (com::port_state::CONNECTED != port_state) {
      return result<std::array<uint8_t, MAX_MSG_SIZE>>::error("Port is not connected");
    }

    std::array<uint8_t, MAX_MSG_SIZE> buffer;
    ssize_t bytes_read = read(port_fd, buffer.data(), buffer.size());
    if (bytes_read < 0) {
      return result<std::array<uint8_t, MAX_MSG_SIZE>>::error("Failed to read from port with error %s" + std::string(strerror(errno)));
    }
    if (bytes_read < 4) {
      return result<std::array<uint8_t, MAX_MSG_SIZE>>::error("Failed to read from port with error %s" + std::string(strerror(errno)));
    }

    return result<std::array<uint8_t, MAX_MSG_SIZE>>::success(buffer);
  }

}