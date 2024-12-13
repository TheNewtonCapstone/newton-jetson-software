#include "uart.h"
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include "logger.h"

namespace com {
  uart::uart() : file_desc(-1) {};
  uart::~uart() override {
    disconnect();
  }
  status uart::connect(const uart_settings& settings) {

   // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    file_desc = open(settings.address.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
      // Read in existing settings, and handle any error
    if (tcgetattr(file_desc, &tty) != 0) {
      Logger::ERROR("uart", "Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return status::ERROR;
    }

    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)


    // Set in/out baud rate to be 9600
    cfsetspeed(&tty, settings.baudrate);
    cfsetospeed(&tty, setting.baudrate);

     // c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
          // Only meaningful when port is set to non-canonical mode
          // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
          // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
          // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
          // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
          //                      after first character has elapsed
          // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
          // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode

    if (settings.timeout_ms == -1) {
        // Always wait for at least one byte, this could
        // block indefinitely
      tty.c_cc[VTIME] = 0;
      tty.c_cc[VMIN] = 1;
    }
    else if (settings.timeout_ms == 0) {
     // Setting both to 0 will give a non-blocking read
      tty.c_cc[VTIME] = 0;
      tty.c_cc[VMIN] = 0;
    }
    else if (settings.timeout_ms > 0) {
      const int deciseconds = (cc_t)setting.timeout_ms / 100;
      tty.c_cc[VTIME] = deciseconds;
      tty.c_cc[VMIN] = 0;
    }
  // Save tty settings, also checking for error
    if (tcsetattr(file_desc, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return status::ERROR;
    }

    return status::OK;
  }

  status uart::disconnect() {
    if (file_desc != -1) {
      close(file_desc);
      file_desc = -1;
    }
    return status::OK;
  }

  status uart::send(const std::vector<uint8_t>& data) {
    if (file_desc == -1) {
      Logger::ERROR("uart", "Not connected");
      return status::NOT_CONNECTED;
    }

    ssize_t bytes_written = write(file_desc, data.data(), data.size());
    if (bytes_written < 0) {
      Logger::ERROR("uart", "Error writing to UART: %s", strerror(errno));
      return status::ERROR;
    }

    return status::OK;
  }
  status uart::receive(std::vector<uint8_t>& data) {
    if (file_desc == -1) {
      Logger::ERROR("uart", "Not connected");
      return status::NOT_CONNECTED;
    }

    // Read up to 255 characters from the port if they are there
    uint8_t buf[255];
    ssize_t bytes_read = read(file_desc, &buf, sizeof(buf));
    if (bytes_read < 0) {
      Logger::ERROR("uart", "Error reading from UART: %s", strerror(errno));
      return status::ERROR;
    }

    data.assign(buf, buf + bytes_read);
    return status::OK;
  }
}