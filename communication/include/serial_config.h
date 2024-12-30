#ifndef SERIAL_CONFIG_H
#define SERIAL_CONFIG_H

#include "logger.h"
#include "message.h"
#include "transport.h"
#include <sstream>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <thread>

namespace com{
    enum class baudrate{
      B_9600 = B9600,
      B_57600 = B57600,
      B_115200 = B115200,
      B_230400 = B230400,
      B_460800 = B460800,
      B_500000 = B500000,
      B_576000 = B576000,
    };
    enum class data_bits{
      /*
         Set how many data bits are transmitted per byte across the serial port.
          The most common setting here is 8 (CS8).
          Definitely use this if you are unsure.
       */
      BITS_5 = CS5, // 5 bits per byte
      BITS_6 = CS6,
      BITS_7 = CS7,
      BITS_8 = CS8,
    };

    enum class parity{
      /*
         Parity is a method used to detect errors in transmitted data.
         It is a single bit that is added to the data being transmitted.
         The parity bit is set to 1 or 0 depending on the number of 1s in the data.
         If the number of 1s is even, the parity bit is set to 0.
         If the number of 1s is odd, the parity bit is set to 1.
         The receiver can then check the parity bit to see if the data is correct.
       */
      NONE = 0, // No parity bit
      ODD = PARODD, // Odd parity bit
      EVEN = PARENB, // Even parity bit
    };

    enum class stop_bits{
      /*
         Stop bits are used to signal the end of a data packet.
         They are used to ensure that the data is transmitted correctly.
         The most common setting is 1 stop bit.
       */
      ONE = CSTOPB, // 1 stop bit
      TWO = 0, // 2 stop bits
    };

    enum class flow_control{
      /*
         Flow control is a method used to prevent data loss when data is being transmitted between devices.
         It is used to control the flow of data between the transmitter and the receiver.
         There are two types of flow control: hardware flow control and software flow control.
         Hardware flow control uses the RTS (Request to Send) and CTS (Clear to Send) signals to control the flow of data.
         Software flow control uses XON and XOFF characters to control the flow of data.
       */
      NONE,
      HARDWARE,
      SOFTWARE,
//      NONE = 0, // No flow control
//      HARDWARE = CRTSCTS, // Hardware flow control
//      SOFTWARE = IXON | IXOFF | IXANY, // Software flow control
    };

    enum class port_state {
      DISCONNECTED,
      CONNECTED,
    };

    struct port_settings {
      std::string device_path;
      com::baudrate baudrate;
      com::data_bits data_bits;
      com::parity parity;
      com::stop_bits stop_bits;
      com::flow_control flow_control;
      bool echo_enabled;
      int32_t read_timeout_ms;
    };

    struct termios2 {
      tcflag_t c_iflag; /* input mode flags */
      tcflag_t c_oflag; /* output mode flags */
      tcflag_t c_cflag; /* control mode flags */
      tcflag_t c_lflag; /* local mode flags */
      cc_t c_line; /* line discipline */
      cc_t c_cc[19]; /* control characters */
    };
};



#endif //SERIAL_CONFIG_H
