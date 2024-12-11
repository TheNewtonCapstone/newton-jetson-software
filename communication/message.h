#pragma once
#include <cstring>
#include <array>
#include <cstdint>
#include <sstream>
#include "utils/logger.hpp"


namespace com {
  struct message {
    static constexpr size_t MAX_SIZE = 255;
    static constexpr size_t HEADER_SIZE = 3;
    static constexpr size_t FOOTER_SIZE = 2;
    static constexpr size_t BODY_SIZE = MAX_SIZE - HEADER_SIZE - FOOTER_SIZE;

    enum class device {
      JETSON = 0x01,
      ESP32_1 = 0x02,
      ESP32_2 = 0x03,
    };

    struct header {
      uint8_t start_byte{ 0x7E };
      device source;
      uint8_t message_type;
      // This is used for multi part 
      uint8_t squence_number;
    } header;
    struct footer {
      uint8_t end_byte;
      uint8_t crc;
    };

    std::array<uint8_t, BODY_SIZE> body;
  };

}