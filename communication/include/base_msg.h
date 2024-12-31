#pragma once
#include <cstring>
#include <array>
#include <vector>
#include <cstdint>
#include <sstream>
#include "logger.h"

namespace com {
  namespace msg {

    enum class message_type : uint8_t {
      MOTOR_COMMAND = 0x01,
    };
    class base_msg {
    public:
      base_msg();
      virtual ~base_msg() = default;
      uint8_t calculate_crc() const;

      message_type get_message_type() const;
      const std::vector<uint8_t>& get_payload() const;
      uint8_t get_crc() const { return footer.crc; }

      void set_message_type(message_type type);
      void set_payload(const std::vector<uint8_t>& payload);

    public:
      static constexpr uint8_t START_BYTE = 's';
      static constexpr uint8_t END_BYTE = 'e';
      static constexpr uint8_t MAX_MSG_SIZE = 255;
    protected:
      std::vector<uint8_t> payload;
      struct header {
        uint8_t start_byte;
        uint8_t message_type;
      }header;
      struct footer {
        uint8_t crc;
        uint8_t end_byte;
      } footer;
    };



  }
}