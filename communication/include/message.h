#pragma once
#include <cstring>
#include <array>
#include <cstdint>
#include <sstream>
#include "logger.h"


namespace com {
  enum class message_type : uint8_t {
    MOTOR_COMMAND = 0x01,
  };
  template <typename M>
  class message {
  public:
    message(const M& _data, message_type _type)
      : data(_data), start_byte(0x7E), type(type), size(sizeof(M)), footer{ 0x7F, 0 } {
      footer.crc = calulate_crc();
      std::memcpy(bytes.data(), &data, sizeof(M));
    }
    message(const M& data);
    message();

    void add_data(const M& data) {
      this->data = data;
    }
    uint8_t calulate_crc() {
      uint8_t crc = 0;
      crc ^= start_byte;
      crc ^= static_cast<uint8_t>(type);
      char buffer[sizeof(M)];
      std::memcpy(buffer, &data, sizeof(M));
      for (size_t i = 0; i < sizeof(M); i++) {
        crc ^= buffer[i];
        crc = (crc << 1) | (crc >> 7);
      }
      crc ^= footer.end_byte;
      return crc;
    }
    uint8_t check_crc(const unsigned char[]) const {
      return 0;
    }

    M get_data() const {
      return data;
    }
    message_type get_message_type() const {
      return type;
    }
    std::array<uint8_t, sizeof(M)> get_bytes() const {
      return bytes;
    }
    uint8_t get_crc() const {
      return footer.crc;
    }
    uint8_t get_start_byte() const {
      return start_byte;
    }
    uint8_t get_end_byte() const {
      return footer.end_byte;
    }
    uint8_t get_size() const {
      return size;
    }
  private:
    M data;
    std::array<uint8_t, sizeof(M)> bytes{ 0 };
    uint8_t start_byte{ 0x7E };
    com::message_type type;

    uint8_t size = sizeof(M);
    struct footer {
      uint8_t end_byte{ 0x7F };
      uint8_t crc;
    }footer;
    const std::string tag = "Message";
  };

}