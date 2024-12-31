#include "base_msg.h" 
#include <cstring>

namespace com {
  namespace msg {
    base_msg::base_msg() {
      header.start_byte = START_BYTE;
      footer.end_byte = END_BYTE;
    }
    uint8_t base_msg::calculate_crc() const {
      uint8_t crc = 0;
      for (auto& byte : payload) {
        crc ^= byte;
      }
      return crc;
    }
    message_type base_msg::get_message_type() const {
      return static_cast<message_type>(header.message_type);
    }
    const std::vector<uint8_t>& base_msg::get_payload() const {
      return payload;
    }
    void base_msg::set_message_type(message_type type) {
      header.message_type = static_cast<uint8_t>(type);
    }
    void base_msg::set_payload(const std::vector<uint8_t>& _payload) {
      payload = _payload;
      footer.crc = calculate_crc();
    }
    void base_msg::hex_dump() {
      std::stringstream ss;
      for (auto& byte : payload) {
        ss << std::hex << static_cast<int>(byte) << " ";
      }
      Logger::LOG_INFO("Payload", ss.str().c_str());
    }
  }
}