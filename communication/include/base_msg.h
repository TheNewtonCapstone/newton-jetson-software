#pragma once
#include <cstring>
#include <array>
#include <algorithm>
#include "result.h"
#include "logger.h"

namespace com {

  enum class MessageType : uint8_t {
    SYNC = 0x00,
    MOTOR_COMMAND = 0x01,
    MOTOR_STATE = 0x02,
    RESYNC = 0x03
  };

  enum class ErrorCode : uint8_t {
    NONE = 0x00,
    TIMING_ERROR = 0x01,
    BUFFER_OVERFLOW = 0x02,
    INVALID_COMMAND = 0x03,
    MOTOR_FAULT = 0x04,
    SYNC_LOST = 0x05,
    CRC_ERROR = 0x06,
    EMERGENCY_STOP = 0x07
  };

#pragma pack(push, 1)
  struct MessageHeader {
    uint8_t start_byte;
    uint8_t message_type;
    uint32_t timestamp_us;
    uint8_t source_id;
    uint8_t target_id;
  };

  struct MessageFooter {
    uint16_t crc;
    uint8_t end_byte;
  };
#pragma pack(pop)

  class Message {
  public:
    static constexpr uint8_t START_BYTE = 's';
    static constexpr uint8_t END_BYTE = 'e';
    static constexpr uint16_t MAX_PAYLOAD_SIZE = 255;
    static constexpr uint8_t HEADER_SIZE = sizeof(MessageHeader);
    static constexpr uint8_t FOOTER_SIZE = sizeof(MessageFooter);
    static constexpr uint16_t MAX_MSG_SIZE = MAX_PAYLOAD_SIZE + HEADER_SIZE + FOOTER_SIZE;

    explicit Message(uint32_t timestamp_us) {
      header.start_byte = START_BYTE;
      header.timestamp_us = timestamp_us;
      header.source_id = 0x00;  // Jetson ID
      header.target_id = 0x01;  // ESP32 ID
      footer.end_byte = END_BYTE;
      std::memset(payload.data(), 0, MAX_PAYLOAD_SIZE);
    }

    uint8_t calculate_crc() const {
      uint8_t crc = 0;
      for (const auto& byte : payload) {
        crc ^= byte;
      }
      return crc;
    }

    result<void> set_message_type(MessageType type) {
      header.message_type = static_cast<uint8_t>(type);
      return success();
    }

    result<void> set_payload(const uint8_t* data, uint16_t length) {
      if (length > MAX_PAYLOAD_SIZE) {
        return error("Payload size exceeds maximum");
      }
      std::memcpy(payload.data(), data, length);
      footer.crc = calculate_crc();
      return success();
    }

    bool is_valid() const {
      return header.start_byte == START_BYTE &&
        footer.end_byte == END_BYTE &&
        footer.crc == calculate_crc();
    }

    uint16_t get_full_message(uint8_t* buffer, uint16_t buffer_size) const {
      if (buffer_size < MAX_MSG_SIZE) {
        return 0;
      }

      uint16_t offset = 0;

      // Copy header
      std::memcpy(buffer + offset, &header, HEADER_SIZE);
      offset += HEADER_SIZE;

      // Copy payload
      std::memcpy(buffer + offset, payload.data(), MAX_PAYLOAD_SIZE);
      offset += MAX_PAYLOAD_SIZE;

      // Copy footer
      std::memcpy(buffer + offset, &footer, FOOTER_SIZE);
      offset += FOOTER_SIZE;

      return offset;
    }

    bool parse_message(const uint8_t* buffer, uint16_t length) {
      if (length < (HEADER_SIZE + FOOTER_SIZE)) {
        return false;
      }

      // Copy header
      std::memcpy(&header, buffer, HEADER_SIZE);

      // Copy payload
     uint16_t payload_length = std::min<uint16_t>(
    static_cast<uint16_t>(length - HEADER_SIZE - FOOTER_SIZE), 
    MAX_PAYLOAD_SIZE
      ); 
    std::memcpy(payload.data(), buffer + HEADER_SIZE, payload_length);

      // Copy footer
      std::memcpy(&footer, buffer + HEADER_SIZE + payload_length, FOOTER_SIZE);

      return is_valid();
    }

    MessageType get_message_type() const {
      return static_cast<MessageType>(header.message_type);
    }

    uint32_t get_timestamp() const {
      return header.timestamp_us;
    }

    const std::array<uint8_t, MAX_PAYLOAD_SIZE>& get_payload() const {
      return payload;
    }

  private:
    MessageHeader header;
    std::array<uint8_t, MAX_PAYLOAD_SIZE> payload;
    MessageFooter footer;
    static constexpr const char* TAG = "Message";
  };

} 