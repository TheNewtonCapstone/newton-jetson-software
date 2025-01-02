#pragma once
#include <cassert>
#include <cstring>
#include <array>
#include <algorithm>
#include "logger.h"

enum class MessageType : uint8_t {
  SYNC = 0x00,
  MOTOR_COMMAND = 0x01,
  MOTOR_STATE = 0x02,
  RESYNC = 0x03
};

enum class code : uint8_t {
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
struct MotorCommand {
  uint8_t motor_id;
  uint8_t motor_speed_x;
  uint8_t motor_speed_y;
  uint8_t motor_speed;
};

struct MotorState {
  uint8_t motor_id;
  uint8_t current_speed;
  uint8_t error_flags;
};

struct Sync {
  uint32_t sync_counter;
};

struct MessageHeader {
  uint8_t frame_start;
  uint32_t timestamp_ns;
  MessageType msg_type;
  uint32_t payload_length;
} __attribute__((packed));

struct MessageFooter {
  uint16_t checksum; // 2 bytes
  uint8_t frame_end;
} __attribute__((packed));

struct Message {
  static constexpr uint8_t FRAME_START_MARKER = 0xAA;
  static constexpr uint8_t FRAME_END_MARKER = 0xFF;
  static constexpr uint8_t BUFER_CAPACITY = 255;

  // sizes 
  static const uint8_t HEADER_SIZE = sizeof(MessageHeader);
  static const uint8_t FOOTER_SIZE = sizeof(MessageFooter);
  static constexpr size_t MAX_PAYLOAD_LENGTH = BUFER_CAPACITY - HEADER_SIZE - FOOTER_SIZE;

  // buffer offsets 
  static constexpr size_t FRAME_START_OFFSET = 0;
  static constexpr size_t HEADER_OFFSET = FRAME_START_OFFSET;
  static constexpr size_t PAYLOAD_OFFSET = HEADER_OFFSET + HEADER_SIZE;

  size_t total_length = 0;
  MessageHeader header;
  uint8_t payload[BUFER_CAPACITY];
  MessageFooter footer;

  Message(uint32_t _timestamp_us, MessageType _type) {
    header.timestamp_ns = _timestamp_us;
    header.msg_type = _type;
    memset(payload, 0, BUFER_CAPACITY);
  }

  int serialize(const void* data, uint8_t length) {
    assert(length < MAX_PAYLOAD_LENGTH && "Payload is too large");

    // Set the header fields
    header.frame_start = FRAME_START_MARKER;
    header.payload_length = length;
    assert(header.timestamp_ns > 0 && "Timestamp not set");

    // header
    memcpy(payload + HEADER_OFFSET, &header, HEADER_SIZE);

    // payload
    memcpy(payload + PAYLOAD_OFFSET, data, length);

    // Calculate checksum over payload only
    footer.checksum = calculate_crc();
    footer.frame_end = FRAME_END_MARKER;

    // Write footer
    size_t footer_offset = PAYLOAD_OFFSET + length;
    memcpy(payload + footer_offset, &footer, FOOTER_SIZE);

    total_length = HEADER_SIZE + length + FOOTER_SIZE;
    return total_length;
  }

  uint16_t calculate_crc() {
    uint16_t crc = 0;
    const uint8_t* payload_data = payload + PAYLOAD_OFFSET;
    for (size_t i = 0; i < header.payload_length; i++) {
      crc ^= payload_data[i];
    }
    return crc;
  }

  MessageHeader get_header() {
    MessageHeader hdr;
    memcpy(&hdr, payload + HEADER_OFFSET, HEADER_SIZE);
    return hdr;
  }

  MessageFooter get_footer() {
    MessageFooter ftr;
    size_t footer_offset = PAYLOAD_OFFSET + header.payload_length;
    memcpy(&ftr, payload + footer_offset, FOOTER_SIZE);
    return ftr;
  }

  void deserialize(void* msg_struct, uint8_t length) {
    assert(length <= header.payload_length && "Requested length larger than payload");
    memcpy(msg_struct, payload + PAYLOAD_OFFSET, length);
  }

  void hex_dump() {
    printf("\nMessage Hex Dump:\n");
    printf("Total Length: %zu bytes\n", total_length);

    printf("Header (%u bytes):", HEADER_SIZE);
    for (size_t i = 0; i < HEADER_SIZE; i++) {
      if (i % 8 == 0) printf("\n  ");
      printf("%02X ", payload[HEADER_OFFSET + i]);
    }

    printf("\nPayload (%u bytes):", header.payload_length);
    for (size_t i = 0; i < header.payload_length; i++) {
      if (i % 8 == 0) printf("\n  ");
      printf("%02X ", payload[PAYLOAD_OFFSET + i]);
    }

    printf("\nFooter (%u bytes):", FOOTER_SIZE);
    size_t footer_offset = PAYLOAD_OFFSET + header.payload_length;
    for (size_t i = 0; i < FOOTER_SIZE; i++) {
      if (i % 8 == 0) printf("\n  ");
      printf("%02X ", payload[footer_offset + i]);
    }
    printf("\n");
  }

  void print_debug() {
    printf("\n=== Message Debug Information ===\n");
    printf("Structure:\n");
    printf("  Total Length: %zu bytes\n", total_length);
    printf("  Header Size: %u bytes\n", HEADER_SIZE);
    printf("  Payload Length: %u bytes\n", header.payload_length);
    printf("  Footer Size: %u bytes\n", FOOTER_SIZE);

    printf("\nHeader Details:\n");
    printf("  Frame Start: 0x%02X %s\n", header.frame_start,
      header.frame_start == FRAME_START_MARKER ? "(Valid)" : "(Invalid)");
    printf("  Timestamp: %u ns\n", header.timestamp_ns);
    printf("  Type: 0x%02X\n", static_cast<uint8_t>(header.msg_type));

    printf("\nFooter Details:\n");
    MessageFooter ftr = get_footer();
    printf("  Checksum: 0x%04X\n", ftr.checksum);
    printf("  Frame End: 0x%02X %s\n", ftr.frame_end,
      ftr.frame_end == FRAME_END_MARKER ? "(Valid)" : "(Invalid)");

    printf("\nBuffer Content:\n");
    hex_dump();
    printf("=== End Debug Info ===\n\n");
  }
};

#pragma pack(pop)