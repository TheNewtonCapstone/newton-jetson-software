#include <catch2/catch_test_macros.hpp>
#include "message.h"
#include "logger.h"

struct TestInit {
  TestInit() {
    Logger::get_instance().enable_all();
    Logger::LOG_INFO("MessageTest", "Starting message test suite");
  }
  ~TestInit() {
    Logger::LOG_INFO("MessageTest", "Message test suite completed");
  }
};

static TestInit testInit;

TEST_CASE("Message memory layout", "[message]") {
  Logger::LOG_INFO("MessageTest", "Testing message memory layout");

  SECTION("constant validation") {
    Logger::LOG_DEBUG("MessageTest", "Validating memory constants");
    Logger::LOG_INFO("MessageTest", "Header size: %d", Message::HEADER_SIZE);
    Logger::LOG_INFO("MessageTest", "Footer size: %d", Message::FOOTER_SIZE);
    Logger::LOG_INFO("MessageTest", "Max payload: %d", Message::MAX_PAYLOAD_LENGTH);
    Logger::LOG_INFO("MessageTest", "Buffer capacity: %d", Message::BUFER_CAPACITY);

    REQUIRE(Message::HEADER_SIZE == sizeof(MessageHeader));
    REQUIRE(Message::FOOTER_SIZE == sizeof(MessageFooter));
    REQUIRE(Message::MAX_PAYLOAD_LENGTH == Message::BUFER_CAPACITY - Message::HEADER_SIZE - Message::FOOTER_SIZE);

    Logger::LOG_INFO("MessageTest", "Memory constants validated successfully");
  }

  SECTION("offsets validation") {
    Logger::LOG_DEBUG("MessageTest", "Validating memory offsets");
    Logger::LOG_INFO("MessageTest", "Header offset: %d", Message::HEADER_OFFSET);
    Logger::LOG_INFO("MessageTest", "Payload offset: %d", Message::PAYLOAD_OFFSET);

    REQUIRE(Message::HEADER_OFFSET == 0);
    REQUIRE(Message::PAYLOAD_OFFSET == Message::HEADER_SIZE);

    Logger::LOG_INFO("MessageTest", "Memory offsets validated successfully");
  }
}

TEST_CASE("Message basic serialization", "[message]") {
  Logger::LOG_INFO("MessageTest", "Testing basic message serialization");

  Message msg(1234567, MessageType::MOTOR_COMMAND);
  MotorCommand cmd{
      .motor_id = 1,
      .motor_speed_x = 100,
      .motor_speed_y = 150,
      .motor_speed = 200
  };

  Logger::LOG_DEBUG("MessageTest", "Created test message with:");
  Logger::LOG_DEBUG("MessageTest", "  Motor ID: %d", cmd.motor_id);
  Logger::LOG_DEBUG("MessageTest", "  Speed X: %d", cmd.motor_speed_x);
  Logger::LOG_DEBUG("MessageTest", "  Speed Y: %d", cmd.motor_speed_y);
  Logger::LOG_DEBUG("MessageTest", "  Speed: %d", cmd.motor_speed);

  SECTION("serialization results") {
    Logger::LOG_INFO("MessageTest", "Testing message serialization");

    int length = msg.serialize(&cmd, sizeof(MotorCommand));
    Logger::LOG_INFO("MessageTest", "Serialized length: %d bytes", length);

    int expected_length = Message::HEADER_SIZE + sizeof(MotorCommand) + Message::FOOTER_SIZE;
    Logger::LOG_INFO("MessageTest", "Expected length: %d bytes", expected_length);

    REQUIRE(length == expected_length);

    MessageHeader hdr = msg.get_header();
    Logger::LOG_DEBUG("MessageTest", "Header validation:");
    Logger::LOG_DEBUG("MessageTest", "  Frame start: 0x%02X", hdr.frame_start);
    Logger::LOG_DEBUG("MessageTest", "  Timestamp: %u", hdr.timestamp_ns);
    Logger::LOG_DEBUG("MessageTest", "  Message type: 0x%02X", static_cast<uint8_t>(hdr.msg_type));
    Logger::LOG_DEBUG("MessageTest", "  Payload length: %u", hdr.payload_length);

    REQUIRE(hdr.frame_start == Message::FRAME_START_MARKER);
    REQUIRE(hdr.timestamp_ns == 1234567);
    REQUIRE(hdr.msg_type == MessageType::MOTOR_COMMAND);
    REQUIRE(hdr.payload_length == sizeof(MotorCommand));

    MessageFooter ftr = msg.get_footer();
    Logger::LOG_DEBUG("MessageTest", "Footer validation:");
    Logger::LOG_DEBUG("MessageTest", "  Frame end: 0x%02X", ftr.frame_end);
    Logger::LOG_DEBUG("MessageTest", "  Checksum: 0x%04X", ftr.checksum);

    REQUIRE(ftr.frame_end == Message::FRAME_END_MARKER);

    Logger::LOG_INFO("MessageTest", "Serialization test completed successfully");
  }
}

TEST_CASE("Message CRC verification", "[message]") {
  Logger::LOG_INFO("MessageTest", "Testing message CRC verification");

  Message msg(1234567, MessageType::MOTOR_COMMAND);
  MotorCommand cmd{
      .motor_id = 1,
      .motor_speed_x = 100,
      .motor_speed_y = 150,
      .motor_speed = 200
  };

  SECTION("CRC calculation") {
    Logger::LOG_DEBUG("MessageTest", "Starting CRC calculation test");

    msg.serialize(&cmd, sizeof(MotorCommand));
    Logger::LOG_DEBUG("MessageTest", "Message serialized, calculating CRC");

    uint16_t manual_crc = 0;
    const uint8_t* payload_data = msg.payload + Message::PAYLOAD_OFFSET;
    Logger::LOG_DEBUG("MessageTest", "Payload data at offset: %d", Message::PAYLOAD_OFFSET);

    for (size_t i = 0; i < sizeof(MotorCommand); i++) {
      manual_crc ^= payload_data[i];
      Logger::LOG_DEBUG("MessageTest", "CRC byte %zu: 0x%02X, running CRC: 0x%04X",
        i, payload_data[i], manual_crc);
    }

    MessageFooter ftr = msg.get_footer();
    Logger::LOG_INFO("MessageTest", "Manual CRC: 0x%04X", manual_crc);
    Logger::LOG_INFO("MessageTest", "Stored CRC: 0x%04X", ftr.checksum);
    Logger::LOG_INFO("MessageTest", "Calculated CRC: 0x%04X", msg.calculate_crc());

    REQUIRE(ftr.checksum == manual_crc);
    REQUIRE(ftr.checksum == msg.calculate_crc());

    Logger::LOG_INFO("MessageTest", "CRC verification completed successfully");
  }
}

TEST_CASE("Message round-trip serialization", "[message]") {
  Logger::LOG_INFO("MessageTest", "Testing message round-trip serialization");

  Message msg(1234567, MessageType::MOTOR_COMMAND);
  MotorCommand original{
      .motor_id = 1,
      .motor_speed_x = 100,
      .motor_speed_y = 150,
      .motor_speed = 200
  };

  Logger::LOG_DEBUG("MessageTest", "Original command values:");
  Logger::LOG_DEBUG("MessageTest", "  Motor ID: %d", original.motor_id);
  Logger::LOG_DEBUG("MessageTest", "  Speed X: %d", original.motor_speed_x);
  Logger::LOG_DEBUG("MessageTest", "  Speed Y: %d", original.motor_speed_y);
  Logger::LOG_DEBUG("MessageTest", "  Speed: %d", original.motor_speed);

  int serialized_len = msg.serialize(&original, sizeof(MotorCommand));
  Logger::LOG_INFO("MessageTest", "Serialized message length: %d bytes", serialized_len);

  MotorCommand deserialized;
  msg.deserialize(&deserialized, sizeof(MotorCommand));

  Logger::LOG_DEBUG("MessageTest", "Deserialized command values:");
  Logger::LOG_DEBUG("MessageTest", "  Motor ID: %d", deserialized.motor_id);
  Logger::LOG_DEBUG("MessageTest", "  Speed X: %d", deserialized.motor_speed_x);
  Logger::LOG_DEBUG("MessageTest", "  Speed Y: %d", deserialized.motor_speed_y);
  Logger::LOG_DEBUG("MessageTest", "  Speed: %d", deserialized.motor_speed);

  int compare_result = memcmp(&original, &deserialized, sizeof(MotorCommand));
  Logger::LOG_INFO("MessageTest", "Memory comparison result: %d", compare_result);

  REQUIRE(compare_result == 0);
  Logger::LOG_INFO("MessageTest", "Round-trip serialization test completed successfully");
}

TEST_CASE("Message corruption detection", "[message]") {
  Logger::LOG_INFO("MessageTest", "Testing message corruption detection");

  Message msg(1234567, MessageType::MOTOR_COMMAND);
  MotorCommand cmd{
      .motor_id = 1,
      .motor_speed_x = 100,
      .motor_speed_y = 150,
      .motor_speed = 200
  };

  msg.serialize(&cmd, sizeof(MotorCommand));
  uint16_t original_crc = msg.calculate_crc();
  Logger::LOG_INFO("MessageTest", "Original CRC: 0x%04X", original_crc);

  SECTION("payload corruption") {
    Logger::LOG_DEBUG("MessageTest", "Corrupting payload at offset %d", Message::PAYLOAD_OFFSET);
    Logger::LOG_DEBUG("MessageTest", "Original byte: 0x%02X", msg.payload[Message::PAYLOAD_OFFSET]);

    msg.payload[Message::PAYLOAD_OFFSET] ^= 0xFF;
    Logger::LOG_DEBUG("MessageTest", "Corrupted byte: 0x%02X", msg.payload[Message::PAYLOAD_OFFSET]);

    uint16_t corrupted_crc = msg.calculate_crc();
    Logger::LOG_INFO("MessageTest", "Corrupted CRC: 0x%04X", corrupted_crc);

    REQUIRE(corrupted_crc != original_crc);
    Logger::LOG_INFO("MessageTest", "Corruption detection test completed successfully");
  }
}