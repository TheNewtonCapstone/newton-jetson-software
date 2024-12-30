#pragma once
#include "message.h"
#include "logger.h"
#include <memory>


namespace com {
  template <typename T>
  class handler {
  public:
    explicit handler(std::shared_ptr<serial> _uart) : uart(_uart) {}
    ~handler();
    result <void> serialize_message(const message<T>& msg) {
      std::array<uint8_t, MAX_MSG_SIZE> data;
      data[0] = msg.get_start_byte();
      data[1] = static_cast<uint8_t>(msg.get_message_type());
      std::array<uint8_t, sizeof(T)> bytes = msg.get_bytes();
      std::memcpy(data.data() + 2, bytes.data(), sizeof(T));
      data[sizeof(T) + 2] = msg.get_crc();
      data[sizeof(T) + 3] = msg.get_end_byte();
      return result<void>::success();
    }
    result <void> deserialize_message(const std::array<uint8_t, MAX_MSG_SIZE>& data) {
      message<T> msg;
      msg.set_start_byte(data[0]);
      msg.set_message_type(static_cast<message_type>(data[1]));
      std::array<uint8_t, sizeof(T)> bytes;
      std::memcpy(bytes.data(), data.data() + 2, sizeof(T));
      msg.set_bytes(bytes);
      msg.set_crc(data[sizeof(T) + 2]);
      msg.set_end_byte(data[sizeof(T) + 3]);
      return result<void>::success();
    }

    result<void> send_message(const message<T>& msg) {
      auto result = serialize_message(msg);
      if (result.has_error()) {
        return result<void>::error("Failed to serialize message with error %s", result.get_error_msg().c_str());
      }
      auto result = uart->send(result.get_value());
      return _uart->send(result.get_value());
    }
    result<message<T>> receive_message() {
      auto result = uart->receive();
      if (result.has_error()) {
        return result<message<T>>::error("Failed to receive message with error %s", result.get_error_msg().c_str());
      }
      auto result = deserialize_message(result.get_value());
      if (result.has_error()) {
        return result<message<T>>::error("Failed to deserialize message with error %s", result.get_error_msg().c_str());
      }
      return result<message<T>>::success(result.get_value());
    }

  private:
    std::shared_ptr<serial> uart;
  };
};