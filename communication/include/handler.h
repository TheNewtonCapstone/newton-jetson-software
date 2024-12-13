#pragma once
#include "message.h"
#include "logger.h"
#include "transport.h"
#include <memory>


namespace com {
  class MessageHandler {
  public:
    explicit MessageHandler(std::unique_ptr<Transport> _transport)
      : transport(std::move(_transport)) {
    }

    template <typename T>
    status send(const T& msg) {
      const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&msg);
      std::vector<uint8_t> buffer(bytes, bytes + sizeof(T));
      // std::memcpy(data.data(), &msg, Message::MAX_SIZE);
      return transport->send(buffer);
    }

    template<typename T>
    status reiceive(T& msg) {
      std::vector<uint8_t> buffer;
      status status = transport->receive(buffer);
      if (status == status::OK && buffer.size() >= sizeof(T)) {
        std::memcpy(&msg, msg.data(), buffer.size());
        return status::OK;
      }
      Logger::LOG_ERROR(
      )
        return status::ERROR;
    }
  private:
    std::unique_ptr<Transport> transport;
  };
};