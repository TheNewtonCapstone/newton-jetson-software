#pragma once
#include <string> 
#include <cstdint>
#include <vector>

namespace com {
  enum class status {
    OK,
    ERROR,
    TIMEOUT,
    NOT_CONNECTED,
    BUFFER_OVERFLOW
  };


  class Transport {
  public:

    virtual ~Transport() = default;
    virtual status connect() = 0;
    virtual status disconnect() = 0;
    virtual status send(const std::vector<uint8_t>& data) = 0;
    virtual status receive(std::vector<uint8_t>& data) = 0;
  };

}