#include <vector>
#include "timer.h"
#include "motor_msg.h"

namespace com {
  namespace msg {

    motor_msg::motor_msg() : Message(timer::apptime_us().get_value()) {
      set_message_type(MessageType::MOTOR_COMMAND);
    }

    motor_msg::motor_msg(float velocity, float steering, uint8_t flags)
      : Message(timer::apptime_us().get_value()),
      velocity(velocity),
      flags(flags),
      steering(steering) {
      set_message_type(MessageType::MOTOR_COMMAND);
      update_payload();
    }

    void motor_msg::update_payload() {
        // Create a buffer for our data
      std::vector<uint8_t> temp_payload(sizeof(float) * 2 + sizeof(uint8_t));

      // Copy the float values into the buffer
      memcpy(temp_payload.data(), &velocity, sizeof(float));
      memcpy(temp_payload.data() + sizeof(float), &steering, sizeof(float));
      memcpy(temp_payload.data() + 2 * sizeof(float), &flags, sizeof(uint8_t));

      // Update the message payload
      set_payload(temp_payload.data(), temp_payload.size());
    }

  }
}// namespace msg