#include "motor_msg.h"

namespace com {
  namespace msg {
    motor_msg::motor_msg() {
      header.message_type = static_cast<uint8_t>(message_type::MOTOR_COMMAND);
    }

    motor_msg::motor_msg(float _velocity, float _position, uint8_t _motor_id)
      : velocity(_velocity), position(_position), motor_id(_motor_id) {
      header.message_type = static_cast<uint8_t>(message_type::MOTOR_COMMAND);
      update_payload();
    }

    float motor_msg::get_velocity() const {
      return velocity;
    }

    float motor_msg::get_position() const {
      return position;
    }

    uint8_t motor_msg::get_motor_id() const {
      return motor_id;
    }

    void motor_msg::set_velocity(float _velocity) {
      velocity = _velocity;
      update_payload();
    }

    void motor_msg::set_position(float _position) {
      position = _position;
      update_payload();
    }

    void motor_msg::set_motor_id(uint8_t _motor_id) {
      motor_id = _motor_id;
      update_payload();
    }

    void motor_msg::update_payload() {
      std::vector<uint8_t> payload(sizeof(float) * 2 + 1);
      // memory to memory copy
      memcpy(payload.data(), &velocity, sizeof(float));
      memcpy(payload.data() + sizeof(float), &position, sizeof(float));
      payload[sizeof(float) * 2] = motor_id;
      set_payload(payload);
    };
  }
}