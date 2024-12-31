#pragma once
#include "base_msg.h"
namespace com {
  namespace msg {
    class motor_msg : public base_msg {
    public:
      motor_msg();
      motor_msg(float velocity, float position, uint8_t motor_id);
      ~motor_msg() = default;

      float get_velocity() const;
      float get_position() const;
      uint8_t get_motor_id() const;

      void set_velocity(float velocity);
      void set_position(float position);
      void set_motor_id(uint8_t motor_id);
    private:
      void update_payload();
    private:
      float velocity;
      float position;
      uint8_t motor_id;
    };
  }
}