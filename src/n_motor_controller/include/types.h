# pragma once
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace newton { namespace joint{
    enum id {
      HAA = 0,
      HFE = 1,
      KFE = 2,
    };
    enum class axis_state{
      UNDEFINED = 0,
      IDLE=1,
      STARTUP_SEQUENCE = 2,
      CLOSED_LOOP_CONTROL = 8
    };
    enum class mode {
      position = 0,
      velocity = 1,
      torque = 2,
    };

    struct limits {
      float pos_min;
      float pos_max;
      float vel_min;
      float vel_max;
      float torque_min;
      float torque_max;
    };
    
    struct state{
      int index;
      float position;
      float velocity;
      float torque;
      float torque_target;
    };

    struct config{
      std::string name;
      uint8_t can_node_id;
      float offset;

      float gear_ratio;
      float torque_constant;
      float watchdog_timeout;
      joint::limits limits;
    };
  } // namespace joint
  

  namespace leg {
    enum id {
      FL = 0,
      FR = 1,
      BL = 2,
      BR = 3,
    };
    struct state {
    id leg_id;
    std::array<joint::state, 3> joints;
    uint32_t error_flags;
    };
    constexpr std::array<id, 4> all = {FL, FR, BL, BR};
  }  // namespace leg_id

  namespace system
  {
   struct state{
     std::array<leg::state, 4> legs;
     uint32_t error_flags;
     uint32_t error_status;
   }; 
  } // namespace system
}  // namespace newton