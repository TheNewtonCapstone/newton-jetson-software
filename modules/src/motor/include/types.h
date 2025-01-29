# pragma once
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace newton { 
  namespace leg {
    enum id {
      FL = 0,
      FR = 1,
      BL = 2,
      BR = 3,
    };
  struct state {
    id leg_id;
    std::array<joint::state> joints;
    uint32_t error_flags;
  };

    constexpr std::array<id, 4> all = {FL, FR, BL, BR};
  }  // namespace leg_id

  namespace joint{
    enum id {
      HAA = 0,
      HFE = 1,
      KFE = 2,
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
      float position;
      float velocity;
      float torque;
      float torque_target;
      uint32_t watchdog_status;
    };

    struct config{
      std::string name;
      uint8_t can_node_id;
      float offset;

      float gear_ratio;
      float torque_constant;
      float watchdog_timeout;
      limits limits;
    };

    constexpr std::array<id, 3> all = {HAA, HFE, KFE};  
    using joint_configs = std::unordered_map<std::string, joint::config>;
  } // namespace joint
  



  namespace system
  {
   struct state{
     std::array<leg::state, 4> legs;
     uint32_t error_flags;
     uint32_t error_status;
   }; 
  } // namespace system
  
  struct motor_config


  
}  // namespace newton