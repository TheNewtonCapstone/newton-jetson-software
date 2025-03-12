# pragma once
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include "unit.h"

namespace newton { 
  struct joint{
    int id; // node id, used by can
    std::string name;
    float min_pos;
    float max_pos;
    
    float cur_pos; //in radians
    float curr_vel; // in rad/s
    float curr_torque; // in Nm
    float curr_torque_target; // in Nm

    float offset;
    float direction;  // 1 or -1 to have uniformity in the direction of every joint
    float watchdog_timeout;
  };

  // normalize the angle to be between -PI and PI
  inline float normalize_angle(float angle){
    angle = std::fmod(angle, TWO_PI);
    if (angle > PI) angle -= TWO_PI; 
    if (angle < -PI) angle += TWO_PI;

    return angle;
  }

  // we dont want the motor to turn taking the longest path to the target
  inline float shortest_path(float current, float target){
    current = normalize_angle(current);
    target = normalize_angle(target);

    float diff = target - current;

    if(diff > PI){
      diff -= TWO_PI;
    } else if(diff < -PI){
      diff += TWO_PI;
    }
    return current + diff;
  }

  inline float rad_to_turns(float angle, const joint& j){
    // adjust the position to the direction of the joint
    float adjusted_pos = angle * j.direction;
     // convert to motor turns
    return adjusted_pos * RAD_TO_MOTOR_TURNS + j.offset;
  }

  inline float turns_to_rad(float motor_turns, const joint& j){
    // convert to rad
    return (motor_turns - j.offset) * MOTOR_TURNS_TO_RAD;
  }

  inline bool is_within_limits(float pos, const joint& j){
    return pos >= j.min_pos && pos <= j.max_pos;
  }

  inline float clamp_to_limits(float pos, const joint& j){
    return std::min(std::max(pos, j.min_pos), j.max_pos);
  }


};  // namespace newton

