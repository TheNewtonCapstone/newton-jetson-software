#pragma once

#include "data/cmd.h"
#include "data/imu.h"
#include "data/joint.h"
#include "gaits/gait_manager.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "result.h"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "unit.h"

namespace newton {
class BaseGait : public rclcpp::Node {
 public:
  explicit BaseGait(const std::string name, const bool should_set_to_standing,
                    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~BaseGait() = default;

 protected:
  static constexpr int NUM_JOINTS = GaitManager::NUM_JOINTS;
  static constexpr int POSITION_IDX = GaitManager::POSITION_IDX;
  static constexpr int VELOCITY_IDX = GaitManager::VELOCITY_IDX;
  static constexpr int PREV_ACTION_IDX = GaitManager::PREV_ACTION_IDX;
  static constexpr int NUM_OBSERVATIONS = GaitManager::NUM_OBSERVATIONS;

  std::unordered_map<std::string, float> phases = {
      {"fl", 0.0},  // front left
      {"fr", PI},   // front right
      {"hl", PI},   // hind left
      {"hr", 0.0},  // hind right
  };

  const std::array<std::string, NUM_JOINTS> joint_names = {
      "fl_hfe", "fl_kfe",  // front left
      "fr_hfe", "fr_kfe",  // front right
      "hl_hfe", "hl_kfe",  // hind left
      "hr_hfe", "hr_kfe",  // hind right
  };

  std::unordered_map<std::string, std::array<float, 2>> leg_standing_positions =
      {
          {"fl", {0.8, -1.4}},  // front left
          {"fr", {0.8, -1.4}},  // front right
          {"hl", {0.8, -1.4}},  // hind left
          {"hr", {0.8, -1.4}},  // hind right
      };

  std::array<float, NUM_JOINTS> standing_positions = {
      0.8, -1.4,  // front left
      0.8, -1.4,  // front right
      0.8, -1.4,  // hind left
      0.8, -1.4,  // hind right

  };

  // map the legs to ids
  const std::unordered_map<std::string, std::array<int, 2>> leg_ids = {
      {"fl", {0, 1}},  // front left
      {"fr", {2, 3}},  // front right
      {"hl", {4, 5}},  // hind left
      {"hr", {6, 7}},  // hind right
  };

  rclcpp::Time last_time;

  virtual std::array<float, GaitManager::NUM_JOINTS> update(
      const std::array<float, GaitManager::NUM_OBSERVATIONS> &observations) = 0;
};
}  // namespace newton
