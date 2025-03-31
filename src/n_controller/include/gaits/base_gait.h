#pragma once

#include "data/cmd.h"
#include "data/imu.h"
#include "data/joint.h"
#include "geometry_msgs/msg/twist.hpp"
#include "globals.h"
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

  rclcpp::Time last_time;

  virtual std::array<float, NUM_JOINTS> update(
      const std::array<float, NUM_OBSERVATIONS> &observations) = 0;
};
}  // namespace newton
