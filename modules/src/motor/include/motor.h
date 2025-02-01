// motor_control/include/motor_control/motor.h
#pragma once

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "result.h"
#include "types.h"

namespace newton {

class MotorDriver : public rclcpp::Node {
 public:
  explicit MotorDriver(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MotorDriver() = default;
  result<void> init();  // initialize the motor driver do checks
  result<void> start();
  result<void> stop();
  result<void> set_joint_mode(const joint::id joint_id, const joint::mode mode);
  result<void> set_joint_position(const joint::id joint_id,
                                  const float position);
  result<void> set_joint_velocity(const joint::id joint_id,
                                  const float velocity);
  result<void> set_joint_torque(const joint::id joint_id, const float torque);
  result<void> set_joints_positions(
      const std::array<float, NUM_JOINTS> positions);

 private:
  void statusCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg);
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr state_sub;
  result<void> load_configs();

  // safety checks
  bool check_limits();
  bool check_watchdog();
  bool check_errors();

  clear_errors();

 private:
  static constexpr uint8_t NUM_LEGS = 4;
  static constexpr uint8_t NUM_JOINTS_PER_LEG = 3;
  static constexpr uint8_t NUM_JOINTS =
      NUM_LEGS * NUM_JOINTS_PER_LEG;  // 12 joints used interchangeably with
                                      // motors with motors

  std::array<joint_configs, NUM_JOINTS> joints_configs;
  std::array<joint::state, NUM_JOINTS> joint_states;
  std::array<leg::state, NUM_LEGS> leg_states;

  std::map<joint::id, int> id_to_index;

  std::array<rclcpp::Publisher<odrive_can::msg::ControllerStatus>::SharedPtr,
             NUM_JOINTS>
      joint_pubs;
  std::array<rclcpp::Subscription<odrive_can::msg::ControlMessage>::SharedPtr,
             NUM_JOINTS>
      joint_subs;
  std::array<rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr, NUM_JOINTS>
      joint_srvs;
};

}  // namespace newton
