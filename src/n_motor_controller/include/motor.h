// motor_control/include/motor_control/motor.h
#pragma once

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "types.h"
#include "result.h"

namespace newton {

class MotorDriver : public rclcpp::Node {
public:
  explicit MotorDriver(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MotorDriver() = default;
  /**
    * @brief Initialize all the motors with the necessary runtime parameters
    * Odrive node takes care of communicating with the motor drivers via CAN.
    * The method loops through all the joints and sets the runtime parameters

    * @return result<void> indicating success or failure
   */
  result<void> init();  // initialize all the motors with the necessary runtime parameters
  void start();
  void stop();
  void shutdown();
  void set_joint_mode(const joint::mode mode, int joint_index);
  void set_joint_position(float, int);

/**
  * @brief Send a request to the motor driver node to set the joint mode. 
  *  See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-msg-set-axis-state for more details
  * @param mode: the mode to set the joint to
  * @param joint_index: the index of the joint to set the mode for
 */
  result<void> request_axis_state(size_t joint_index, uint32_t requested_state);
  void set_joints_positions(const std::vector<float> positions);

private:
  void update_driver_status(const odrive_can::msg::ODriveStatus::SharedPtr msg,
                            int joint_index);
  void
  update_joint_state(const odrive_can::msg::ControllerStatus::SharedPtr msg,
                     int joint_index);

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr state_sub;
  void load_configs();

  // safety checks
  bool check_limits();
  bool check_watchdog();
  bool check_errors();

  void clear_errors();

private:
  static constexpr uint8_t NUM_LEGS = 4;
  static constexpr uint8_t NUM_JOINTS_PER_LEG = 3;
  static constexpr uint8_t NUM_JOINTS =
      NUM_LEGS * NUM_JOINTS_PER_LEG; // 12 joints used interchangeably with
                                     // motors with motors
  std::array<std::string, NUM_JOINTS> joint_names = {
      "fl_haa", "fl_hfe", "fl_kfe", "fr_haa", "fr_hfe", "fr_kfe",
      "hl_haa", "hl_hfe", "hl_kfe", "hr_haa", "hr_hfe", "hr_kfe"};

  std::array<newton::joint::config, NUM_JOINTS> joints_configs;
  std::array<newton::joint::state, NUM_JOINTS> joint_states;
  std::array<newton::leg::state, NUM_LEGS> leg_states;

  rclcpp::TimerBase::SharedPtr timer_;

  std::array<rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr, NUM_JOINTS>
      clients;

  std::array<rclcpp::Subscription<odrive_can::msg::ODriveStatus>::SharedPtr,
             NUM_JOINTS>
      status_subs;
  std::array<rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr,
             NUM_JOINTS>
      joint_state_subs;
  std::array<rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr,
             NUM_JOINTS>
      control_pubs;
};
} // namespace newton
