// motor_control/include/harmonic_gait.h
#pragma once

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "types.h"
#include "result.h"
#include "unit.h"

namespace newton {

class HarmonicGait : public rclcpp::Node {
public:
  explicit HarmonicGait(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~HarmonicGait() = default;
  /**
    * @brief Initialize all the motors with the necessary runtime parameters
    * Odrive node takes care of communicating with the motor drivers via CAN.
    * The method loops through all the joints and sets the runtime parameters

    * @return result<void> indicating success or failure
   */
  result<void> init_clients();
  result<void> init_pubs();
  result<void> init_subs(); 

  result<void> init_offset(const std::array<float, 12> &positions);

  result<void> arm(int joint_index); // change the state of the motor driver to control mode
  result<void> disarm(int joint_index);

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
  result<void> clear_error(int joint_index);

  void set_joints_positions(const std::vector<float> positions);

private:
  void update_driver_status(const odrive_can::msg::ODriveStatus::SharedPtr msg,
                            int joint_index);
  void
  update_joint_state(const odrive_can::msg::ControllerStatus::SharedPtr msg,
                     int joint_index);

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr state_sub;
  void load_configs();
  void get_encoder_offsets();

  // safety checks
  bool check_limits();
  bool check_watchdog();
  bool check_errors();

  // result<void> clear_errors();
  void move();

private:
  static constexpr uint8_t NUM_JOINTS = 12;
  std::array<std::string, NUM_JOINTS> joint_names = {
      "fl_haa", "fl_hfe", "fl_kfe", "fr_haa", "fr_hfe", "fr_kfe",
      "hl_haa", "hl_hfe", "hl_kfe", "hr_haa", "hr_hfe", "hr_kfe"};

  std::array<newton::joint::config, NUM_JOINTS> joints_configs;
  std::array<newton::joint::state, NUM_JOINTS> joint_states;
  std::array<float, NUM_JOINTS> encoder_offsets; 
  bool offset_loaded = false;

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
rclcpp::Time last_time;
};
} // namespace newton
