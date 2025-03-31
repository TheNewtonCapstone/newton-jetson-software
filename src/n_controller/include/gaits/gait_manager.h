#pragma once

#include <map>
#include <memory>
#include <string>

#include "data/cmd.h"
#include "data/gait_type.h"
#include "data/imu.h"
#include "data/joint.h"
#include "gaits/base_gait.h"
#include "gaits/harmonic_gait.h"
#include "gaits/machine_gait.h"
#include "gaits/standing_gait.h"
#include "geometry_msgs/msg/twist.hpp"
#include "handlers/onnx_handler.h"
#include "rclcpp/rclcpp.hpp"
#include "result.h"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace newton {
class GaitManager : public rclcpp::Node {
 public:
  GaitManager();
  ~GaitManager() = default;

  // Initialize ROS communication
  result<void> init_publishers();
  result<void> init_subscribers();

  // gait management
  result<void> set_gait(GaitType type);
  result<std::shared_ptr<BaseGait>> get_gait(GaitType type);

  // transition management
  void update();
  void start_transition(GaitType target_gait);
  void set_joint_positions(const std::array<float, NUM_JOINTS>& positions);

  // select gait based on velocity
  result<GaitType> select_gait_by_velocity(float linear_magnitude);

  // callbacks
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void gait_change_cb(const std_msgs::msg::String::SharedPtr msg);
  void joint_states_pos_cb(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);  // cb is callback
  void joint_states_vel_cb(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void joint_states_tor_cb(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void imu_state_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odrive_ready_cb(
      const std_msgs::msg::Bool::SharedPtr msg);  // odrive ready callback

  // shutdown
  result<void> shutdown();

 public:
 protected:
  std::map<GaitType, std::shared_ptr<BaseGait>> gaits;

  GaitType current_gait_type;
  GaitType target_gait_type;

  float transition_progress;  // 0.0  is current gait, 1.0 is target gait
  float transition_duration;
  /// @brief Flag that dictates if the gait manager should automatically change
  /// gaits based on speed
  bool enable_automatic;

  // robot state

  std::array<float, NUM_OBSERVATIONS> observations;
  std::unique_ptr<newton::ImuReading> imu;
  std::unique_ptr<newton::VelocityCmd> cmd;

  // previous actions
  std::array<float, NUM_JOINTS> previous_actions;

  // output actions
  std::array<float, NUM_JOINTS> current_positions;
  std::array<float, NUM_JOINTS> current_velocities;
  std::array<float, NUM_JOINTS> current_torques;
  std::array<float, NUM_JOINTS> target_positions;
  std::array<float, NUM_JOINTS> joint_cmd_positions;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_gait_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_change_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      joint_states_pos_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      joint_states_vel_sub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      joint_states_torque_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odrive_ready_sub;
  rclcpp::TimerBase::SharedPtr update_timer;

  // control period
  const std::chrono::milliseconds CONTROLLER_PERIOD = 20ms;
  rclcpp::Time last_time;
  bool odrive_ready = false;
};
}  // namespace newton