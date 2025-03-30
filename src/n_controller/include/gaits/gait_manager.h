#pragma ONCE
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
#include "gaits/sliding_gait.h"
#include "gaits/walking_gait.h"
#include "geometry_msgs/msg/twist.hpp"
#include "handlers/onnx_handler.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace newton {
using namespace std::chrono_literals;

class GaitManager : public rclcpp::Node {
 public:
  GaitManager();
  ~GaitManager() = default;  //

  // Initialize ROS communication
  result<void> init_publishers();
  result<void> init_subscribers();

  // callbacks
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr
                      msg);  // the velocity of the robot
  void gait_change_cb(const std_msgs::msg::String::SharedPtr msg);
  void joint_states_pos_cb(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);  // cb is callback
  void joint_states_vel_cb(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void imu_state_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void update_odrive_ready(
      const std_msgs::msg::Bool::SharedPtr msg);  // odrive ready callback

  // gait management
  result<std::shared_ptr<BaseGait>> get_gait(GaitType type);
  result<void> set_gait(GaitType type);

  // transition management
  result<void> start_transition(GaitType target_gait);
  void update();
  void set_joint_positions(
      const std::array<float, BaseGait::NUM_JOINTS>& positions);

  // publish joint positions
  void publish_joint_positions(
      const std::array<float, BaseGait::NUM_JOINTS>& positions);

  // select gait based on velocity
  result<GaitType> select_gait_by_velocity(float linear_magnitude);

 private:
  static constexpr uint8_t NUM_JOINTS = 8;
  static constexpr int NUM_OBSERVATIONS = 33;
  static constexpr int NUM_ACTIONS = 8;
  static constexpr int ANG_VEL_IDX = 0;
  static constexpr int PROJ_GRAV_IDX = 3;
  static constexpr int CMD_VEL_IDX = 6;
  static constexpr int POSITION_IDX = 9;
  static constexpr int VELOCITY_IDX = POSITION_IDX + NUM_JOINTS;
  static constexpr int PREV_ACTION_IDX = VELOCITY_IDX + NUM_JOINTS;

  std::map<GaitType, std::shared_ptr<BaseGait>> gaits;
  GaitType current_gait_type;
  GaitType target_gait_type;
  float transition_progress;  // 0.0  is current gait, 1.0 is target gait
  float transition_duration;
  bool enable_automatic;  // boolean flag to get automatic transitions based on
                          // speed. default is true

  // robot state

  std::array<float, NUM_OBSERVATIONS> observations;  // observations // 33
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

  // static constexpr std::chrono::milliseconds CONTROLLER_PERIOD = 20ms; //
  // 20ms update period

  // control period
  static constexpr std::chrono::milliseconds CONTROLLER_PERIOD =
      20ms;  // 20ms update period
  rclcpp::Time last_time;
  bool odrive_ready = false;
}

}  // namespace newton