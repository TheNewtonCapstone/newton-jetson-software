#include "gaits/base_gait.h"

#include <array>
#include <rclcpp/rclcpp.hpp>

#include "data/vectors.h"
#include "logger.h"

using namespace newton;
using namespace std::chrono_literals;

BaseGait::BaseGait(const std::string node_name,
                   const bool should_set_to_standing,
                   const rclcpp::NodeOptions &options)
    : Node(node_name), should_set_to_standing(should_set_to_standing) {};

result<void> BaseGait::init() { last_time = this->get_clock()->now(); }

result<void> BaseGait::init_pubs() {
  joints_position_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
      "joint_cmd_positions", 10);

  RCLCPP_INFO(this->get_logger(),
              "Created a publisher to /joint_cmd_positions");

  return result<void>::success();
}

result<void> BaseGait::init_subs() {
  joisnt_states_po_sub =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "joint_state_positions", 10,
          [=, this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            this->update_joints_position(msg);
          });

  joint_states_vel_sub =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "joint_state_velocities", 10,
          [=, this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            this->update_joints_velocity(msg);
          });
  joint_states_torque_sub =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "joint_state_torques", 10,
          [=, this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            this->update_joints_torque(msg);
          });

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", 10, [=, this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        this->update_imu(msg);
      });

  cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "vel_cmds", 10,
      [=, this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->update_cmd(msg);
      });

  odrive_ready_sub = this->create_subscription<std_msgs::msg::Bool>(
      "odrive_ready", 10, [=, this](const std_msgs::msg::Bool::SharedPtr msg) {
        this->update_odrive_ready(msg);
      });

  return result<void>::success();
}

result<void> BaseGait::shutdown() {
  rclcpp::shutdown();

  return result<void>::success();
}

result<void> BaseGait::update_joints_torque(
    std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].curr_torque = msg->data[i];
  }

  return result<void>::success();
}
result<void> BaseGait::update_joints_position(
    std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].curr_pos = msg->data[i];
  }

  return result<void>::success();
}

result<void> BaseGait::update_joints_velocity(
    std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].curr_vel = msg->data[i];
  }

  return result<void>::success();
}

result<void> BaseGait::update_imu(sensor_msgs::msg::Imu::SharedPtr msg) {
  const auto dt =
      static_cast<double>(msg->header.stamp.nanosec - imu->timestamp) / 1e9;

  imu->timestamp = msg->header.stamp.nanosec;
  imu->linear_acceleration =
      vector3(msg->linear_acceleration.x, msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  imu->linear_velocity += imu->linear_acceleration * dt;
  imu->angular_velocity =
      vector3(msg->angular_velocity.x, msg->angular_velocity.y,
              msg->angular_velocity.z);
  imu->rotation = quat_to_rpy(msg->orientation.w, msg->orientation.x,
                              msg->orientation.y, msg->orientation.z);
  imu->projected_gravity =
      quat_to_proj_gravity(msg->orientation.w, msg->orientation.x,
                           msg->orientation.y, msg->orientation.z);

  return result<void>::success();
}

result<void> BaseGait::update_cmd(geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd->linear_velocity = Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
  cmd->angular_velocity =
      Vector3(msg->angular.x, msg->angular.y, msg->angular.z);

  return result<void>::success();
}

result<void> BaseGait::update_odrive_ready(std_msgs::msg::Bool::SharedPtr msg) {
  odrive_ready = msg->data;

  // for some reason, we get this message with a false, just leave
  if (!odrive_ready) {
    Logger::WARN("BaseGait", "Received non-ready odrive message");
    return result<void>::success();
  }

  // if we are ready but we don't have an initial position, leave
  if (!should_set_to_standing) return result<void>::success();

  set_joints_position(standing_positions);
  std::this_thread::sleep_for(3s);

  move_timer = this->create_wall_timer(20ms, std::bind(&BaseGait::move, this));

  return result<void>::success();
}
