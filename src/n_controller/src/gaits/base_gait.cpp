#include "gaits/base_gait.h"
#include "data/vectors.h"
#include "logger.h"

#include <array>
#include <rclcpp/rclcpp.hpp>

using namespace newton;
using namespace std::chrono_literals;

BaseGait::BaseGait(const std::string node_name,
                   const rclcpp::NodeOptions &options)
    : Node(node_name) {};

result<void> BaseGait::init()
{
  // declare the parameters for the joints

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    auto name = joints[i].name = joint_names[i];

    double default_pos_limit = 3.14f;
    double default_direction = 1.f;

    this->declare_parameter(name + "_position_limit", default_pos_limit);
    this->declare_parameter(name + "_direction", default_direction);
  }

  init_pubs();
  init_clients();
  init_subs();

  imu = std::make_unique<ImuReading>();
  cmd = std::make_unique<VelocityCmd>();

  move_timer = this->create_wall_timer(20ms, std::bind(&BaseGait::move, this));
  last_time = this->get_clock()->now();
}

result<void> BaseGait::init_clients() { return result<void>::success(); }

result<void> BaseGait::init_pubs()
{
  joints_position_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
      "joints_cmd_positions", 10);

  RCLCPP_INFO(this->get_logger(),
              "Created a publisher to /joint_cmd_positions");

  return result<void>::success();
}

result<void> BaseGait::init_subs()
{
  joints_position_sub =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "joints_state_positions", 10,
          [=, this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
          {
            this->update_joints_position(msg);
          });

  joints_velocity_sub =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "joints_state_velocities", 10,
          [=, this](const std_msgs::msg::Float32MultiArray::SharedPtr msg)
          {
            this->update_joints_velocity(msg);
          });

  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", 10, [=, this](const sensor_msgs::msg::Imu::SharedPtr msg)
      { this->update_imu(msg); });

  cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "vel_cmds/keyboard", 10, [=, this](const geometry_msgs::msg::Twist::SharedPtr msg)
      { this->update_cmd(msg); });

  return result<void>::success();
}

result<void> BaseGait::shutdown()
{
  rclcpp::shutdown();

  return result<void>::success();
}

result<void>
BaseGait::set_joints_position(const std::array<float, NUM_JOINTS> &positions)
{
  auto msg = std_msgs::msg::Float32MultiArray();

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    float set_position = clamp_to_limits(positions[i], joints[i]);
    set_position = rad_to_turns(set_position, joints[i]);

    msg.data.push_back(set_position);

    RCLCPP_INFO(this->get_logger(), "Set joint position to %f for %d",
                set_position, i);
  }

  joints_position_pub->publish(msg);
}

result<void> BaseGait::update_joints_position(
    std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    joints[i].curr_pos = msg->data[i];
  }

  return result<void>::success();
}

result<void> BaseGait::update_joints_velocity(
    std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    joints[i].curr_vel = msg->data[i];
  }

  return result<void>::success();
}

result<void> BaseGait::update_imu(sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto dt =
      static_cast<double>(msg->header.stamp.nanosec - imu->timestamp) / 1e9;

  imu->timestamp = msg->header.stamp.nanosec;
  imu->linear_acceleration =
      Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y,
              msg->linear_acceleration.z);
  imu->linear_velocity += imu->linear_acceleration * dt;
  imu->angular_velocity =
      Vector3(msg->angular_velocity.x, msg->angular_velocity.y,
              msg->angular_velocity.z);
  imu->rotation = quat_to_rpy(msg->orientation.w, msg->orientation.x,
                              msg->orientation.y, msg->orientation.z);
  imu->projected_gravity =
      quat_to_proj_gravity(msg->orientation.w, msg->orientation.x,
                           msg->orientation.y, msg->orientation.z);

  return result<void>::success();
}

result<void>
BaseGait::update_cmd(geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd->linear_velocity = Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
  cmd->angular_velocity =
      Vector3(msg->angular.x, msg->angular.y, msg->angular.z);

  return result<void>::success();
}
