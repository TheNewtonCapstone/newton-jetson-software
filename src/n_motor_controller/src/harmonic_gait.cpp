#include "harmonic_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;
using namespace std::chrono_literals;

HarmonicGait::HarmonicGait(const rclcpp::NodeOptions &options)
    : Node("motor_driver"){
  // declare the parameters for the joints

  
  const std::string num_joints_param  = "num_joints";
  const int NUM_JOINTS = 12;
  this->declare_parameter(num_joints_param, NUM_JOINTS); 

  for(int i=0; i < NUM_JOINTS; i++){
    auto name = joints[i].name = joint_names[i];
    double default_pos_limit = 3.14f; 
    double default_direction = 1.f;
    this->declare_parameter(name + "_position_limit", default_pos_limit);
    this->declare_parameter(name + "_direction", default_direction);
  }

  
  load_joint_configs();
  init_pubs();
  init_clients();
  init_subs();

  timer_ = this->create_wall_timer(20ms, std::bind(&HarmonicGait::move, this));
  last_time = this->get_clock()->now();
};

result<void> HarmonicGait::init_clients()
{

  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    auto name = joint_names[i];

    axis_state_clients[i] = this->create_client<odrive_can::srv::AxisState>(
        name + "/request_axis_state");
  }

  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    auto name = joint_names[i];

    clear_error_clients[i] = this->create_client<std_srvs::srv::Empty>(
        name + "/clear_errors");
  }

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    if (disarm(i).has_error())
    {
      Logger::ERROR("Harmonic Gait", "Error: request to disarm failed for %s", joint_names[i].c_str());
      shutdown();
    }
  }

  std::this_thread::sleep_for(std::chrono::seconds(5));

  RCLCPP_INFO(this->get_logger(), "Initializing HarmonicGait...");

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    odrive_can::msg::ControlMessage msg;
    msg.control_mode = 3;
    msg.input_mode = 5;
    msg.input_pos = 0.0;
    msg.input_vel = 0.0;
    msg.input_torque = 0.0;
    control_pubs[i]->publish(msg);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    if (arm(i).has_error())
    {
      Logger::ERROR("Harmonic Gait", "Error: request to arm failed for %s", joint_names[i].c_str());
      // if (request_axis_state(i, 3).has_error())
      // {
      //   Logger::ERROR("Harmonic Gait", "Error request to arm failed for %s", joint_names[i].c_str());
      //   shutdown();
      // }

      // if (request_axis_state(i, 8).has_error())
      // {
      //   Logger::ERROR("Harmonic Gait", "Error request to arm failed for %s", joint_names[i].c_str());
      //   shutdown();
      // }
    }
  }
  return result<void>::success();
}
result<void> HarmonicGait::init_pubs(){
  for (int i = 0; i < NUM_JOINTS; ++i) {
  auto name = joint_names[i];

    control_pubs[i] =
        this->create_publisher<odrive_can::msg::ControlMessage>(name + "/control_message", 10);

    RCLCPP_INFO(this->get_logger(), "Created a publisher to to %s", name.c_str());
  }

  return result<void>::success();
}

result<void> HarmonicGait::init_subs()
{

  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    auto name = joint_names[i];

    status_subs[i] =
        this->create_subscription<odrive_can::msg::ODriveStatus>(
            name + "/odrive_status", 10,
            [=, this](const odrive_can::msg::ODriveStatus::SharedPtr msg)
            {
              this->update_driver_status(msg, i);
            });

    joint_state_subs[i] =
        this->create_subscription<odrive_can::msg::ControllerStatus>(
            name + "/controller_status", 10,
            [=, this](const odrive_can::msg::ControllerStatus::SharedPtr msg)
            {
              this->update_joint_state(msg, i);
            });
  }

  return result<void>::success();
}

void HarmonicGait::update_driver_status(
    const odrive_can::msg::ODriveStatus::SharedPtr msg, int joint_index)
{
}

result<void> HarmonicGait::update_joint_state(
    const odrive_can::msg::ControllerStatus::SharedPtr msg, int joint_index) {
        // todo: add checks for the limits and shutdown if the limits are exceeded
        // convert from turns to rad
      joints[joint_index].cur_pos = rad_to_turns(msg->pos_estimate, joints[joint_index]);
      joints[joint_index].curr_vel = rad_to_turns(msg->vel_estimate, joints[joint_index]);

  if (!offset_loaded)
  {
    Logger::INFO("HarmonicGait", "Updating joint state for %d", joint_index);
    // encoder_offsets[joint_index] = msg->pos_estimate;
    joints[joint_index].offset = msg->pos_estimate;
  }

  // first time we get to the end of the loop, we have all the offsets
  if (joint_index == NUM_JOINTS - 1)
  {
    for (size_t i = 0; i < NUM_JOINTS; i++){
      Logger::INFO("HarmonicGait", "Joint %d offset: %f", i, joints[i].offset);
    }
    offset_loaded = true;
  }
  return result<void>::success();
}

result<void> HarmonicGait::set_joint_position(float position, int index) {

  // takes in the position in rad/ and apply
  // the gear ratio to get the motor turns 
      if(!is_within_limits(position, joints[index])){
        return result<void>::error("HarmonicGait", "Position out of limits for joint %d", index);
      }
      float motor_turns = rad_to_turns(position, joints[index]);
       
      odrive_can::msg::ControlMessage msg;
      msg.control_mode = 3; 
      msg.input_mode = 5;  
      msg.input_pos = motor_turns;
      msg.input_vel = 0.0;  
      msg.input_torque = 0.0; 
      control_pubs[index]->publish(msg);
}

result<void> HarmonicGait::request_axis_state(size_t joint_index, uint32_t requested_state)
{
  auto client = this->axis_state_clients[joint_index];
  const auto &joint_name = joint_names[joint_index];

  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      return result<void>::error("HarmonicGait", "Interrupted while waiting for service for joint %s", joint_name.c_str());
    }
    Logger::WARN("HarmonicGait", "Waiting for axis state service for joint %s", joint_name.c_str());
  }

  auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
  request->axis_requested_state = requested_state;

  Logger::INFO("HarmonicGait", "Requesting state %u for joint %s", requested_state, joint_name.c_str());

  auto future = client->async_send_request(request);
  auto timeout = std::chrono::seconds(20);
  auto future_status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout);

  // future status has 3 possible values: SUCCESS, TIMEOUT, INTERRUPTED
  if (future_status == rclcpp::FutureReturnCode::TIMEOUT)
  {
    return result<void>::error("Motor Driver", "Timeout while waiting for response for joint %s, request timedout", joint_name.c_str());
  }

  if (future_status == rclcpp::FutureReturnCode::INTERRUPTED)
  {
    return result<void>::error("Motor Driver", "Interrupted while waiting for response for joint %s", joint_name.c_str());
  }

  if (future_status == rclcpp::FutureReturnCode::SUCCESS && future.get()->procedure_result != 0)
  {
    return result<void>::error("Motor Driver", "Failed to set state for joint %s", joint_name.c_str());
  }

  Logger::INFO("HarmonicGait", "Success");

  return result<void>::success();
}

result<void> HarmonicGait::clear_error(int joint_index)
{
  auto client = clear_error_clients[joint_index];
  const auto &joint_name = joint_names[joint_index];

  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      return result<void>::error("HarmonicGait", "Interrupted while waiting for service for joint %s", joint_name.c_str());
    }
    Logger::WARN("HarmonicGait", "Waiting for clear error service for joint %s", joint_name.c_str());
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

  Logger::INFO("HarmonicGait", "Clearing errors for joint %s", joint_name.c_str());

  auto future = client->async_send_request(request);
  auto timeout = std::chrono::seconds(20);
  auto future_status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout);

  // future status has 3 possible values: SUCCESS, TIMEOUT, INTERRUPTED
  if (future_status == rclcpp::FutureReturnCode::TIMEOUT)
  {
    return result<void>::error("Motor Driver", "Timeout while waiting for response for joint %s, request timedout", joint_name.c_str());
  }

  if (future_status == rclcpp::FutureReturnCode::INTERRUPTED)
  {
    return result<void>::error("Motor Driver", "Interrupted while waiting for response for joint %s", joint_name.c_str());
  }

  Logger::INFO("HarmonicGait", "Success: cleared error for joint %s", joint_name.c_str());

  return result<void>::success();
}

void HarmonicGait::move()
{

  const float MAX_DURATION = 30.0;
  const float amplitude = 2;
  const float frequency = 5;

  const float TWO_PI = 6.28318530718;
  const float PI = 3.14159265359;
  const float link_length = 0.16;

  auto now = this->get_clock()->now();
  auto duration = (now - last_time).seconds();

  if (duration > MAX_DURATION)
  {
    RCLCPP_INFO(this->get_logger(), "Finished Harmonic Gait");
    shutdown();
  }

  Logger::INFO("HarmonicGait", "Waiting for encoder offsets to be loaded");
  while (!offset_loaded)
  {
  };

  std::array<float, NUM_JOINTS> offset = {
      0, 0, 0,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0};

  std::array<float, NUM_JOINTS> jp_mult = {
      1, 1, -1.0,
      1, -1.0, 1.0,
      1, 1, -1,
      1, -1.0, 1.0};

  std::array<float, NUM_JOINTS> standing_positions ={
    0.0, - 0.5, 1.0,   // fl
    0.0, - 0.5, 1.0, // fr
    0.0, - 0.5, 1.0,    // hl
    0.0, - 0.5, 1.0,    // hr
  };

  // get the offsets
  float base_position = amplitude * sin(TWO_PI * frequency * now.seconds());
  float hfe_offset = amplitude * -1.0 * base_position;
  float kfe_offset = amplitude * 2.0 * base_position;

  std::array<double, NUM_JOINTS> positions{};
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    if (i % 3 == 1)
    {
      positions[i] = standing_positions[i] + hfe_offset;
    }
    else if (i % 3 == 2)
    {
      positions[i] = standing_positions[i] + kfe_offset;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Harmonic motion %ld, Base position: %f, HFE offset: %f, KFE offset: %f", now.nanoseconds(), base_position, hfe_offset, kfe_offset);
  // print the positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i, positions[i]);
  }

  RCLCPP_INFO(this->get_logger(), "Duration: %f", duration);
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    set_joint_position(positions[i], i);
  }
}

result<void> HarmonicGait::arm(int joint_index)
{
  auto res = request_axis_state(joint_index, 8);
  if (res.has_error())
  {
    return result<void>::error("Harmonic Gait", "Error: request to arm failed for %s", joints[joint_index].name.c_str());
  }

  return result<void>::success();
}

result<void> HarmonicGait::disarm(int joint_index)
{
  auto res = request_axis_state(joint_index, 1);
  if (res.has_error())
  {
    return result<void>::error("Harmonic Gait", "Issues with init sequence exiting");
  }
  return result<void>::success();
}

// result<void> HarmonicGait::clear_errors(){
//   for(int i = 0; i < NUM_JOINTS -1 ; i++){
//     clear_error(i);
//   }

//   return result<void>::success();
// }

result<void> HarmonicGait::shutdown()
{
  for (size_t i = 0; i < NUM_JOINTS - 1; i++)
  {
    request_axis_state(i, 1);
  }
  rclcpp::shutdown();
  return result<void>::success();
}

result<void> HarmonicGait::load_joint_configs(){
  for(int i = 0; i <  NUM_JOINTS; i++){
    auto name = joints[i].name;

    auto pos_limit = this->get_parameter(name + "_position_limit").as_double();
    joints[i].min_pos = -pos_limit;
    joints[i].max_pos = pos_limit;

    int direction =  this->get_parameter(name + "_direction").as_double();
    joints[i].direction = direction;
    Logger::INFO("HarmonicGait", "Loaded joint config for %s Position is between %f and %f, direction is %f", name.c_str(), joints[i].min_pos, joints[i].max_pos, joints[i].direction);

    for(int i = 0; i < NUM_JOINTS; i++){
      Logger::INFO("HarmonicGait", "Loaded joint config for %s Position is between %f and %f, direction is %f", joints[i].name.c_str(), joints[i].min_pos, joints[i].max_pos, joints[i].direction);
    }
  } 
  return result<void>::success();
}
