#include "gaits/base_gait.h"
#include "logger.h"
#include "data/vectors.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;
using namespace std::chrono_literals;

BaseGait::BaseGait(const std::string node_name, const rclcpp::NodeOptions &options)
    : Node(node_name)
{
};

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

    load_joint_configs();
    init_pubs();
    init_clients();
    init_subs();

    imu = std::make_unique<Imu>();

    move_timer = this->create_wall_timer(20ms, std::bind(&BaseGait::move, this));
    last_time = this->get_clock()->now();
    rclcpp::shutdown();
}

result<void> BaseGait::init_clients()
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

    // for (size_t i = 0; i < NUM_JOINTS; i++)
    // {
    //   if (disarm(i).has_error())
    //   {
    //     Logger::ERROR("Harmonic Gait", "Error: request to disarm failed for %s", joint_names[i].c_str());
    //     shutdown();
    //   }
    // }

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
            shutdown();
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

result<void> BaseGait::init_pubs()
{
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
        auto name = joint_names[i];

        control_pubs[i] =
            this->create_publisher<odrive_can::msg::ControlMessage>(name + "/control_message", 10);

        RCLCPP_INFO(this->get_logger(), "Created a publisher to to %s", name.c_str());
    }

    return result<void>::success();
}

result<void> BaseGait::init_subs()
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

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10,
        [=, this](const sensor_msgs::msg::Imu::SharedPtr msg)
        {
            this->update_imu(msg);
        });

    return result<void>::success();
}

void BaseGait::update_driver_status(
    const odrive_can::msg::ODriveStatus::SharedPtr msg, int joint_index)
{
}

result<void> BaseGait::update_joint_state(
    const odrive_can::msg::ControllerStatus::SharedPtr msg, int joint_index)
{
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
        offset_loaded = true;
    }
    return result<void>::success();
}

result<void> BaseGait::update_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const auto dt = static_cast<double>(msg->header.stamp.nanosec - imu->timestamp) / 1e9;
    
    imu->timestamp = msg->header.stamp.nanosec;
    imu->linear_acceleration = Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    imu->linear_velocity += imu->linear_acceleration * dt;
    imu->angular_velocity = Vector3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    imu->rotation = quat_to_rpy(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    imu->projected_gravity = quat_to_proj_gravity(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    return result<void>::success();
}

result<void> BaseGait::set_joint_position(int joint_index, float position)
{
    // takes in the position in rad/ and apply
    // the gear ratio to get the motor turns
    if (!is_within_limits(position, joints[joint_index]))
    {
        return result<void>::error("HarmonicGait", "Position out of limits for joint %d", joint_index);
    }
    float motor_turns = rad_to_turns(position, joints[joint_index]);

    odrive_can::msg::ControlMessage msg;
    msg.control_mode = 3;
    msg.input_mode = 5;
    msg.input_pos = motor_turns;
    msg.input_vel = 0.0;
    msg.input_torque = 0.0;
    control_pubs[joint_index]->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Set joint position to %f for %d", position, joint_index);
}

result<void> BaseGait::request_axis_state(size_t joint_index, uint32_t requested_state)
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

result<void> BaseGait::clear_error(int joint_index)
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

result<void> BaseGait::arm(int joint_index)
{
    auto res = request_axis_state(joint_index, 8);
    if (res.has_error())
    {
        return result<void>::error("Harmonic Gait", "Error: request to arm failed for %s", joints[joint_index].name.c_str());
    }

    return result<void>::success();
}

result<void> BaseGait::disarm(int joint_index)
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

result<void> BaseGait::shutdown()
{
    for (size_t i = 0; i < NUM_JOINTS - 1; i++)
    {
        request_axis_state(i, 1);
    }
    rclcpp::shutdown();
    return result<void>::success();
}

result<void> BaseGait::load_joint_configs()
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        auto name = joints[i].name;

        auto pos_limit = this->get_parameter(name + "_position_limit").as_double();
        joints[i].min_pos = -pos_limit;
        joints[i].max_pos = pos_limit;

        int direction = this->get_parameter(name + "_direction").as_double();
        joints[i].direction = direction;
        Logger::INFO("HarmonicGait", "Loaded joint config for %s Position is between %f and %f, direction is %f", name.c_str(), joints[i].min_pos, joints[i].max_pos, joints[i].direction);
    }
    return result<void>::success();
}
