#pragma once

#include "data/cmd.h"
#include "data/imu.h"
#include "data/joint.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "result.h"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "unit.h"

namespace newton
{
    class BaseGait : public rclcpp::Node
    {
    public:
        explicit BaseGait(const std::string name, const bool should_set_to_standing,
                          const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~BaseGait() = default;

        static constexpr uint8_t NUM_JOINTS = 8;

        result<void> init();
        result<void> init_clients();
        result<void> init_pubs();
        result<void> init_subs();
        result<void> shutdown();

        result<void> set_joints_position(
            const std::array<float, NUM_JOINTS> &positions);

    protected:
        std::unordered_map<std::string, float> phases = {
            {"fl", 0.0}, // front left
            {"fr", PI},  // front right
            {"hl", PI},  // hind left
            {"hr", 0.0}, // hind right
        };

        const std::array<std::string, NUM_JOINTS> joint_names = {
            "fl_hfe", "fl_kfe", // front left
            "fr_hfe", "fr_kfe", // front right
            "hl_hfe", "hl_kfe", // hind left
            "hr_hfe", "hr_kfe", // hind right
        };

        std::unordered_map<std::string, std::array<float, 2>> leg_standing_positions =
            {
                {"fl", {0.8, -1.4}}, // front left
                {"fr", {0.8, -1.4}},   // front right
                {"hl", {0.8, -1.4}},   // hind left
                {"hr", {0.8, -1.4}}, // hind right
            };

        std::array<float, NUM_JOINTS> standing_positions = {
            0.8, -1.4, // front left
            0.8, -1.4, // front right
            0.8, -1.4, // hind left
            0.8, -1.4, // hind right

        };

        // map the legs to ids
        const std::unordered_map<std::string, std::array<int, 2>> leg_ids = {
            {"fl", {0, 1}},   // front left
            {"fr", {2, 3}},   // front right
            {"hl", {4, 5}},   // hind left
            {"hr", {6, 7}}, // hind right
        };

        std::array<newton::Joint, NUM_JOINTS> joints;
        std::unique_ptr<newton::ImuReading> imu;
        std::unique_ptr<newton::VelocityCmd> cmd;

        bool should_set_to_standing = false;
        bool odrive_ready = false;

        rclcpp::TimerBase::SharedPtr move_timer;
        rclcpp::Time last_time;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
            joints_position_pub;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
            joints_velocity_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
            joints_torque_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
            joints_position_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr odrive_ready_sub;

        // update methods
        result<void> update_joints_position(
            const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        result<void> update_joints_velocity(
            const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        result<void> update_joints_torque(
            const std_msgs::msg::Float32MultiArray::SharedPtr msg);

        result<void> update_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
        result<void> update_cmd(const geometry_msgs::msg::Twist::SharedPtr msg);
        result<void> update_odrive_ready(const std_msgs::msg::Bool::SharedPtr msg);

        virtual result<void> move() = 0;
    };
} // namespace newton
