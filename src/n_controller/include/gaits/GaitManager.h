#pragma ONCE 
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "data/joint.h"
#include "data/cmd.h"
#include "data/imu.h"
#include "data/gait_type.h"

#include "handlers/onnx_handler.h"

#include "gaits/base_gait.h"
#include "gaits/harmonic_gait.h"
#include "gaits/walking_gait.h"
#include "gaits/sliding_gait.h"
#include "gaits/machine_gait.h"

#include <memory>
#include <map>
#include <string>


namespace newton{

    class GaitManager : public rclcpp::Node {
        public:
            GaitManager();
            ~GaitManager() = default; // 

            // Initialize ROS communication
            void init_publishers();
            void init_subscribers();

            // callbacks 
            void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void gait_change_callback(const std_msgs::msg::String::SharedPtr msg);
            void joints_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
            void joints_velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

            // gait management
            result<std::shared_ptr<BaseGait>> get_gait(GaitType type);
            result<void> set_gait(GaitType type);

            // transition management
            result<void> start_transition(GaitType target_gait);
            result<void> update_transition();

            void update();

            // publish joint positions
            void publish_joint_positions(const std::array<float, BaseGait::NUM_JOINTS>& positions);

            // select gait based on velocity
            result<GaitType> select_gait_by_velocity(float linear_magnitude);

        private:

        std::map<GaitType, std::shared_ptr<BaseGait>> gaits;
        GaitType current_gait_type;
        GaitType target_gait_type;
        float transition_progress; // 0.0  is current gait, 1.0 is target gait
        float transition_duration; 
        bool enable_automatic; // boolean flag to get automatic transitions based on speed. default is true 

        // robot state
        std::array<newton::Joint, 8> joints;
        std::unique_ptr<newton::ImuReading> imu;
        std::unique_ptr<newton::VelocityCmd> cmd;

        // output actions 
        std::array<float, 8> current_positions;
        std::array<float, 8> target_positions;
        std::array<float, 8> joint_cmd_positions; 


        // ROS interfaces
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_cmd_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_gait_pub;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_change_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joints_position_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joints_velocity_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::TimerBase::SharedPtr update_timer;
        
    }

}