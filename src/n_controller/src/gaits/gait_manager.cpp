#include "gaits/gait_manager.h"
using namespace newton;
using namespace std::chrono_literals;
using std::placeholders::_1;
#include <cmath>
#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_events_filter.hpp>


GaitManager::GaitManager()
    :Node("gait_manager"){
    // Declare parameters
    this->declare_parameter("transition_duration", 1.0);
    this->declare_parameter("initial_gait", "standing");
    this->declare_parameter("enable_automatic_transitions", true);
    this->declare_parameter("velocity_threshold_walking", 0.1);
    this->declare_parameter("velocity_threshold_sliding", 0.3);
    this->declare_parameter("velocity_threshold_machine", 0.8);
    
    transition_duration = this->get_parameter("transition_duration").as_double();
    std::string initial_gait = this->get_parameter("initial_gait").as_string();
    enable_automatic = this->get_parameter("enable_automatic_transitions").as_bool();

    // Velocity thresholds for automatic transitions
    velocity_thresholds = {
        {"walking", this->get_parameter("velocity_threshold_walking").as_double()},
        {"sliding", this->get_parameter("velocity_threshold_sliding").as_double()},
        {"machine", this->get_parameter("velocity_threshold_machine").as_double()}
    };


    init_publishers();
    init_subscribers();

    // timer 
    update_timer = this->create_wall_timer(
        CONTROLLER_PERIOD, std::bind(&GaitManager::update, this));
    

    // Initialize gaits
    gaits[GaitType::STANDING] = std::make_shared<StandingGait>();
    gaits[GaitType::WALKING] = std::make_shared<WalkingGait>();
    gaits[GaitType::SLIDING] = std::make_shared<SlidingGait>();
    gaits[GaitType::HARMONIC] = std::make_shared<HarmonicGait>();
    gaits[GaitType::MACHINE_LEARNING] = std::make_shared<MachineGait>();

}

result<void> GaitManager::init_publishers(){
    joint_cmd_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "joint_cmd_positions", 10);

    current_gait_pub = this->create_publisher<std_msgs::msg::String>(
        "current_gait", 10);

    return result<void>::success();
}

result<void> GaitManager::init_subscribers(){
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "vel_cmds", 10, std::bind(&GaitManager::cmd_vel_callback, this, _1));

    gait_change_sub = this->create_subscription<std_msgs::msg::String>(
        "change_gait", 10, std::bind(&GaitManager::gait_change_callback, this, _1));

    joint_states_pos_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "joint_state_positions", 10, std::bind(&GaitManager::joint_states_pos_cb, this, _1));

    joint_states_vel_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "joint_state_velocities", 10, std::bind(&GaitManager::joint_states_vel_cb, this, _1));

    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10, std::bind(&GaitManager::imu_state_cb, this, _1));

    return result<void>::success();
}

void GaitManager::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd->linear_velocity = Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
    cmd->angular_velocity = Vector3(msg->angular.x, msg->angular.y, msg->angular.z);
}

void GaitManager::gait_change_callback(const std_msgs::msg::String::SharedPtr msg) {
    GaitType requested_gait = string_to_gait_type(msg->data);
    start_transition(requested_gait);
}

void GaitManager::joint_states_pos_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        current_joint_positions[i] = msg->data[i];
    }
}

void GaitManager::joint_states_vel_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        current_joint_velocities[i] = msg->data[i];
    }
}
void GaitManager::imu_state_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
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
}



void GaitManager::update_odrive_ready(std_msgs::msg::Bool::SharedPtr msg) {
    odrive_ready = msg->data;
    if(!odrive_ready){
        Logger::WARN("GM","Odrive not ready");
        return;
    }

    // if ready but dont have init 
    //  if odrive is ready, set the standing positions

    
    // set the standing positions
    set_joint_positions(BaseGait::standing_positions);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    update_timer = this->create_wall_timer(
        CONTROLLER_PERIOD, std::bind(&GaitManager::update, this));
   
    last_time = this->get_clock()->now();
}

void GaitManager::update(){
    if(!odrive_ready){
        return;
    }

    if(transition_progress < 1.0){
        transition_progress += 0.02 / transition_duration; // 20ms update

    }

    // check if we are in transition
    // update the observations array
    observations[0] = imu->angular_velocity.x;
    observations[1] = imu->angular_velocity.y;
    observations[2] = imu->angular_velocity.z;

    // projected gravity
    observations[3] = imu->projected_gravity.x;
    observations[4] = imu->projected_gravity.y;
    observations[5] = imu->projected_gravity.z;
    // commands
    observations[6] = cmd->linear_velocity.y; // the commands use y as the forward
    observations[7] = cmd->linear_velocity.x; // and x as the sideways
    observations[8] = cmd->angular_velocity.z;


    // update the joint positions
    for (int i = 0; i < NUM_JOINTS; i++) {
        // if current_gait is machine gait then pass in the delta
        if (current_gait_type == GaitType::MACHINE_LEARNING){
            observations[POSITION_IDX + i] = current_joint_positions[i] - BaseGait::standing_positions[i];
        } else{ 
            observations[POSITION_IDX + i] = current_joint_positions[i];
        }
    } 

    // joint velocities
    for (int i = 0; i < NUM_JOINTS; i++) {
        observations[VELOCITY_IDX + i] = current_joint_velocities[i];
    }

    // previous actions
    for (int i = 0; i < NUM_JOINTS; i++) {
        observations[PREV_ACTION_IDX + i] = previous_actions[i];
    }


    auto current_gait = get_gait(current_gait_type);
    std::array<float, NUM_JOINTS> actions;
    

    // joint velocities
    if(transition_progress < 1.0){
        //  get the target gait
        auto target_gait = get_gait(target_gait_type);
        // get the proposed positions from the current and target gaits
        auto curr_actions = current_gait->update(observations);
        auto target_actions  = target_gait->update(observations);

        // blend the actions 
        for (int i = 0; i < NUM_JOINTS; i++) {
            actions[i] = curr_actions[i] * (1.0 - transition_progress) +
                                    target_actions[i] * transition_progress;
        }
    } else {
        // get the proposed positions from the current gait
        actions = current_gait->update(observations);
     }

    // publish the joint positions
    publish_joint_positions(actions);
     // update the previous actions
    for (int i = 0; i < NUM_JOINTS; i++) {
        previous_actions[i] = actions[i];
    }   

}


