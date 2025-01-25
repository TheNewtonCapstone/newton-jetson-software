#ifndef MOTOR_H
#define MOTOR_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "minimal_composition/visibility.h"

class Motor : public rclcpp::Node
{
public:
  MINIMAL_COMPOSITION_PUBLIC SubscriberNode(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  
