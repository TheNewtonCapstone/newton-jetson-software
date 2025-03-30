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
    : Node(node_name) {
  last_time = this->get_clock()->now();
};
