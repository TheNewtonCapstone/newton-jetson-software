#include <chrono>
#include <functional>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    
    std::vector<std::string> joint_names = 
    {
      "fl_haa", "fl_hfe", "fl_kfe",
      "fr_haa", "fr_hfe", "fr_kfe",
      "rl_haa", "rl_hfe", "rl_kfe",
      "rr_haa", "rr_hfe", "rr_kfe"
    };

    for(auto joint_name : joint_names)
    {
      this -> declare_parameter(std::string("motor_") + joint_name + "_node_id", "00");
    }

    auto timer_callback = [this](){
    std::vector<std::string> joint_names = 
    {
      "fl_haa", "fl_hfe", "fl_kfe",
      "fr_haa", "fr_hfe", "fr_kfe",
      "rl_haa", "rl_hfe", "rl_kfe",
      "rr_haa", "rr_hfe", "rr_kfe"
    };
      // auto params = this->get_parameter("motor_fl_haa_node_id");
      // RCLCPP_INFO(this->get_logger(), "motor_fl_haa_node_id: %s", params.as_string().c_str());
      for (auto joint_name : joint_names)
      {
        auto params = this->get_parameter(std::string("motor_") + joint_name + "_node_id");
        RCLCPP_INFO(this->get_logger(), "%s: %s", joint_name.c_str(), params.as_string().c_str());
      }
    };
    timer_ = this->create_wall_timer(1000ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}