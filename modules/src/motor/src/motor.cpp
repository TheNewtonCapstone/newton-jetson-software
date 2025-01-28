#include "motor.h"

using namespace newton;
Motor::Motor(const std::string& _id)
    :Node("odrive_node_" + _id), position(0.0){

  state_sub = create_subscription<odrive_can::msg::ControllerStatus>(
      "controller_status", 10,
      std::bind(&Motor::statusCallback, this, std::placeholders::_1));
};

void Motor::statusCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
    printf("Ping\n");
}


