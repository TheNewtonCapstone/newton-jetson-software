#include "nodes/gamepad_node.h"

using namespace newton;

GamepadNode::GamepadNode() : Node("gamepad_node")
{
    m_input_manager = std::make_unique<gainput::InputManager>();

    m_gamepad_id = m_input_manager->CreateDevice<gainput::InputDevicePad>();

    m_input_map = std::make_unique<gainput::InputMap>(*m_input_manager);
    m_input_map->MapFloat(HORIZONTAL, m_gamepad_id, gainput::PadButtonLeftStickX);
    m_input_map->MapFloat(VERTICAL, m_gamepad_id, gainput::PadButtonLeftStickY);
    m_input_map->MapFloat(ROTATE, m_gamepad_id, gainput::PadButtonRightStickX);

    m_publisher = create_publisher<geometry_msgs::msg::Twist>("/vel_cmds/gamepad", 10);
    m_timer = create_wall_timer(std::chrono::milliseconds(10), [this]()
                                { read_gamepad(); });
}

void newton::GamepadNode::read_gamepad()
{
    m_input_manager->Update();

    const auto horizontal = m_input_map->GetFloat(HORIZONTAL);
    const auto vertical = m_input_map->GetFloat(VERTICAL);
    const auto rotate = m_input_map->GetFloat(ROTATE);

    auto linear = Vector3(horizontal, vertical, 0.0f);
    auto angular = Vector3(0.0f, 0.0f, rotate);

    publish_twist(linear, angular);
}

void newton::GamepadNode::publish_twist(const Vector3 &linear, const Vector3 &angular)
{
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear.x;
    twist_msg.linear.y = linear.y;
    twist_msg.linear.z = linear.z;

    twist_msg.angular.x = angular.x;
    twist_msg.angular.y = angular.y;
    twist_msg.angular.z = angular.z;

    m_publisher->publish(twist_msg);
}
