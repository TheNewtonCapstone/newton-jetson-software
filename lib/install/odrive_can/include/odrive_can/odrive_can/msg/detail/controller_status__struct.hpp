// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__STRUCT_HPP_
#define ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__odrive_can__msg__ControllerStatus __attribute__((deprecated))
#else
# define DEPRECATED__odrive_can__msg__ControllerStatus __declspec(deprecated)
#endif

namespace odrive_can
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControllerStatus_
{
  using Type = ControllerStatus_<ContainerAllocator>;

  explicit ControllerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_estimate = 0.0f;
      this->vel_estimate = 0.0f;
      this->torque_target = 0.0f;
      this->torque_estimate = 0.0f;
      this->iq_setpoint = 0.0f;
      this->iq_measured = 0.0f;
      this->active_errors = 0ul;
      this->axis_state = 0;
      this->procedure_result = 0;
      this->trajectory_done_flag = false;
    }
  }

  explicit ControllerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_estimate = 0.0f;
      this->vel_estimate = 0.0f;
      this->torque_target = 0.0f;
      this->torque_estimate = 0.0f;
      this->iq_setpoint = 0.0f;
      this->iq_measured = 0.0f;
      this->active_errors = 0ul;
      this->axis_state = 0;
      this->procedure_result = 0;
      this->trajectory_done_flag = false;
    }
  }

  // field types and members
  using _pos_estimate_type =
    float;
  _pos_estimate_type pos_estimate;
  using _vel_estimate_type =
    float;
  _vel_estimate_type vel_estimate;
  using _torque_target_type =
    float;
  _torque_target_type torque_target;
  using _torque_estimate_type =
    float;
  _torque_estimate_type torque_estimate;
  using _iq_setpoint_type =
    float;
  _iq_setpoint_type iq_setpoint;
  using _iq_measured_type =
    float;
  _iq_measured_type iq_measured;
  using _active_errors_type =
    uint32_t;
  _active_errors_type active_errors;
  using _axis_state_type =
    uint8_t;
  _axis_state_type axis_state;
  using _procedure_result_type =
    uint8_t;
  _procedure_result_type procedure_result;
  using _trajectory_done_flag_type =
    bool;
  _trajectory_done_flag_type trajectory_done_flag;

  // setters for named parameter idiom
  Type & set__pos_estimate(
    const float & _arg)
  {
    this->pos_estimate = _arg;
    return *this;
  }
  Type & set__vel_estimate(
    const float & _arg)
  {
    this->vel_estimate = _arg;
    return *this;
  }
  Type & set__torque_target(
    const float & _arg)
  {
    this->torque_target = _arg;
    return *this;
  }
  Type & set__torque_estimate(
    const float & _arg)
  {
    this->torque_estimate = _arg;
    return *this;
  }
  Type & set__iq_setpoint(
    const float & _arg)
  {
    this->iq_setpoint = _arg;
    return *this;
  }
  Type & set__iq_measured(
    const float & _arg)
  {
    this->iq_measured = _arg;
    return *this;
  }
  Type & set__active_errors(
    const uint32_t & _arg)
  {
    this->active_errors = _arg;
    return *this;
  }
  Type & set__axis_state(
    const uint8_t & _arg)
  {
    this->axis_state = _arg;
    return *this;
  }
  Type & set__procedure_result(
    const uint8_t & _arg)
  {
    this->procedure_result = _arg;
    return *this;
  }
  Type & set__trajectory_done_flag(
    const bool & _arg)
  {
    this->trajectory_done_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    odrive_can::msg::ControllerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const odrive_can::msg::ControllerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      odrive_can::msg::ControllerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      odrive_can::msg::ControllerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__odrive_can__msg__ControllerStatus
    std::shared_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__odrive_can__msg__ControllerStatus
    std::shared_ptr<odrive_can::msg::ControllerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControllerStatus_ & other) const
  {
    if (this->pos_estimate != other.pos_estimate) {
      return false;
    }
    if (this->vel_estimate != other.vel_estimate) {
      return false;
    }
    if (this->torque_target != other.torque_target) {
      return false;
    }
    if (this->torque_estimate != other.torque_estimate) {
      return false;
    }
    if (this->iq_setpoint != other.iq_setpoint) {
      return false;
    }
    if (this->iq_measured != other.iq_measured) {
      return false;
    }
    if (this->active_errors != other.active_errors) {
      return false;
    }
    if (this->axis_state != other.axis_state) {
      return false;
    }
    if (this->procedure_result != other.procedure_result) {
      return false;
    }
    if (this->trajectory_done_flag != other.trajectory_done_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControllerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControllerStatus_

// alias to use template instance with default allocator
using ControllerStatus =
  odrive_can::msg::ControllerStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace odrive_can

#endif  // ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__STRUCT_HPP_
