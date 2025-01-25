// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from odrive_can:msg/ODriveStatus.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__STRUCT_HPP_
#define ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__odrive_can__msg__ODriveStatus __attribute__((deprecated))
#else
# define DEPRECATED__odrive_can__msg__ODriveStatus __declspec(deprecated)
#endif

namespace odrive_can
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ODriveStatus_
{
  using Type = ODriveStatus_<ContainerAllocator>;

  explicit ODriveStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bus_voltage = 0.0f;
      this->bus_current = 0.0f;
      this->fet_temperature = 0.0f;
      this->motor_temperature = 0.0f;
      this->active_errors = 0ul;
      this->disarm_reason = 0ul;
    }
  }

  explicit ODriveStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bus_voltage = 0.0f;
      this->bus_current = 0.0f;
      this->fet_temperature = 0.0f;
      this->motor_temperature = 0.0f;
      this->active_errors = 0ul;
      this->disarm_reason = 0ul;
    }
  }

  // field types and members
  using _bus_voltage_type =
    float;
  _bus_voltage_type bus_voltage;
  using _bus_current_type =
    float;
  _bus_current_type bus_current;
  using _fet_temperature_type =
    float;
  _fet_temperature_type fet_temperature;
  using _motor_temperature_type =
    float;
  _motor_temperature_type motor_temperature;
  using _active_errors_type =
    uint32_t;
  _active_errors_type active_errors;
  using _disarm_reason_type =
    uint32_t;
  _disarm_reason_type disarm_reason;

  // setters for named parameter idiom
  Type & set__bus_voltage(
    const float & _arg)
  {
    this->bus_voltage = _arg;
    return *this;
  }
  Type & set__bus_current(
    const float & _arg)
  {
    this->bus_current = _arg;
    return *this;
  }
  Type & set__fet_temperature(
    const float & _arg)
  {
    this->fet_temperature = _arg;
    return *this;
  }
  Type & set__motor_temperature(
    const float & _arg)
  {
    this->motor_temperature = _arg;
    return *this;
  }
  Type & set__active_errors(
    const uint32_t & _arg)
  {
    this->active_errors = _arg;
    return *this;
  }
  Type & set__disarm_reason(
    const uint32_t & _arg)
  {
    this->disarm_reason = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    odrive_can::msg::ODriveStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const odrive_can::msg::ODriveStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      odrive_can::msg::ODriveStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      odrive_can::msg::ODriveStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__odrive_can__msg__ODriveStatus
    std::shared_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__odrive_can__msg__ODriveStatus
    std::shared_ptr<odrive_can::msg::ODriveStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ODriveStatus_ & other) const
  {
    if (this->bus_voltage != other.bus_voltage) {
      return false;
    }
    if (this->bus_current != other.bus_current) {
      return false;
    }
    if (this->fet_temperature != other.fet_temperature) {
      return false;
    }
    if (this->motor_temperature != other.motor_temperature) {
      return false;
    }
    if (this->active_errors != other.active_errors) {
      return false;
    }
    if (this->disarm_reason != other.disarm_reason) {
      return false;
    }
    return true;
  }
  bool operator!=(const ODriveStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ODriveStatus_

// alias to use template instance with default allocator
using ODriveStatus =
  odrive_can::msg::ODriveStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace odrive_can

#endif  // ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__STRUCT_HPP_
