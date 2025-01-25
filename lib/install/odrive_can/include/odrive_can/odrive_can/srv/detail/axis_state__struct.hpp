// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from odrive_can:srv/AxisState.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_HPP_
#define ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__odrive_can__srv__AxisState_Request __attribute__((deprecated))
#else
# define DEPRECATED__odrive_can__srv__AxisState_Request __declspec(deprecated)
#endif

namespace odrive_can
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AxisState_Request_
{
  using Type = AxisState_Request_<ContainerAllocator>;

  explicit AxisState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->axis_requested_state = 0ul;
    }
  }

  explicit AxisState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->axis_requested_state = 0ul;
    }
  }

  // field types and members
  using _axis_requested_state_type =
    uint32_t;
  _axis_requested_state_type axis_requested_state;

  // setters for named parameter idiom
  Type & set__axis_requested_state(
    const uint32_t & _arg)
  {
    this->axis_requested_state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    odrive_can::srv::AxisState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const odrive_can::srv::AxisState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      odrive_can::srv::AxisState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      odrive_can::srv::AxisState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__odrive_can__srv__AxisState_Request
    std::shared_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__odrive_can__srv__AxisState_Request
    std::shared_ptr<odrive_can::srv::AxisState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AxisState_Request_ & other) const
  {
    if (this->axis_requested_state != other.axis_requested_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const AxisState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AxisState_Request_

// alias to use template instance with default allocator
using AxisState_Request =
  odrive_can::srv::AxisState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace odrive_can


#ifndef _WIN32
# define DEPRECATED__odrive_can__srv__AxisState_Response __attribute__((deprecated))
#else
# define DEPRECATED__odrive_can__srv__AxisState_Response __declspec(deprecated)
#endif

namespace odrive_can
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct AxisState_Response_
{
  using Type = AxisState_Response_<ContainerAllocator>;

  explicit AxisState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active_errors = 0ul;
      this->axis_state = 0;
      this->procedure_result = 0;
    }
  }

  explicit AxisState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->active_errors = 0ul;
      this->axis_state = 0;
      this->procedure_result = 0;
    }
  }

  // field types and members
  using _active_errors_type =
    uint32_t;
  _active_errors_type active_errors;
  using _axis_state_type =
    uint8_t;
  _axis_state_type axis_state;
  using _procedure_result_type =
    uint8_t;
  _procedure_result_type procedure_result;

  // setters for named parameter idiom
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

  // constant declarations

  // pointer types
  using RawPtr =
    odrive_can::srv::AxisState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const odrive_can::srv::AxisState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      odrive_can::srv::AxisState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      odrive_can::srv::AxisState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__odrive_can__srv__AxisState_Response
    std::shared_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__odrive_can__srv__AxisState_Response
    std::shared_ptr<odrive_can::srv::AxisState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AxisState_Response_ & other) const
  {
    if (this->active_errors != other.active_errors) {
      return false;
    }
    if (this->axis_state != other.axis_state) {
      return false;
    }
    if (this->procedure_result != other.procedure_result) {
      return false;
    }
    return true;
  }
  bool operator!=(const AxisState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AxisState_Response_

// alias to use template instance with default allocator
using AxisState_Response =
  odrive_can::srv::AxisState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace odrive_can

namespace odrive_can
{

namespace srv
{

struct AxisState
{
  using Request = odrive_can::srv::AxisState_Request;
  using Response = odrive_can::srv::AxisState_Response;
};

}  // namespace srv

}  // namespace odrive_can

#endif  // ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_HPP_
