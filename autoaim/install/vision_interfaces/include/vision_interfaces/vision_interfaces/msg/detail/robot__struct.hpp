// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__ROBOT__STRUCT_HPP_
#define VISION_INTERFACES__MSG__DETAIL__ROBOT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vision_interfaces__msg__Robot __attribute__((deprecated))
#else
# define DEPRECATED__vision_interfaces__msg__Robot __declspec(deprecated)
#endif

namespace vision_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Robot_
{
  using Type = Robot_<ContainerAllocator>;

  explicit Robot_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->foe_color = 0;
      this->self_yaw = 0.0f;
      this->self_pitch = 0.0f;
      this->muzzle_speed = 0.0f;
    }
  }

  explicit Robot_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->foe_color = 0;
      this->self_yaw = 0.0f;
      this->self_pitch = 0.0f;
      this->muzzle_speed = 0.0f;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _foe_color_type =
    uint8_t;
  _foe_color_type foe_color;
  using _self_yaw_type =
    float;
  _self_yaw_type self_yaw;
  using _self_pitch_type =
    float;
  _self_pitch_type self_pitch;
  using _muzzle_speed_type =
    float;
  _muzzle_speed_type muzzle_speed;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__foe_color(
    const uint8_t & _arg)
  {
    this->foe_color = _arg;
    return *this;
  }
  Type & set__self_yaw(
    const float & _arg)
  {
    this->self_yaw = _arg;
    return *this;
  }
  Type & set__self_pitch(
    const float & _arg)
  {
    this->self_pitch = _arg;
    return *this;
  }
  Type & set__muzzle_speed(
    const float & _arg)
  {
    this->muzzle_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_interfaces::msg::Robot_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_interfaces::msg::Robot_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_interfaces::msg::Robot_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_interfaces::msg::Robot_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_interfaces::msg::Robot_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_interfaces::msg::Robot_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_interfaces::msg::Robot_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_interfaces::msg::Robot_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_interfaces::msg::Robot_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_interfaces::msg::Robot_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_interfaces__msg__Robot
    std::shared_ptr<vision_interfaces::msg::Robot_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_interfaces__msg__Robot
    std::shared_ptr<vision_interfaces::msg::Robot_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Robot_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->foe_color != other.foe_color) {
      return false;
    }
    if (this->self_yaw != other.self_yaw) {
      return false;
    }
    if (this->self_pitch != other.self_pitch) {
      return false;
    }
    if (this->muzzle_speed != other.muzzle_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const Robot_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Robot_

// alias to use template instance with default allocator
using Robot =
  vision_interfaces::msg::Robot_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_interfaces

#endif  // VISION_INTERFACES__MSG__DETAIL__ROBOT__STRUCT_HPP_
