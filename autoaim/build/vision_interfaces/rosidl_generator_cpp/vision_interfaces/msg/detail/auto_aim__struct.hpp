// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vision_interfaces:msg/AutoAim.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__STRUCT_HPP_
#define VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__vision_interfaces__msg__AutoAim __attribute__((deprecated))
#else
# define DEPRECATED__vision_interfaces__msg__AutoAim __declspec(deprecated)
#endif

namespace vision_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AutoAim_
{
  using Type = AutoAim_<ContainerAllocator>;

  explicit AutoAim_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->aim_yaw = 0.0f;
      this->aim_pitch = 0.0f;
      this->fire = 0;
      this->tracking = 0;
    }
  }

  explicit AutoAim_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->aim_yaw = 0.0f;
      this->aim_pitch = 0.0f;
      this->fire = 0;
      this->tracking = 0;
    }
  }

  // field types and members
  using _aim_yaw_type =
    float;
  _aim_yaw_type aim_yaw;
  using _aim_pitch_type =
    float;
  _aim_pitch_type aim_pitch;
  using _fire_type =
    uint8_t;
  _fire_type fire;
  using _tracking_type =
    uint8_t;
  _tracking_type tracking;

  // setters for named parameter idiom
  Type & set__aim_yaw(
    const float & _arg)
  {
    this->aim_yaw = _arg;
    return *this;
  }
  Type & set__aim_pitch(
    const float & _arg)
  {
    this->aim_pitch = _arg;
    return *this;
  }
  Type & set__fire(
    const uint8_t & _arg)
  {
    this->fire = _arg;
    return *this;
  }
  Type & set__tracking(
    const uint8_t & _arg)
  {
    this->tracking = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vision_interfaces::msg::AutoAim_<ContainerAllocator> *;
  using ConstRawPtr =
    const vision_interfaces::msg::AutoAim_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vision_interfaces::msg::AutoAim_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vision_interfaces::msg::AutoAim_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vision_interfaces__msg__AutoAim
    std::shared_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vision_interfaces__msg__AutoAim
    std::shared_ptr<vision_interfaces::msg::AutoAim_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AutoAim_ & other) const
  {
    if (this->aim_yaw != other.aim_yaw) {
      return false;
    }
    if (this->aim_pitch != other.aim_pitch) {
      return false;
    }
    if (this->fire != other.fire) {
      return false;
    }
    if (this->tracking != other.tracking) {
      return false;
    }
    return true;
  }
  bool operator!=(const AutoAim_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AutoAim_

// alias to use template instance with default allocator
using AutoAim =
  vision_interfaces::msg::AutoAim_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vision_interfaces

#endif  // VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__STRUCT_HPP_
