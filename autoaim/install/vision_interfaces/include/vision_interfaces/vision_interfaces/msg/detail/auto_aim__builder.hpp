// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vision_interfaces:msg/AutoAim.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__BUILDER_HPP_
#define VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vision_interfaces/msg/detail/auto_aim__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vision_interfaces
{

namespace msg
{

namespace builder
{

class Init_AutoAim_tracking
{
public:
  explicit Init_AutoAim_tracking(::vision_interfaces::msg::AutoAim & msg)
  : msg_(msg)
  {}
  ::vision_interfaces::msg::AutoAim tracking(::vision_interfaces::msg::AutoAim::_tracking_type arg)
  {
    msg_.tracking = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vision_interfaces::msg::AutoAim msg_;
};

class Init_AutoAim_fire
{
public:
  explicit Init_AutoAim_fire(::vision_interfaces::msg::AutoAim & msg)
  : msg_(msg)
  {}
  Init_AutoAim_tracking fire(::vision_interfaces::msg::AutoAim::_fire_type arg)
  {
    msg_.fire = std::move(arg);
    return Init_AutoAim_tracking(msg_);
  }

private:
  ::vision_interfaces::msg::AutoAim msg_;
};

class Init_AutoAim_aim_pitch
{
public:
  explicit Init_AutoAim_aim_pitch(::vision_interfaces::msg::AutoAim & msg)
  : msg_(msg)
  {}
  Init_AutoAim_fire aim_pitch(::vision_interfaces::msg::AutoAim::_aim_pitch_type arg)
  {
    msg_.aim_pitch = std::move(arg);
    return Init_AutoAim_fire(msg_);
  }

private:
  ::vision_interfaces::msg::AutoAim msg_;
};

class Init_AutoAim_aim_yaw
{
public:
  Init_AutoAim_aim_yaw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AutoAim_aim_pitch aim_yaw(::vision_interfaces::msg::AutoAim::_aim_yaw_type arg)
  {
    msg_.aim_yaw = std::move(arg);
    return Init_AutoAim_aim_pitch(msg_);
  }

private:
  ::vision_interfaces::msg::AutoAim msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vision_interfaces::msg::AutoAim>()
{
  return vision_interfaces::msg::builder::Init_AutoAim_aim_yaw();
}

}  // namespace vision_interfaces

#endif  // VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__BUILDER_HPP_
