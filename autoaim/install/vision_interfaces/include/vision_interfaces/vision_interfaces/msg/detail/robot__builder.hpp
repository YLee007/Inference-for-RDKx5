// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vision_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__ROBOT__BUILDER_HPP_
#define VISION_INTERFACES__MSG__DETAIL__ROBOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vision_interfaces/msg/detail/robot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vision_interfaces
{

namespace msg
{

namespace builder
{

class Init_Robot_muzzle_speed
{
public:
  explicit Init_Robot_muzzle_speed(::vision_interfaces::msg::Robot & msg)
  : msg_(msg)
  {}
  ::vision_interfaces::msg::Robot muzzle_speed(::vision_interfaces::msg::Robot::_muzzle_speed_type arg)
  {
    msg_.muzzle_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vision_interfaces::msg::Robot msg_;
};

class Init_Robot_self_pitch
{
public:
  explicit Init_Robot_self_pitch(::vision_interfaces::msg::Robot & msg)
  : msg_(msg)
  {}
  Init_Robot_muzzle_speed self_pitch(::vision_interfaces::msg::Robot::_self_pitch_type arg)
  {
    msg_.self_pitch = std::move(arg);
    return Init_Robot_muzzle_speed(msg_);
  }

private:
  ::vision_interfaces::msg::Robot msg_;
};

class Init_Robot_self_yaw
{
public:
  explicit Init_Robot_self_yaw(::vision_interfaces::msg::Robot & msg)
  : msg_(msg)
  {}
  Init_Robot_self_pitch self_yaw(::vision_interfaces::msg::Robot::_self_yaw_type arg)
  {
    msg_.self_yaw = std::move(arg);
    return Init_Robot_self_pitch(msg_);
  }

private:
  ::vision_interfaces::msg::Robot msg_;
};

class Init_Robot_foe_color
{
public:
  explicit Init_Robot_foe_color(::vision_interfaces::msg::Robot & msg)
  : msg_(msg)
  {}
  Init_Robot_self_yaw foe_color(::vision_interfaces::msg::Robot::_foe_color_type arg)
  {
    msg_.foe_color = std::move(arg);
    return Init_Robot_self_yaw(msg_);
  }

private:
  ::vision_interfaces::msg::Robot msg_;
};

class Init_Robot_mode
{
public:
  Init_Robot_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Robot_foe_color mode(::vision_interfaces::msg::Robot::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Robot_foe_color(msg_);
  }

private:
  ::vision_interfaces::msg::Robot msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vision_interfaces::msg::Robot>()
{
  return vision_interfaces::msg::builder::Init_Robot_mode();
}

}  // namespace vision_interfaces

#endif  // VISION_INTERFACES__MSG__DETAIL__ROBOT__BUILDER_HPP_
