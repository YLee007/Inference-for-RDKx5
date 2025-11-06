// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hbm_img_msgs:msg/HbmMsg1080P.idl
// generated code does not contain a copyright notice

#ifndef HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__STRUCT_HPP_
#define HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'time_stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hbm_img_msgs__msg__HbmMsg1080P __attribute__((deprecated))
#else
# define DEPRECATED__hbm_img_msgs__msg__HbmMsg1080P __declspec(deprecated)
#endif

namespace hbm_img_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HbmMsg1080P_
{
  using Type = HbmMsg1080P_<ContainerAllocator>;

  explicit HbmMsg1080P_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->height = 0ul;
      this->width = 0ul;
      this->data_size = 0ul;
      this->step = 0ul;
      std::fill<typename std::array<uint8_t, 12>::iterator, uint8_t>(this->encoding.begin(), this->encoding.end(), 0);
      std::fill<typename std::array<uint8_t, 6220800>::iterator, uint8_t>(this->data.begin(), this->data.end(), 0);
    }
  }

  explicit HbmMsg1080P_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_stamp(_alloc, _init),
    encoding(_alloc),
    data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0l;
      this->height = 0ul;
      this->width = 0ul;
      this->data_size = 0ul;
      this->step = 0ul;
      std::fill<typename std::array<uint8_t, 12>::iterator, uint8_t>(this->encoding.begin(), this->encoding.end(), 0);
      std::fill<typename std::array<uint8_t, 6220800>::iterator, uint8_t>(this->data.begin(), this->data.end(), 0);
    }
  }

  // field types and members
  using _index_type =
    int32_t;
  _index_type index;
  using _time_stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _time_stamp_type time_stamp;
  using _height_type =
    uint32_t;
  _height_type height;
  using _width_type =
    uint32_t;
  _width_type width;
  using _data_size_type =
    uint32_t;
  _data_size_type data_size;
  using _step_type =
    uint32_t;
  _step_type step;
  using _encoding_type =
    std::array<uint8_t, 12>;
  _encoding_type encoding;
  using _data_type =
    std::array<uint8_t, 6220800>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__index(
    const int32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__time_stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->time_stamp = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__data_size(
    const uint32_t & _arg)
  {
    this->data_size = _arg;
    return *this;
  }
  Type & set__step(
    const uint32_t & _arg)
  {
    this->step = _arg;
    return *this;
  }
  Type & set__encoding(
    const std::array<uint8_t, 12> & _arg)
  {
    this->encoding = _arg;
    return *this;
  }
  Type & set__data(
    const std::array<uint8_t, 6220800> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t MAX_SIZE =
    6220800u;

  // pointer types
  using RawPtr =
    hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator> *;
  using ConstRawPtr =
    const hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hbm_img_msgs__msg__HbmMsg1080P
    std::shared_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hbm_img_msgs__msg__HbmMsg1080P
    std::shared_ptr<hbm_img_msgs::msg::HbmMsg1080P_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HbmMsg1080P_ & other) const
  {
    if (this->index != other.index) {
      return false;
    }
    if (this->time_stamp != other.time_stamp) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->data_size != other.data_size) {
      return false;
    }
    if (this->step != other.step) {
      return false;
    }
    if (this->encoding != other.encoding) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const HbmMsg1080P_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HbmMsg1080P_

// alias to use template instance with default allocator
using HbmMsg1080P =
  hbm_img_msgs::msg::HbmMsg1080P_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t HbmMsg1080P_<ContainerAllocator>::MAX_SIZE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace hbm_img_msgs

#endif  // HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__STRUCT_HPP_
