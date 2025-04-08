// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lio_sam:msg/CloudInfo.idl
// generated code does not contain a copyright notice

#ifndef LIO_SAM__MSG__DETAIL__CLOUD_INFO__STRUCT_HPP_
#define LIO_SAM__MSG__DETAIL__CLOUD_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'cloud_deskewed'
// Member 'cloud_corner'
// Member 'cloud_surface'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lio_sam__msg__CloudInfo __attribute__((deprecated))
#else
# define DEPRECATED__lio_sam__msg__CloudInfo __declspec(deprecated)
#endif

namespace lio_sam
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CloudInfo_
{
  using Type = CloudInfo_<ContainerAllocator>;

  explicit CloudInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    cloud_deskewed(_init),
    cloud_corner(_init),
    cloud_surface(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->imu_available = 0ll;
      this->odom_available = 0ll;
      this->imu_roll_init = 0.0f;
      this->imu_pitch_init = 0.0f;
      this->imu_yaw_init = 0.0f;
      this->initial_guess_x = 0.0f;
      this->initial_guess_y = 0.0f;
      this->initial_guess_z = 0.0f;
      this->initial_guess_roll = 0.0f;
      this->initial_guess_pitch = 0.0f;
      this->initial_guess_yaw = 0.0f;
    }
  }

  explicit CloudInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    cloud_deskewed(_alloc, _init),
    cloud_corner(_alloc, _init),
    cloud_surface(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->imu_available = 0ll;
      this->odom_available = 0ll;
      this->imu_roll_init = 0.0f;
      this->imu_pitch_init = 0.0f;
      this->imu_yaw_init = 0.0f;
      this->initial_guess_x = 0.0f;
      this->initial_guess_y = 0.0f;
      this->initial_guess_z = 0.0f;
      this->initial_guess_roll = 0.0f;
      this->initial_guess_pitch = 0.0f;
      this->initial_guess_yaw = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _start_ring_index_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _start_ring_index_type start_ring_index;
  using _end_ring_index_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _end_ring_index_type end_ring_index;
  using _point_col_ind_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _point_col_ind_type point_col_ind;
  using _point_range_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _point_range_type point_range;
  using _imu_available_type =
    int64_t;
  _imu_available_type imu_available;
  using _odom_available_type =
    int64_t;
  _odom_available_type odom_available;
  using _imu_roll_init_type =
    float;
  _imu_roll_init_type imu_roll_init;
  using _imu_pitch_init_type =
    float;
  _imu_pitch_init_type imu_pitch_init;
  using _imu_yaw_init_type =
    float;
  _imu_yaw_init_type imu_yaw_init;
  using _initial_guess_x_type =
    float;
  _initial_guess_x_type initial_guess_x;
  using _initial_guess_y_type =
    float;
  _initial_guess_y_type initial_guess_y;
  using _initial_guess_z_type =
    float;
  _initial_guess_z_type initial_guess_z;
  using _initial_guess_roll_type =
    float;
  _initial_guess_roll_type initial_guess_roll;
  using _initial_guess_pitch_type =
    float;
  _initial_guess_pitch_type initial_guess_pitch;
  using _initial_guess_yaw_type =
    float;
  _initial_guess_yaw_type initial_guess_yaw;
  using _cloud_deskewed_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _cloud_deskewed_type cloud_deskewed;
  using _cloud_corner_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _cloud_corner_type cloud_corner;
  using _cloud_surface_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _cloud_surface_type cloud_surface;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__start_ring_index(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->start_ring_index = _arg;
    return *this;
  }
  Type & set__end_ring_index(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->end_ring_index = _arg;
    return *this;
  }
  Type & set__point_col_ind(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->point_col_ind = _arg;
    return *this;
  }
  Type & set__point_range(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->point_range = _arg;
    return *this;
  }
  Type & set__imu_available(
    const int64_t & _arg)
  {
    this->imu_available = _arg;
    return *this;
  }
  Type & set__odom_available(
    const int64_t & _arg)
  {
    this->odom_available = _arg;
    return *this;
  }
  Type & set__imu_roll_init(
    const float & _arg)
  {
    this->imu_roll_init = _arg;
    return *this;
  }
  Type & set__imu_pitch_init(
    const float & _arg)
  {
    this->imu_pitch_init = _arg;
    return *this;
  }
  Type & set__imu_yaw_init(
    const float & _arg)
  {
    this->imu_yaw_init = _arg;
    return *this;
  }
  Type & set__initial_guess_x(
    const float & _arg)
  {
    this->initial_guess_x = _arg;
    return *this;
  }
  Type & set__initial_guess_y(
    const float & _arg)
  {
    this->initial_guess_y = _arg;
    return *this;
  }
  Type & set__initial_guess_z(
    const float & _arg)
  {
    this->initial_guess_z = _arg;
    return *this;
  }
  Type & set__initial_guess_roll(
    const float & _arg)
  {
    this->initial_guess_roll = _arg;
    return *this;
  }
  Type & set__initial_guess_pitch(
    const float & _arg)
  {
    this->initial_guess_pitch = _arg;
    return *this;
  }
  Type & set__initial_guess_yaw(
    const float & _arg)
  {
    this->initial_guess_yaw = _arg;
    return *this;
  }
  Type & set__cloud_deskewed(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->cloud_deskewed = _arg;
    return *this;
  }
  Type & set__cloud_corner(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->cloud_corner = _arg;
    return *this;
  }
  Type & set__cloud_surface(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->cloud_surface = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lio_sam::msg::CloudInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const lio_sam::msg::CloudInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lio_sam::msg::CloudInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lio_sam::msg::CloudInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lio_sam__msg__CloudInfo
    std::shared_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lio_sam__msg__CloudInfo
    std::shared_ptr<lio_sam::msg::CloudInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CloudInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->start_ring_index != other.start_ring_index) {
      return false;
    }
    if (this->end_ring_index != other.end_ring_index) {
      return false;
    }
    if (this->point_col_ind != other.point_col_ind) {
      return false;
    }
    if (this->point_range != other.point_range) {
      return false;
    }
    if (this->imu_available != other.imu_available) {
      return false;
    }
    if (this->odom_available != other.odom_available) {
      return false;
    }
    if (this->imu_roll_init != other.imu_roll_init) {
      return false;
    }
    if (this->imu_pitch_init != other.imu_pitch_init) {
      return false;
    }
    if (this->imu_yaw_init != other.imu_yaw_init) {
      return false;
    }
    if (this->initial_guess_x != other.initial_guess_x) {
      return false;
    }
    if (this->initial_guess_y != other.initial_guess_y) {
      return false;
    }
    if (this->initial_guess_z != other.initial_guess_z) {
      return false;
    }
    if (this->initial_guess_roll != other.initial_guess_roll) {
      return false;
    }
    if (this->initial_guess_pitch != other.initial_guess_pitch) {
      return false;
    }
    if (this->initial_guess_yaw != other.initial_guess_yaw) {
      return false;
    }
    if (this->cloud_deskewed != other.cloud_deskewed) {
      return false;
    }
    if (this->cloud_corner != other.cloud_corner) {
      return false;
    }
    if (this->cloud_surface != other.cloud_surface) {
      return false;
    }
    return true;
  }
  bool operator!=(const CloudInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CloudInfo_

// alias to use template instance with default allocator
using CloudInfo =
  lio_sam::msg::CloudInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lio_sam

#endif  // LIO_SAM__MSG__DETAIL__CLOUD_INFO__STRUCT_HPP_
