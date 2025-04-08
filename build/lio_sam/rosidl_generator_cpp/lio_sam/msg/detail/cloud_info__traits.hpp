// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lio_sam:msg/CloudInfo.idl
// generated code does not contain a copyright notice

#ifndef LIO_SAM__MSG__DETAIL__CLOUD_INFO__TRAITS_HPP_
#define LIO_SAM__MSG__DETAIL__CLOUD_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lio_sam/msg/detail/cloud_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'cloud_deskewed'
// Member 'cloud_corner'
// Member 'cloud_surface'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace lio_sam
{

namespace msg
{

inline void to_flow_style_yaml(
  const CloudInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: start_ring_index
  {
    if (msg.start_ring_index.size() == 0) {
      out << "start_ring_index: []";
    } else {
      out << "start_ring_index: [";
      size_t pending_items = msg.start_ring_index.size();
      for (auto item : msg.start_ring_index) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: end_ring_index
  {
    if (msg.end_ring_index.size() == 0) {
      out << "end_ring_index: []";
    } else {
      out << "end_ring_index: [";
      size_t pending_items = msg.end_ring_index.size();
      for (auto item : msg.end_ring_index) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: point_col_ind
  {
    if (msg.point_col_ind.size() == 0) {
      out << "point_col_ind: []";
    } else {
      out << "point_col_ind: [";
      size_t pending_items = msg.point_col_ind.size();
      for (auto item : msg.point_col_ind) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: point_range
  {
    if (msg.point_range.size() == 0) {
      out << "point_range: []";
    } else {
      out << "point_range: [";
      size_t pending_items = msg.point_range.size();
      for (auto item : msg.point_range) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: imu_available
  {
    out << "imu_available: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_available, out);
    out << ", ";
  }

  // member: odom_available
  {
    out << "odom_available: ";
    rosidl_generator_traits::value_to_yaml(msg.odom_available, out);
    out << ", ";
  }

  // member: imu_roll_init
  {
    out << "imu_roll_init: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_roll_init, out);
    out << ", ";
  }

  // member: imu_pitch_init
  {
    out << "imu_pitch_init: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_pitch_init, out);
    out << ", ";
  }

  // member: imu_yaw_init
  {
    out << "imu_yaw_init: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_yaw_init, out);
    out << ", ";
  }

  // member: initial_guess_x
  {
    out << "initial_guess_x: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_x, out);
    out << ", ";
  }

  // member: initial_guess_y
  {
    out << "initial_guess_y: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_y, out);
    out << ", ";
  }

  // member: initial_guess_z
  {
    out << "initial_guess_z: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_z, out);
    out << ", ";
  }

  // member: initial_guess_roll
  {
    out << "initial_guess_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_roll, out);
    out << ", ";
  }

  // member: initial_guess_pitch
  {
    out << "initial_guess_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_pitch, out);
    out << ", ";
  }

  // member: initial_guess_yaw
  {
    out << "initial_guess_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_yaw, out);
    out << ", ";
  }

  // member: cloud_deskewed
  {
    out << "cloud_deskewed: ";
    to_flow_style_yaml(msg.cloud_deskewed, out);
    out << ", ";
  }

  // member: cloud_corner
  {
    out << "cloud_corner: ";
    to_flow_style_yaml(msg.cloud_corner, out);
    out << ", ";
  }

  // member: cloud_surface
  {
    out << "cloud_surface: ";
    to_flow_style_yaml(msg.cloud_surface, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CloudInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: start_ring_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.start_ring_index.size() == 0) {
      out << "start_ring_index: []\n";
    } else {
      out << "start_ring_index:\n";
      for (auto item : msg.start_ring_index) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: end_ring_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.end_ring_index.size() == 0) {
      out << "end_ring_index: []\n";
    } else {
      out << "end_ring_index:\n";
      for (auto item : msg.end_ring_index) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: point_col_ind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.point_col_ind.size() == 0) {
      out << "point_col_ind: []\n";
    } else {
      out << "point_col_ind:\n";
      for (auto item : msg.point_col_ind) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: point_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.point_range.size() == 0) {
      out << "point_range: []\n";
    } else {
      out << "point_range:\n";
      for (auto item : msg.point_range) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: imu_available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_available: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_available, out);
    out << "\n";
  }

  // member: odom_available
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odom_available: ";
    rosidl_generator_traits::value_to_yaml(msg.odom_available, out);
    out << "\n";
  }

  // member: imu_roll_init
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_roll_init: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_roll_init, out);
    out << "\n";
  }

  // member: imu_pitch_init
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_pitch_init: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_pitch_init, out);
    out << "\n";
  }

  // member: imu_yaw_init
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_yaw_init: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_yaw_init, out);
    out << "\n";
  }

  // member: initial_guess_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_guess_x: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_x, out);
    out << "\n";
  }

  // member: initial_guess_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_guess_y: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_y, out);
    out << "\n";
  }

  // member: initial_guess_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_guess_z: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_z, out);
    out << "\n";
  }

  // member: initial_guess_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_guess_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_roll, out);
    out << "\n";
  }

  // member: initial_guess_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_guess_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_pitch, out);
    out << "\n";
  }

  // member: initial_guess_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_guess_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_guess_yaw, out);
    out << "\n";
  }

  // member: cloud_deskewed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud_deskewed:\n";
    to_block_style_yaml(msg.cloud_deskewed, out, indentation + 2);
  }

  // member: cloud_corner
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud_corner:\n";
    to_block_style_yaml(msg.cloud_corner, out, indentation + 2);
  }

  // member: cloud_surface
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud_surface:\n";
    to_block_style_yaml(msg.cloud_surface, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CloudInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace lio_sam

namespace rosidl_generator_traits
{

[[deprecated("use lio_sam::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lio_sam::msg::CloudInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  lio_sam::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lio_sam::msg::to_yaml() instead")]]
inline std::string to_yaml(const lio_sam::msg::CloudInfo & msg)
{
  return lio_sam::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lio_sam::msg::CloudInfo>()
{
  return "lio_sam::msg::CloudInfo";
}

template<>
inline const char * name<lio_sam::msg::CloudInfo>()
{
  return "lio_sam/msg/CloudInfo";
}

template<>
struct has_fixed_size<lio_sam::msg::CloudInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lio_sam::msg::CloudInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lio_sam::msg::CloudInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIO_SAM__MSG__DETAIL__CLOUD_INFO__TRAITS_HPP_
