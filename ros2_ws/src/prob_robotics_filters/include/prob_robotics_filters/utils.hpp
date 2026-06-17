#pragma once

#include <cmath>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace prob_robotics_filters
{

/// Extrahiert Yaw aus Quaternion
inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/// Yaw -> Quaternion (nur Z-Rotation)
inline geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.z = std::sin(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  return q;
}

/// Winkeldifferenz auf [-pi, pi]
inline double angle_diff(double a, double b)
{
  double d = a - b;
  while (d >  M_PI) d -= 2.0 * M_PI;
  while (d < -M_PI) d += 2.0 * M_PI;
  return d;
}

/// Baut PoseWithCovarianceStamped
inline geometry_msgs::msg::PoseWithCovarianceStamped make_pose_msg(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  double x, double y, double yaw,
  double cov_xx = 0.0, double cov_yy = 0.0, double cov_yawyaw = 0.0)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp    = stamp;
  msg.header.frame_id = frame_id;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = yaw_to_quaternion(yaw);
  // 6x6 covariance, row-major; indices 0=xx, 7=yy, 35=yaw*yaw
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0]  = cov_xx;
  msg.pose.covariance[7]  = cov_yy;
  msg.pose.covariance[35] = cov_yawyaw;
  return msg;
}

/// Konvertiert rclcpp::Time -> builtin_interfaces::msg::Time (Jazzy-kompatibel)
inline builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & t)
{
  builtin_interfaces::msg::Time msg;
  msg.sec     = static_cast<int32_t>(t.nanoseconds() / 1000000000LL);
  msg.nanosec = static_cast<uint32_t>(t.nanoseconds() % 1000000000LL);
  return msg;
}

}  // namespace prob_robotics_filters
