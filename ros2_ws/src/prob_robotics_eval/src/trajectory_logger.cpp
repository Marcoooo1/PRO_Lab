/**
 * Trajectory Logger Node (C++)
 *
 * Abonniert:
 *   /odom          (Ground Truth + rohe Odometrie)
 *   /kf/pose       (Kalman Filter)
 *   /ekf/pose      (Extended Kalman Filter)
 *   /pf/pose       (Particle Filter)
 *
 * Schreibt synchronisiert ein CSV mit allen Posen pro Zeile.
 * Spalten:
 *   t, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw,
 *   kf_x, kf_y, kf_yaw, kf_cov_x, kf_cov_y,
 *   ekf_x, ekf_y, ekf_yaw, ekf_cov_x, ekf_cov_y, ekf_cov_yaw,
 *   pf_x, pf_y, pf_yaw, pf_cov_x, pf_cov_y, pf_cov_yaw
 */

#include <cmath>
#include <fstream>
#include <string>
#include <array>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// NaN shorthand
static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

struct Pose3 { double x{NaN}, y{NaN}, yaw{NaN}; };
struct PoseCov { double x{NaN}, y{NaN}, yaw{NaN},
                        cov_x{NaN}, cov_y{NaN}, cov_yaw{NaN}; };

static double yaw_from_quat(
  double qx, double qy, double qz, double qw)
{
  double siny = 2.0 * (qw * qz + qx * qy);
  double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny, cosy);
}

class TrajectoryLogger : public rclcpp::Node
{
public:
  TrajectoryLogger()
  : Node("trajectory_logger")
  {
    declare_parameter("output_csv", std::string("/tmp/trajectory_log.csv"));
    declare_parameter("rate_hz",    20.0);

    auto path = get_parameter("output_csv").as_string();
    double rate = get_parameter("rate_hz").as_double();

    // Open CSV
    file_.open(path);
    if (!file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Kann CSV nicht oeffnen: %s", path.c_str());
      throw std::runtime_error("CSV open failed");
    }
    file_ << "t,"
          << "gt_x,gt_y,gt_yaw,"
          << "odom_x,odom_y,odom_yaw,"
          << "kf_x,kf_y,kf_yaw,kf_cov_x,kf_cov_y,"
          << "ekf_x,ekf_y,ekf_yaw,ekf_cov_x,ekf_cov_y,ekf_cov_yaw,"
          << "pf_x,pf_y,pf_yaw,pf_cov_x,pf_cov_y,pf_cov_yaw\n";
    file_.flush();

    // Subscriptions
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        auto & p = m->pose.pose;
        double yaw = yaw_from_quat(
          p.orientation.x, p.orientation.y,
          p.orientation.z, p.orientation.w);
        gt_   = {p.position.x, p.position.y, yaw};
        odom_ = {p.position.x, p.position.y, yaw};
      });

    sub_kf_ = create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/kf/pose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        auto & p = m->pose.pose;
        kf_ = {p.position.x, p.position.y,
               yaw_from_quat(p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w),
               m->pose.covariance[0],
               m->pose.covariance[7],
               NaN};
      });

    sub_ekf_ = create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ekf/pose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        auto & p = m->pose.pose;
        ekf_ = {p.position.x, p.position.y,
                yaw_from_quat(p.orientation.x, p.orientation.y,
                              p.orientation.z, p.orientation.w),
                m->pose.covariance[0],
                m->pose.covariance[7],
                m->pose.covariance[35]};
      });

    sub_pf_ = create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pf/pose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        auto & p = m->pose.pose;
        pf_ = {p.position.x, p.position.y,
               yaw_from_quat(p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w),
               m->pose.covariance[0],
               m->pose.covariance[7],
               m->pose.covariance[35]};
      });

    t0_ = get_clock()->now().nanoseconds() * 1e-9;

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&TrajectoryLogger::tick, this));

    RCLCPP_INFO(get_logger(),
      "Logger schreibt nach %s @ %.1f Hz", path.c_str(), rate);
  }

  ~TrajectoryLogger()
  {
    if (file_.is_open()) file_.close();
  }

private:
  void tick()
  {
    double t = get_clock()->now().nanoseconds() * 1e-9 - t0_;

    auto f = [](double v) -> std::string {
      if (std::isnan(v)) return "nan";
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.5f", v);
      return buf;
    };

    file_
      << f(t)            << ","
      << f(gt_.x)        << "," << f(gt_.y)   << "," << f(gt_.yaw)  << ","
      << f(odom_.x)      << "," << f(odom_.y) << "," << f(odom_.yaw)<< ","
      << f(kf_.x)        << "," << f(kf_.y)   << "," << f(kf_.yaw)
      << "," << f(kf_.cov_x) << "," << f(kf_.cov_y) << ","
      << f(ekf_.x)       << "," << f(ekf_.y)  << "," << f(ekf_.yaw)
      << "," << f(ekf_.cov_x) << "," << f(ekf_.cov_y) << "," << f(ekf_.cov_yaw) << ","
      << f(pf_.x)        << "," << f(pf_.y)   << "," << f(pf_.yaw)
      << "," << f(pf_.cov_x) << "," << f(pf_.cov_y) << "," << f(pf_.cov_yaw)
      << "\n";
    file_.flush();
  }

  std::ofstream file_;
  double t0_{0.0};

  Pose3   gt_, odom_;
  PoseCov kf_, ekf_, pf_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_kf_, sub_ekf_, sub_pf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryLogger>());
  rclcpp::shutdown();
  return 0;
}
