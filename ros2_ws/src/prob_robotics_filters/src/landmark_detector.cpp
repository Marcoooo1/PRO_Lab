/**
 * Landmark Detector Node
 *
 * Definiert ein festes Landmark im Raum.
 * Wenn der Roboter nah genug dran ist, wird eine "Detektion" gepublisht.
 *
 * Verwendung fuer Wrong-Init:
 *   - Zeigt wie weit die Filter vom echten Landmark entfernt schaetzen
 *   - KF/EKF/PF koennen Landmark-Messungen zur Korrektur nutzen
 *
 * Parameter:
 *   landmark_x, landmark_y  : Position des Landmarks im odom-Frame
 *   detection_range         : Erkennungsradius in Metern
 *   landmark_noise          : Messrauschen der Landmark-Erkennung
 *
 * Subscribed:  /odom  (Ground Truth fuer Simulation)
 * Published:   /landmark/detected  (PoseWithCovarianceStamped, wenn erkannt)
 *              /landmark/position  (PoseStamped, immer – fuer RViz2)
 */

#include <cmath>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "prob_robotics_filters/utils.hpp"

class LandmarkDetector : public rclcpp::Node
{
public:
  LandmarkDetector()
  : Node("landmark_detector"), rng_(42)
  {
    declare_parameter("landmark_x",      1.5);
    declare_parameter("landmark_y",      1.0);
    declare_parameter("detection_range", 1.5);
    declare_parameter("landmark_noise",  0.1);
    declare_parameter("frame_id",        std::string("odom"));

    lm_x_     = get_parameter("landmark_x").as_double();
    lm_y_     = get_parameter("landmark_y").as_double();
    range_    = get_parameter("detection_range").as_double();
    noise_    = get_parameter("landmark_noise").as_double();
    frame_id_ = get_parameter("frame_id").as_string();

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&LandmarkDetector::odom_cb, this, std::placeholders::_1));

    pub_detected_ = create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>("/landmark/detected", 10);

    pub_position_ = create_publisher<
      geometry_msgs::msg::PoseStamped>("/landmark/position", 10);

    pub_marker_ = create_publisher<
      visualization_msgs::msg::Marker>("/landmark/marker", 10);

    // Timer: Landmark-Position immer publishen (fuer RViz2)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LandmarkDetector::publish_landmark_position, this));

    RCLCPP_INFO(get_logger(),
      "LandmarkDetector gestartet | pos=(%.2f, %.2f) range=%.2fm noise=%.3f",
      lm_x_, lm_y_, range_, noise_);
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double rx = msg->pose.pose.position.x;
    double ry = msg->pose.pose.position.y;

    double dist = std::hypot(rx - lm_x_, ry - lm_y_);

    if (dist <= range_) {
      // Landmark erkannt – verrauschte Messung publishen
      std::normal_distribution<double> noise(0.0, noise_);

      double meas_x = lm_x_ + noise(rng_);
      double meas_y = lm_y_ + noise(rng_);

      auto stamp = get_clock()->now();
      builtin_interfaces::msg::Time t;
      auto ns = stamp.nanoseconds();
      t.sec     = static_cast<int32_t>(ns / 1000000000LL);
      t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

      geometry_msgs::msg::PoseWithCovarianceStamped det;
      det.header.stamp    = t;
      det.header.frame_id = frame_id_;
      det.pose.pose.position.x = meas_x;
      det.pose.pose.position.y = meas_y;
      det.pose.pose.position.z = 0.0;
      det.pose.pose.orientation.w = 1.0;
      det.pose.covariance.fill(0.0);
      det.pose.covariance[0]  = noise_ * noise_;
      det.pose.covariance[7]  = noise_ * noise_;
      pub_detected_->publish(det);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "Landmark erkannt! dist=%.3fm meas=(%.3f, %.3f)",
        dist, meas_x, meas_y);
    }
  }

  void publish_landmark_position()
  {
    auto ns = get_clock()->now().nanoseconds();
    builtin_interfaces::msg::Time t;
    t.sec     = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);

    // PoseStamped fuer andere Nodes
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = t;
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = lm_x_;
    ps.pose.position.y = lm_y_;
    ps.pose.orientation.w = 1.0;
    pub_position_->publish(ps);

    // Marker fuer RViz2
    visualization_msgs::msg::Marker marker;
    marker.header.stamp    = t;
    marker.header.frame_id = frame_id_;
    marker.ns   = "landmark";
    marker.id   = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = lm_x_;
    marker.pose.position.y = lm_y_;
    marker.pose.position.z = 0.25;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.5;
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9f;
    pub_marker_->publish(marker);
  }

  double lm_x_, lm_y_, range_, noise_;
  std::string frame_id_;
  std::mt19937 rng_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_detected_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_position_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkDetector>());
  rclcpp::shutdown();
  return 0;
}
