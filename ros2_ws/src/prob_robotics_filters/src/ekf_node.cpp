/**
 * Extended Kalman Filter Node
 *
 * Zustand:  x = [px, py, theta]
 * Control:  u = [v, omega]  aus /cmd_vel (TwistStamped)
 * Messung:  z = [px, py, theta]  aus /odom
 *
 * Kein message_filters - cmd_vel wird als letzter bekannter Wert gespeichert,
 * Update wird bei jedem /odom-Message ausgeloest (wie PF).
 *
 * FIX: Fallback auf Odom-Twist hinzugefuegt, falls cmd_vel (noch) keine
 * Daten liefert (z.B. Topic-Namens-Tippfehler in einer Config) -- analog
 * zum PF, damit der Filter nicht stillschweigend mit v=0,w=0 predicted.
 *
 * WRONG-INIT: init_x, init_y, init_yaw, init_cov_x, init_cov_y, init_cov_yaw
 */

#include <cmath>
#include <string>
#include <array>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "prob_robotics_filters/utils.hpp"

// ---------------------------------------------------------------------------
// Minimal 3x3 matrix helpers
// ---------------------------------------------------------------------------
using Mat3 = std::array<std::array<double, 3>, 3>;
using Vec3 = std::array<double, 3>;

static Mat3 mat3_eye()
{
  Mat3 m{}; m[0][0] = m[1][1] = m[2][2] = 1.0; return m;
}
static Mat3 mat3_mul(const Mat3 & A, const Mat3 & B)
{
  Mat3 C{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}
static Mat3 mat3_transpose(const Mat3 & A)
{
  Mat3 T{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      T[i][j] = A[j][i];
  return T;
}
static Vec3 mat3_vec3(const Mat3 & A, const Vec3 & v)
{
  Vec3 r{};
  for (int i = 0; i < 3; ++i)
    for (int k = 0; k < 3; ++k)
      r[i] += A[i][k] * v[k];
  return r;
}
static Mat3 mat3_inv(const Mat3 & M)
{
  double det =
    M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
   -M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
   +M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
  Mat3 inv{};
  inv[0][0] = (M[1][1]*M[2][2]-M[1][2]*M[2][1])/det;
  inv[0][1] =-(M[0][1]*M[2][2]-M[0][2]*M[2][1])/det;
  inv[0][2] = (M[0][1]*M[1][2]-M[0][2]*M[1][1])/det;
  inv[1][0] =-(M[1][0]*M[2][2]-M[1][2]*M[2][0])/det;
  inv[1][1] = (M[0][0]*M[2][2]-M[0][2]*M[2][0])/det;
  inv[1][2] =-(M[0][0]*M[1][2]-M[0][2]*M[1][0])/det;
  inv[2][0] = (M[1][0]*M[2][1]-M[1][1]*M[2][0])/det;
  inv[2][1] =-(M[0][0]*M[2][1]-M[0][1]*M[2][0])/det;
  inv[2][2] = (M[0][0]*M[1][1]-M[0][1]*M[1][0])/det;
  return inv;
}
static Mat3 mat3_add(const Mat3 & A, const Mat3 & B)
{
  Mat3 R{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R[i][j] = A[i][j] + B[i][j];
  return R;
}
static Mat3 mat3_sub(const Mat3 & A, const Mat3 & B)
{
  Mat3 R{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R[i][j] = A[i][j] - B[i][j];
  return R;
}

// ---------------------------------------------------------------------------
class EKFNode : public rclcpp::Node
{
public:
  EKFNode()
  : Node("ekf_node"), last_time_(-1.0), last_v_(0.0), last_w_(0.0),
    have_cmd_(false),
    cb_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    // --- Parameters ---------------------------------------------------------
    declare_parameter("init_x",   0.0);
    declare_parameter("init_y",   0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_cov_x",   0.5);
    declare_parameter("init_cov_y",   0.5);
    declare_parameter("init_cov_yaw", 0.1);
    declare_parameter("process_noise_q",    0.01);
    declare_parameter("measurement_noise_r", 0.05);
    declare_parameter("odom_topic",   std::string("/odom"));
    declare_parameter("cmd_topic",    std::string("/cmd_vel"));
    declare_parameter("output_topic", std::string("/ekf/pose"));
    declare_parameter("frame_id",     std::string("odom"));

    double init_x   = get_parameter("init_x").as_double();
    double init_y   = get_parameter("init_y").as_double();
    double init_yaw = get_parameter("init_yaw").as_double();
    double cov_x    = get_parameter("init_cov_x").as_double();
    double cov_y    = get_parameter("init_cov_y").as_double();
    double cov_yaw  = get_parameter("init_cov_yaw").as_double();
    q_ = get_parameter("process_noise_q").as_double();
    r_ = get_parameter("measurement_noise_r").as_double();
    auto odom_t = get_parameter("odom_topic").as_string();
    auto cmd_t  = get_parameter("cmd_topic").as_string();
    auto out_t  = get_parameter("output_topic").as_string();
    frame_id_   = get_parameter("frame_id").as_string();

    // --- State --------------------------------------------------------------
    x_ = {init_x, init_y, init_yaw};
    P_ = mat3_eye();
    P_[0][0] = cov_x; P_[1][1] = cov_y; P_[2][2] = cov_yaw;
    for (auto & row : Q_) row.fill(0.0);
    Q_[0][0] = q_; Q_[1][1] = q_; Q_[2][2] = q_;
    for (auto & row : R_) row.fill(0.0);
    R_[0][0] = r_; R_[1][1] = r_; R_[2][2] = r_;

    // --- Subscriptions ------------------------------------------------------
    rclcpp::SubscriptionOptions opts_ekf;
    opts_ekf.callback_group = cb_group_;
    sub_cmd_stamped_ = create_subscription<
      geometry_msgs::msg::TwistStamped>(
      cmd_t, 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr m) {
        last_v_ = m->twist.linear.x;
        last_w_ = m->twist.angular.z;
        have_cmd_ = true;
      }, opts_ekf);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_t, 10,
      std::bind(&EKFNode::odom_cb, this, std::placeholders::_1), opts_ekf);

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      out_t, 10);

    RCLCPP_INFO(get_logger(),
      "EKF gestartet | init=(%.2f,%.2f,%.2f) Q=%.4f R=%.4f cmd_topic=%s",
      init_x, init_y, init_yaw, q_, r_, cmd_t.c_str());
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    double now = get_clock()->now().nanoseconds() * 1e-9;

    if (last_time_ < 0.0) {
      last_time_ = now;
      return;
    }
    double dt = now - last_time_;
    last_time_ = now;
    if (dt <= 0.0 || dt > 1.0) return;

    // FIX: Fallback auf Odom-Twist, falls cmd_vel (noch) nicht empfangen wurde
    double v = have_cmd_ ? last_v_ : odom_msg->twist.twist.linear.x;
    double w = have_cmd_ ? last_w_ : odom_msg->twist.twist.angular.z;
    double theta = x_[2];

    // ---- 1) PREDICT -------------------------------------------------------
    x_[0] += v * dt * std::cos(theta);
    x_[1] += v * dt * std::sin(theta);
    x_[2] += w * dt;
    x_[2]  = std::atan2(std::sin(x_[2]), std::cos(x_[2]));

    Mat3 F = mat3_eye();
    F[0][2] = -v * dt * std::sin(theta);
    F[1][2] =  v * dt * std::cos(theta);

    P_ = mat3_add(mat3_mul(mat3_mul(F, P_), mat3_transpose(F)), Q_);

    // ---- 2) UPDATE --------------------------------------------------------
    Vec3 z = {
      odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y,
      prob_robotics_filters::quaternion_to_yaw(odom_msg->pose.pose.orientation)
    };

    Vec3 innov = {
      z[0] - x_[0],
      z[1] - x_[1],
      prob_robotics_filters::angle_diff(z[2], x_[2])
    };

    Mat3 S  = mat3_add(P_, R_);
    Mat3 K  = mat3_mul(P_, mat3_inv(S));

    Vec3 Kinn = mat3_vec3(K, innov);
    for (int i = 0; i < 3; ++i) x_[i] += Kinn[i];
    x_[2] = std::atan2(std::sin(x_[2]), std::cos(x_[2]));

    P_ = mat3_mul(mat3_sub(mat3_eye(), K), P_);

    // ---- 3) PUBLISH -------------------------------------------------------
    pub_->publish(prob_robotics_filters::make_pose_msg(
      prob_robotics_filters::to_builtin_time(get_clock()->now()), frame_id_,
      x_[0], x_[1], x_[2],
      P_[0][0], P_[1][1], P_[2][2]));
  }

  Vec3 x_;
  Mat3 P_, Q_, R_;
  double q_, r_;
  double last_time_, last_v_, last_w_;
  bool have_cmd_;
  std::string frame_id_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_stamped_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EKFNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
