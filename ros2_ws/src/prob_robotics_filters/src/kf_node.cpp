/**
 * Kalman Filter Node (linear)
 *
 * Zustand:  x = [px, py, vx, vy]
 * Messung:  z = [px, py]  aus /odom
 * Modell:   konstantes Geschwindigkeitsmodell (linear)
 *
 * WRONG-INIT: init_x, init_y, init_cov_x, init_cov_y
 */

#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "prob_robotics_filters/utils.hpp"

// ---------------------------------------------------------------------------
// Minimal 4x4 / 2x4 / 2x2 matrix helpers (Eigen-free)
// ---------------------------------------------------------------------------
using Mat4 = std::array<std::array<double, 4>, 4>;
using Mat2 = std::array<std::array<double, 2>, 2>;
using Mat24 = std::array<std::array<double, 4>, 2>;   // 2 rows, 4 cols
using Mat42 = std::array<std::array<double, 2>, 4>;   // 4 rows, 2 cols
using Vec4 = std::array<double, 4>;
using Vec2 = std::array<double, 2>;

static Mat4 mat4_eye()
{
  Mat4 m{}; m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0; return m;
}
static Mat4 mat4_mul(const Mat4 & A, const Mat4 & B)
{
  Mat4 C{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      for (int k = 0; k < 4; ++k)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}
static Vec4 mat4_vec4(const Mat4 & A, const Vec4 & v)
{
  Vec4 r{};
  for (int i = 0; i < 4; ++i)
    for (int k = 0; k < 4; ++k)
      r[i] += A[i][k] * v[k];
  return r;
}
// H * P  (2x4 * 4x4 -> 2x4)
static Mat24 mat24_mul_mat44(const Mat24 & H, const Mat4 & P)
{
  Mat24 R{};
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 4; ++j)
      for (int k = 0; k < 4; ++k)
        R[i][j] += H[i][k] * P[k][j];
  return R;
}
// (HP) * H^T  (2x4 * 4x2 -> 2x2), here H^T has shape 4x2
static Mat2 mat24_mul_H_T(const Mat24 & HP, const Mat24 & H)
{
  Mat2 S{};
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 4; ++k)
        S[i][j] += HP[i][k] * H[j][k];   // H[j][k] == H^T[k][j]
  return S;
}
// 2x2 inverse
static Mat2 mat2_inv(const Mat2 & M)
{
  double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
  Mat2 inv{};
  inv[0][0] =  M[1][1] / det;
  inv[0][1] = -M[0][1] / det;
  inv[1][0] = -M[1][0] / det;
  inv[1][1] =  M[0][0] / det;
  return inv;
}
// P * H^T  (4x4 * 4x2 -> 4x2)
static Mat42 mat44_mul_H_T(const Mat4 & P, const Mat24 & H)
{
  Mat42 R{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 4; ++k)
        R[i][j] += P[i][k] * H[j][k];
  return R;
}
// K (4x2) * S^-1 (2x2) -> K_gain (4x2) ... already done in one pass
// Actually K = P H^T S^-1, but we compute step by step.
// K * innov  (4x2 * 2 -> 4)
static Vec4 mat42_vec2(const Mat42 & K, const Vec2 & v)
{
  Vec4 r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 2; ++j)
      r[i] += K[i][j] * v[j];
  return r;
}
// K (4x2) * H (2x4) -> 4x4
static Mat4 mat42_mul_mat24(const Mat42 & K, const Mat24 & H)
{
  Mat4 R{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      for (int k = 0; k < 2; ++k)
        R[i][j] += K[i][k] * H[k][j];
  return R;
}
// (I - KH) * P
static Mat4 mat4_sub_mul(const Mat4 & A, const Mat4 & B, const Mat4 & P)
{
  // C = A - B,  then C * P
  Mat4 diff{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      diff[i][j] = A[i][j] - B[i][j];
  return mat4_mul(diff, P);
}
// 4x2  *  2x2 -> 4x2
static Mat42 mat42_mul_mat22(const Mat42 & A, const Mat2 & B)
{
  Mat42 R{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k)
        R[i][j] += A[i][k] * B[k][j];
  return R;
}

// ---------------------------------------------------------------------------
class KalmanFilterNode : public rclcpp::Node
{
public:
  KalmanFilterNode()
  : Node("kf_node"), last_time_(-1.0),
    cb_group_(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  {
    // --- Parameters ---------------------------------------------------------
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_cov_x", 0.5);
    declare_parameter("init_cov_y", 0.5);
    declare_parameter("process_noise_q", 0.01);
    declare_parameter("measurement_noise_r", 0.05);
    declare_parameter("odom_topic", std::string("/odom"));
    declare_parameter("output_topic", std::string("/kf/pose"));
    declare_parameter("frame_id", std::string("odom"));

    double init_x  = get_parameter("init_x").as_double();
    double init_y  = get_parameter("init_y").as_double();
    double cov_x   = get_parameter("init_cov_x").as_double();
    double cov_y   = get_parameter("init_cov_y").as_double();
    q_ = get_parameter("process_noise_q").as_double();
    r_ = get_parameter("measurement_noise_r").as_double();
    auto odom_t = get_parameter("odom_topic").as_string();
    auto out_t  = get_parameter("output_topic").as_string();
    frame_id_   = get_parameter("frame_id").as_string();

    // --- State --------------------------------------------------------------
    x_ = {init_x, init_y, 0.0, 0.0};

    P_ = mat4_eye();
    P_[0][0] = cov_x;
    P_[1][1] = cov_y;
    P_[2][2] = 1.0;
    P_[3][3] = 1.0;

    // H: 2x4
    H_ = {};
    H_[0][0] = 1.0;
    H_[1][1] = 1.0;

    // R: 2x2
    R_ = {};
    R_[0][0] = r_;
    R_[1][1] = r_;

    // --- ROS ----------------------------------------------------------------
    rclcpp::SubscriptionOptions opts_kf;
    opts_kf.callback_group = cb_group_;
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_t, 10,
      std::bind(&KalmanFilterNode::odom_cb, this, std::placeholders::_1), opts_kf);

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      out_t, 10);

    RCLCPP_INFO(get_logger(),
      "KF gestartet | init=(%.2f,%.2f) init_cov=(%.3f,%.3f) Q=%.4f R=%.4f",
      init_x, init_y, cov_x, cov_y, q_, r_);
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double now = get_clock()->now().nanoseconds() * 1e-9;

    if (last_time_ < 0.0) {
      last_time_ = now;
      return;
    }

    double dt = now - last_time_;
    last_time_ = now;

    if (dt <= 0.0 || dt > 1.0) return;

    // ---- 1) PREDICT -------------------------------------------------------
    Mat4 F = mat4_eye();
    F[0][2] = dt;
    F[1][3] = dt;

    Mat4 Q = mat4_eye();
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        Q[i][j] = (i == j) ? q_ : 0.0;

    x_ = mat4_vec4(F, x_);
    P_ = mat4_mul(mat4_mul(F, P_), transpose4(F));
    // P = F P F^T + Q
    for (int i = 0; i < 4; ++i) P_[i][i] += q_;

    // ---- 2) UPDATE --------------------------------------------------------
    Vec2 z = {msg->pose.pose.position.x, msg->pose.pose.position.y};

    // innov = z - H*x
    Vec2 Hx = {H_[0][0]*x_[0] + H_[0][1]*x_[1] + H_[0][2]*x_[2] + H_[0][3]*x_[3],
               H_[1][0]*x_[0] + H_[1][1]*x_[1] + H_[1][2]*x_[2] + H_[1][3]*x_[3]};
    Vec2 innov = {z[0] - Hx[0], z[1] - Hx[1]};

    Mat24 HP  = mat24_mul_mat44(H_, P_);
    Mat2  S   = mat24_mul_H_T(HP, H_);
    S[0][0] += r_;  S[1][1] += r_;        // S += R
    Mat2  Si  = mat2_inv(S);

    Mat42 PHt  = mat44_mul_H_T(P_, H_);
    Mat42 K    = mat42_mul_mat22(PHt, Si); // K = P H^T S^-1

    Vec4 Kinn = mat42_vec2(K, innov);
    for (int i = 0; i < 4; ++i) x_[i] += Kinn[i];

    Mat4 KH = mat42_mul_mat24(K, H_);
    P_ = mat4_sub_mul(mat4_eye(), KH, P_); // (I - KH) P

    // ---- 3) PUBLISH -------------------------------------------------------
    double yaw = prob_robotics_filters::quaternion_to_yaw(
      msg->pose.pose.orientation);

    pub_->publish(prob_robotics_filters::make_pose_msg(
      prob_robotics_filters::to_builtin_time(get_clock()->now()), frame_id_,
      x_[0], x_[1], yaw,
      P_[0][0], P_[1][1], 0.0));
  }

  // Transpose 4x4
  static Mat4 transpose4(const Mat4 & A)
  {
    Mat4 T{};
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        T[i][j] = A[j][i];
    return T;
  }

  // Members
  Vec4   x_;
  Mat4   P_;
  Mat24  H_;
  Mat2   R_;
  double q_, r_;
  double last_time_;
  std::string frame_id_;

  // Runtime measurement
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KalmanFilterNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
