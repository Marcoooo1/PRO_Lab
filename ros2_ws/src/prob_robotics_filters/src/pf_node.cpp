/**
 * Particle Filter Node
 *
 * Partikel:        N x [px, py, theta]
 * Bewegungsmodell: Differential Drive mit Prozess-Rauschen
 * Messmodell:      Odom-Pose als verrauschte Messung
 * Resampling:      systematisch, bei ESS < N/2
 *
 * WRONG-INIT: init_x, init_y, init_yaw, init_spread_x/y/yaw
 *
 * FIX (Jazzy/TurtleBot4): cmd_vel ist TwistStamped, nicht Twist!
 * Eine Subscription mit falschem Typ bekommt NIE Daten (stiller Fehler,
 * kein Crash) -- dadurch blieben last_v_/last_w_ dauerhaft 0.0 und der
 * PF driftete weg, waehrend KF/EKF (die odom-Twist als Fallback nutzen
 * bzw. korrekt TwistStamped abonnieren) das nicht betroffen waren.
 *
 * FIX 2: Der TurtleBot4-Controller ignoriert cmd_vel-Befehle voruebergehend
 * waehrend bestimmter interner Zustaende ("Ignoring velocities commanded
 * while an autonomous behavior is running!"). Der PF predicted in diesen
 * Momenten weiterhin mit dem zuletzt empfangenen (v,w), waehrend sich der
 * echte Roboter nicht bewegt -- Diskrepanz zwischen Annahme und Realitaet.
 * Im Gegensatz zu KF/EKF (volle Kalman-Korrektur jede Messung) verliert
 * der PF dabei strukturell die Spur, wenn sigma_yaw zu eng gekoppelt ist
 * (vorher: sigma_yaw = sigma_z_ * 0.5 = 0.1 rad, sehr eng). Fix: sigma_yaw
 * als eigener, lockerer Parameter (measurement_noise_yaw) + hoeheres
 * Prozessrauschen, damit die Partikelwolke solche Diskrepanzen toleriert
 * und sich nach Aussetzern wieder einfaengt statt dauerhaft abzudriften.
 */

#include <cmath>
#include <string>
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "prob_robotics_filters/utils.hpp"

class ParticleFilterNode : public rclcpp::Node
{
public:
  ParticleFilterNode()
  : Node("pf_node"), last_time_(-1.0), last_v_(0.0), last_w_(0.0),
    have_cmd_(false),
    cb_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    // --- Parameters ---------------------------------------------------------
    declare_parameter("init_x", 0.0);
    declare_parameter("init_y", 0.0);
    declare_parameter("init_yaw", 0.0);
    declare_parameter("init_spread_x", 0.5);
    declare_parameter("init_spread_y", 0.5);
    declare_parameter("init_spread_yaw", 0.1);
    declare_parameter("num_particles", 500);
    declare_parameter("process_noise_v", 0.3);
    declare_parameter("process_noise_w", 0.2);
    declare_parameter("measurement_noise_r", 0.3);
    declare_parameter("measurement_noise_yaw", 0.3);
    declare_parameter("odom_topic", std::string("/odom"));
    declare_parameter("cmd_topic", std::string("/cmd_vel"));
    declare_parameter("output_topic", std::string("/pf/pose"));
    declare_parameter("particles_topic", std::string("/pf/particles"));
    declare_parameter("frame_id", std::string("odom"));

    double init_x     = get_parameter("init_x").as_double();
    double init_y     = get_parameter("init_y").as_double();
    double init_yaw   = get_parameter("init_yaw").as_double();
    double spread_x   = get_parameter("init_spread_x").as_double();
    double spread_y   = get_parameter("init_spread_y").as_double();
    double spread_yaw = get_parameter("init_spread_yaw").as_double();
    N_        = get_parameter("num_particles").as_int();
    sigma_v_  = get_parameter("process_noise_v").as_double();
    sigma_w_  = get_parameter("process_noise_w").as_double();
    sigma_z_   = get_parameter("measurement_noise_r").as_double();
    sigma_yaw_ = get_parameter("measurement_noise_yaw").as_double();
    auto odom_t = get_parameter("odom_topic").as_string();
    auto cmd_t  = get_parameter("cmd_topic").as_string();
    auto out_t  = get_parameter("output_topic").as_string();
    auto part_t = get_parameter("particles_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    // --- RNG (fixed seed for reproducibility) -------------------------------
    rng_.seed(42);

    // --- Particles ----------------------------------------------------------
    particles_.resize(N_, {0.0, 0.0, 0.0});
    weights_.assign(N_, 1.0 / N_);

    std::normal_distribution<double> dx(init_x,   spread_x);
    std::normal_distribution<double> dy(init_y,   spread_y);
    std::normal_distribution<double> dyaw(init_yaw, spread_yaw);
    for (auto & p : particles_) {
      p[0] = dx(rng_);
      p[1] = dy(rng_);
      p[2] = dyaw(rng_);
    }

    // --- ROS ----------------------------------------------------------------
    rclcpp::SubscriptionOptions opts_pf;
    opts_pf.callback_group = cb_group_;
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_t, 10,
      std::bind(&ParticleFilterNode::odom_cb, this, std::placeholders::_1), opts_pf);

    // FIX: TwistStamped statt Twist (Jazzy/TurtleBot4 cmd_vel-Typ)
    sub_cmd_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_t, 10,
      std::bind(&ParticleFilterNode::cmd_cb, this, std::placeholders::_1), opts_pf);

    pub_pose_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(out_t, 10);
    pub_parts_ = create_publisher<geometry_msgs::msg::PoseArray>(part_t, 10);

    RCLCPP_INFO(get_logger(),
      "PF gestartet | N=%d init=(%.2f,%.2f,%.2f) spread=(%.2f,%.2f,%.2f) "
      "cmd_topic=%s sigma_v=%.2f sigma_w=%.2f sigma_z=%.2f sigma_yaw=%.2f",
      N_, init_x, init_y, init_yaw, spread_x, spread_y, spread_yaw,
      cmd_t.c_str(), sigma_v_, sigma_w_, sigma_z_, sigma_yaw_);
  }

private:
  using Particle = std::array<double, 3>;  // [x, y, theta]

  void cmd_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    last_v_ = msg->twist.linear.x;
    last_w_ = msg->twist.angular.z;
    have_cmd_ = true;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double now = get_clock()->now().nanoseconds() * 1e-9;
    if (last_time_ < 0.0) { last_time_ = now; return; }
    double dt = now - last_time_;
    last_time_ = now;
    if (dt <= 0.0 || dt > 1.0) return;

    // Steuerung: bevorzugt cmd_vel (falls je empfangen), sonst Odom-Twist
    double v = have_cmd_ ? last_v_ : msg->twist.twist.linear.x;
    double w = have_cmd_ ? last_w_ : msg->twist.twist.angular.z;

    std::normal_distribution<double> nv(0.0, sigma_v_);
    std::normal_distribution<double> nw(0.0, sigma_w_);

    // ---- 1) PREDICT --------------------------------------------------------
    for (auto & p : particles_) {
      double vn = v + nv(rng_);
      double wn = w + nw(rng_);
      p[0] += vn * dt * std::cos(p[2]);
      p[1] += vn * dt * std::sin(p[2]);
      p[2] += wn * dt;
      p[2]  = std::atan2(std::sin(p[2]), std::cos(p[2]));
    }

    // ---- 2) UPDATE: likelihood weights -------------------------------------
    double z_x   = msg->pose.pose.position.x;
    double z_y   = msg->pose.pose.position.y;
    double z_yaw = prob_robotics_filters::quaternion_to_yaw(
      msg->pose.pose.orientation);

    double inv2z2 = 1.0 / (2.0 * sigma_z_ * sigma_z_);
    double inv2y2 = 1.0 / (2.0 * sigma_yaw_ * sigma_yaw_);

    // Log-weights
    std::vector<double> log_w(N_);
    double log_max = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < N_; ++i) {
      double dx   = particles_[i][0] - z_x;
      double dy   = particles_[i][1] - z_y;
      double dyaw = std::atan2(
        std::sin(particles_[i][2] - z_yaw),
        std::cos(particles_[i][2] - z_yaw));
      log_w[i] = -(dx*dx + dy*dy) * inv2z2 - dyaw*dyaw * inv2y2;
      log_max  = std::max(log_max, log_w[i]);
    }

    double sum = 0.0;
    for (int i = 0; i < N_; ++i) {
      weights_[i] = std::exp(log_w[i] - log_max) * weights_[i];
      sum += weights_[i];
    }
    if (sum > 1e-12) {
      for (auto & w2 : weights_) w2 /= sum;
    } else {
      weights_.assign(N_, 1.0 / N_);
    }

    // ---- 3) RESAMPLE if ESS < N/2 ------------------------------------------
    double ess = 0.0;
    for (auto ww : weights_) ess += ww * ww;
    ess = 1.0 / ess;

    if (ess < N_ / 2.0) {
      particles_ = systematic_resample(particles_, weights_);
      weights_.assign(N_, 1.0 / N_);
    }

    // ---- 4) ESTIMATE (weighted mean) ---------------------------------------
    double mean_x = 0.0, mean_y = 0.0;
    double sin_sum = 0.0, cos_sum = 0.0;
    for (int i = 0; i < N_; ++i) {
      mean_x  += weights_[i] * particles_[i][0];
      mean_y  += weights_[i] * particles_[i][1];
      sin_sum += weights_[i] * std::sin(particles_[i][2]);
      cos_sum += weights_[i] * std::cos(particles_[i][2]);
    }
    double mean_yaw = std::atan2(sin_sum, cos_sum);

    double var_x = 0.0, var_y = 0.0, var_yaw = 0.0;
    for (int i = 0; i < N_; ++i) {
      double ex = particles_[i][0] - mean_x;
      double ey = particles_[i][1] - mean_y;
      double et = particles_[i][2] - mean_yaw;
      var_x   += weights_[i] * ex * ex;
      var_y   += weights_[i] * ey * ey;
      var_yaw += weights_[i] * et * et;
    }

    auto stamp = prob_robotics_filters::to_builtin_time(get_clock()->now());
    pub_pose_->publish(prob_robotics_filters::make_pose_msg(
      stamp, frame_id_, mean_x, mean_y, mean_yaw, var_x, var_y, var_yaw));

    // ---- PoseArray for RViz ------------------------------------------------
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp    = stamp;
    pa.header.frame_id = frame_id_;
    pa.poses.reserve(N_);
    for (const auto & p : particles_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = p[0];
      pose.position.y = p[1];
      pose.orientation = prob_robotics_filters::yaw_to_quaternion(p[2]);
      pa.poses.push_back(pose);
    }
    pub_parts_->publish(pa);
  }

  std::vector<Particle> systematic_resample(
    const std::vector<Particle> & parts,
    const std::vector<double> & w)
  {
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    double r = uni(rng_) / N_;
    std::vector<Particle> out;
    out.reserve(N_);
    double cumsum = w[0];
    int i = 0;
    for (int j = 0; j < N_; ++j) {
      double threshold = r + static_cast<double>(j) / N_;
      while (threshold > cumsum && i < N_ - 1) {
        ++i;
        cumsum += w[i];
      }
      out.push_back(parts[i]);
    }
    return out;
  }

  // Members
  int N_;
  double sigma_v_, sigma_w_, sigma_z_, sigma_yaw_;
  double last_time_, last_v_, last_w_;
  bool have_cmd_;
  std::string frame_id_;
  std::mt19937 rng_;

  std::vector<std::array<double, 3>> particles_;
  std::vector<double> weights_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_parts_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParticleFilterNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
