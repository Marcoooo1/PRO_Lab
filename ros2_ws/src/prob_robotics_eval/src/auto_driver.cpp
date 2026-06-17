/**
 * Auto Driver Node
 *
 * Faehrt den Roboter automatisch eine definierte Trajektorie ab.
 *
 * Parameter:
 *   pattern   : "circle" | "straight" | "eight" | "square"
 *   duration  : Gesamtdauer in Sekunden
 *   linear_v  : Vorwaertsgeschwindigkeit (m/s)
 *   angular_v : Winkelgeschwindigkeit (rad/s)
 *
 * Publiziert: /cmd_vel (geometry_msgs/TwistStamped)
 */

#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class AutoDriver : public rclcpp::Node
{
public:
  AutoDriver()
  : Node("auto_driver"), started_(false)
  {
    declare_parameter("pattern",   std::string("circle"));
    declare_parameter("duration",  60.0);
    declare_parameter("linear_v",  0.2);
    declare_parameter("angular_v", 0.3);

    pattern_   = get_parameter("pattern").as_string();
    duration_  = get_parameter("duration").as_double();
    linear_v_  = get_parameter("linear_v").as_double();
    angular_v_ = get_parameter("angular_v").as_double();

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    // Timer mit 20 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AutoDriver::tick, this));

    RCLCPP_INFO(get_logger(),
      "AutoDriver gestartet | pattern=%s duration=%.1fs v=%.2f w=%.2f",
      pattern_.c_str(), duration_, linear_v_, angular_v_);
  }

private:
  void tick()
  {
    double now = get_clock()->now().nanoseconds() * 1e-9;

    if (!started_) {
      t0_ = now;
      started_ = true;
    }

    double elapsed = now - t0_;

    if (elapsed >= duration_) {
      publish(0.0, 0.0);
      RCLCPP_INFO(get_logger(), "AutoDriver: Trajektorie beendet.");
      timer_->cancel();
      return;
    }

    auto [v, w] = get_cmd(elapsed);
    publish(v, w);
  }

  std::pair<double, double> get_cmd(double t)
  {
    if (pattern_ == "circle") {
      return {linear_v_, angular_v_};

    } else if (pattern_ == "straight") {
      return {linear_v_, 0.0};

    } else if (pattern_ == "eight") {
      // alle 10 Sekunden Richtung wechseln
      double period = 10.0;
      if (static_cast<int>(t / period) % 2 == 0) {
        return {linear_v_,  angular_v_};
      } else {
        return {linear_v_, -angular_v_};
      }

    } else if (pattern_ == "square") {
      // 3s gerade, 2s drehen
      double phase = std::fmod(t, 5.0);
      if (phase < 3.0) {
        return {linear_v_, 0.0};
      } else {
        return {0.0, angular_v_};
      }

    } else {
      RCLCPP_WARN_ONCE(get_logger(),
        "Unbekanntes pattern '%s', fahre Kreis", pattern_.c_str());
      return {linear_v_, angular_v_};
    }
  }

  void publish(double v, double w)
  {
    geometry_msgs::msg::TwistStamped msg;
    auto _t = get_clock()->now().nanoseconds();
    msg.header.stamp.sec     = static_cast<int32_t>(_t / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(_t % 1000000000LL);
    msg.header.frame_id = "base_link";
    msg.twist.linear.x  = v;
    msg.twist.angular.z = w;
    pub_->publish(msg);
  }

  std::string pattern_;
  double duration_, linear_v_, angular_v_;
  double t0_{0.0};
  bool   started_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoDriver>());
  rclcpp::shutdown();
  return 0;
}
