#include <chrono>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class TwistToCommands : public rclcpp::Node
{
public:
  TwistToCommands() : Node("twist_to_commands")
  {
    // ─── Parameters ───────────────────────────────────────────────
    wheel_base_   = declare_parameter("wheel_base",   0.94);   // m  (front axle to rear axle)
    track_width_  = declare_parameter("track_width",  0.65);   // m  (left to right)
    wheel_radius_ = declare_parameter("wheel_radius", 0.11);   // m
    max_steer_    = declare_parameter("max_steer",    0.6);   // rad
    publish_rate_ = declare_parameter("publish_rate_hz", 50.0);
    timeout_sec_  = declare_parameter("twist_timeout", 0.5);   // stop if no twist after this

    drive_topic_    = declare_parameter("drive_topic",    "/drive_controller/commands");
    steering_topic_ = declare_parameter("steering_topic", "/steering_controller/commands");

    // ─── Publishers & Subscriber ─────────────────────────────────
    pub_drive_    = create_publisher<std_msgs::msg::Float64MultiArray>(drive_topic_, 10);
    pub_steering_ = create_publisher<std_msgs::msg::Float64MultiArray>(steering_topic_, 10);

    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TwistToCommands::twistCb, this, std::placeholders::_1));

    // ─── Timer for periodic publish ──────────────────────────────
    const auto dt = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(dt),
                               std::bind(&TwistToCommands::tick, this));

    last_twist_time_ = now();
    last_twist_.linear.x  = 0.0;
    last_twist_.angular.z = 0.0;

    RCLCPP_INFO(get_logger(), "TwistToCommands node started.");
  }

private:
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_twist_ = *msg;
    last_twist_time_ = now();
  }

  void tick()
	{
	  // Timeout -> send zeros
	  if ((now() - last_twist_time_).seconds() > timeout_sec_) {
	    publishZeros();
	    return;
	  }

	  const double vx = last_twist_.linear.x;   // m/s  (forward in base frame +X)
	  const double vy = last_twist_.linear.y;   // m/s  (left  in base frame +Y)
	  const double wz = last_twist_.angular.z;  // rad/s (yaw about +Z)

	  // Wheel positions wrt base_link origin (center), in meters.
	  // Front (+x), rear (-x), left (+y), right (-y)
	  const double hx = wheel_base_ * 0.5;
	  const double hy = track_width_ * 0.5;

	  struct Wheel { double x; double y; };
	  // Order MUST match your YAML joint list: fl, fr, rl, rr
	  const std::array<Wheel,4> wheels = {{
	    { +hx, -hy },  // FL
	    { +hx, +hy },  // FR
	    { -hx, -hy },  // RL
	    { -hx, +hy }   // RR
	  }};

	  std::array<double,4> steer{};
	  std::array<double,4> speed{};

	  for (size_t i = 0; i < wheels.size(); ++i) {
	    const auto &w = wheels[i];

	    // Velocity at wheel = body lin vel + omega x r
	    // omega x r (2D) = [-wz * y, wz * x]
	    const double vx_i = vx - wz * w.y;
	    const double vy_i = vy + wz * w.x;

	    // Steering angle (rad), range (-pi, pi]
	    double ang = std::atan2(vy_i, vx_i);

	    // Linear wheel speed magnitude
	    double v_lin = std::hypot(vx_i, vy_i);

	    // Convert to wheel angular speed
	    double w_ang = (wheel_radius_ > 1e-9) ? (v_lin / wheel_radius_) : 0.0;

	    // Keep steering below 180° (|ang| <= pi). If |ang| > 90°, flip direction
	    // so joint doesn’t need to rotate a full half-turn.
	    if (std::abs(ang) > M_PI_2) {
	      // Move angle by ±pi into the shorter side
	      if (ang > 0) ang -= M_PI; else ang += M_PI;
	      w_ang = -w_ang; // reverse wheel spin
	    }

	    steer[i] = ang;
	    speed[i] = w_ang;
	  }

	  // Pack & publish
	  std_msgs::msg::Float64MultiArray drive_msg;
	  drive_msg.data.assign(speed.begin(), speed.end());

	  std_msgs::msg::Float64MultiArray steer_msg;
	  steer_msg.data.assign(steer.begin(), steer.end());

	  pub_drive_->publish(drive_msg);
	  pub_steering_->publish(steer_msg);
	}


  void publishZeros()
  {
    std_msgs::msg::Float64MultiArray drive_msg;
    drive_msg.data = {0.0, 0.0, 0.0, 0.0};
    std_msgs::msg::Float64MultiArray steer_msg;
    steer_msg.data = {0.0, 0.0, 0.0, 0.0};
    pub_drive_->publish(drive_msg);
    pub_steering_->publish(steer_msg);
  }

  // ─── Params ─────────────────────────────────────────────────────
  double wheel_base_, track_width_, wheel_radius_, rear_steer_ratio_;
  double max_steer_, publish_rate_, timeout_sec_;
  std::string drive_topic_, steering_topic_;

  // ─── ROS objects ────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_steering_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_twist_time_;
  geometry_msgs::msg::Twist last_twist_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToCommands>());
  rclcpp::shutdown();
  return 0;
}

