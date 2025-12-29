#include <array>
#include <unordered_map>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class WheelEncoderOdomNode : public rclcpp::Node
{
public:
  WheelEncoderOdomNode()
  : rclcpp::Node("wheel_encoder_odom_node")
  {
    wheel_joints_ = declare_parameter<std::vector<std::string>>(
        "wheel_joints",
        {"fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"});
    steering_joints_ = declare_parameter<std::vector<std::string>>(
        "steering_joints",
        {"fl_steering_joint", "fr_steering_joint", "rl_steering_joint", "rr_steering_joint"});
    wheel_base_ = declare_parameter<double>("wheel_base", 0.94);
    track_width_ = declare_parameter<double>("track_width", 0.65);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.11);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    const auto odom_topic =
        declare_parameter<std::string>("wheel_odom_topic", "/wheel_encoder/odometry");

    if (wheel_joints_.size() != 4 || steering_joints_.size() != 4) {
      RCLCPP_FATAL(get_logger(), "Expected 4 wheel_joints and 4 steering_joints");
      throw std::runtime_error("Invalid joint list size");
    }

    const auto js_topic = declare_parameter<std::string>("joint_state_topic", "/joint_states");

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        js_topic, rclcpp::SensorDataQoS(),
        std::bind(&WheelEncoderOdomNode::jointStateCb, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    const auto period = rclcpp::Rate(publish_rate_hz_).period();
    timer_ = create_wall_timer(period, std::bind(&WheelEncoderOdomNode::publishOdom, this));
  }

private:
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg; // keep a strong ref; weak_ptr would expire immediately
  }

  bool fillWheelStates(std::array<double, 4> &wheel_ang_vel,
                       std::array<double, 4> &steer_angle)
  {
    auto js = last_joint_state_;
    if (!js) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No joint_states received yet");
      return false;
    }

    std::unordered_map<std::string, size_t> name_to_idx;
    name_to_idx.reserve(js->name.size());
    for (size_t i = 0; i < js->name.size(); ++i) {
      name_to_idx[js->name[i]] = i;
    }

    for (size_t i = 0; i < 4; ++i) {
      const auto it_w = name_to_idx.find(wheel_joints_[i]);
      const auto it_s = name_to_idx.find(steering_joints_[i]);
      if (it_w == name_to_idx.end() || it_s == name_to_idx.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Missing joint_states for %s or %s",
                             wheel_joints_[i].c_str(), steering_joints_[i].c_str());
        return false;
      }
      const size_t idx_w = it_w->second;
      const size_t idx_s = it_s->second;
      if (idx_w >= js->velocity.size() || idx_s >= js->position.size()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Incomplete joint_states arrays");
        return false;
      }
      wheel_ang_vel[i] = js->velocity[idx_w];
      steer_angle[i] = js->position[idx_s];
      if (!std::isfinite(wheel_ang_vel[i])) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Wheel joint %s velocity is NaN/inf; treating as 0",
                             wheel_joints_[i].c_str());
        wheel_ang_vel[i] = 0.0;
      }
      if (!std::isfinite(steer_angle[i])) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Steering joint %s position is NaN/inf; treating as 0",
                             steering_joints_[i].c_str());
        steer_angle[i] = 0.0;
      }
    }
    return true;
  }

  void publishOdom()
  {
    std::array<double, 4> wheel_ang_vel{};
    std::array<double, 4> steer_angle{};
    if (!fillWheelStates(wheel_ang_vel, steer_angle)) {
      return;
    }

    const double hx = wheel_base_ * 0.5;
    const double hy = track_width_ * 0.5;
    const std::array<std::array<double, 2>, 4> wheel_pos = {{
        {+hx, -hy}, // FL
        {+hx, +hy}, // FR
        {-hx, -hy}, // RL
        {-hx, +hy}  // RR
    }};

    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;
    for (size_t i = 0; i < 4; ++i) {
      const double theta = steer_angle[i];
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      const double vx_coeff = c;
      const double vy_coeff = s;
      const double wz_coeff = -c * wheel_pos[i][1] + s * wheel_pos[i][0];
      A(i, 0) = vx_coeff;
      A(i, 1) = vy_coeff;
      A(i, 2) = wz_coeff;
      b(i, 0) = wheel_ang_vel[i] * wheel_radius_;
    }

    // Solve least squares for [vx, vy, wz]^T.
    Eigen::Matrix<double, 3, 3> AtA = A.transpose() * A;
    AtA += 1e-6 * Eigen::Matrix<double, 3, 3>::Identity(); // mild damping
    Eigen::Matrix<double, 3, 1> Atb = A.transpose() * b;
    Eigen::Matrix<double, 3, 1> twist = AtA.ldlt().solve(Atb);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;

    // Pose unknown: leave at origin with large covariance.
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    constexpr double big_cov = 1e6;
    odom.pose.covariance = {
        big_cov, 0, 0, 0, 0, 0,
        0, big_cov, 0, 0, 0, 0,
        0, 0, big_cov, 0, 0, 0,
        0, 0, 0, big_cov, 0, 0,
        0, 0, 0, 0, big_cov, 0,
        0, 0, 0, 0, 0, big_cov};

    odom.twist.twist.linear.x = twist(0);
    odom.twist.twist.linear.y = twist(1);
    odom.twist.twist.angular.z = twist(2);
    constexpr double twist_cov = 0.05;
    odom.twist.covariance = {
        twist_cov, 0, 0, 0, 0, 0,
        0, twist_cov, 0, 0, 0, 0,
        0, 0, twist_cov, 0, 0, 0,
        0, 0, 0, twist_cov, 0, 0,
        0, 0, 0, 0, twist_cov, 0,
        0, 0, 0, 0, 0, twist_cov};

    odom_pub_->publish(odom);
  }

  std::vector<std::string> wheel_joints_;
  std::vector<std::string> steering_joints_;
  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double publish_rate_hz_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelEncoderOdomNode>());
  rclcpp::shutdown();
  return 0;
}
