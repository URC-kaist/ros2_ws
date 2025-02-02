#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

const std::string JOINT_STATE_TOPIC = "/steering_node/joint_states";
const std::string TWIST_TOPIC = "/driving_node/current_twist";

class SteeringNode : public rclcpp::Node {
public:
  SteeringNode() : Node("steering_node") {

    this->declare_parameter("rover_width", 0.65);   // m
    this->declare_parameter("rover_length", 0.954); // m

    cache_parameters();

    target_twist_subscriber_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            TWIST_TOPIC, 10,
            std::bind(&SteeringNode::twist_callback, this,
                      std::placeholders::_1));

    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC,
                                                             10);

    joint_state_.name = {"front_left_wheel_joint", "front_right_wheel_joint",
                         "back_left_wheel_joint", "back_right_wheel_joint"};
    joint_state_.position = {0.0, 0.0, 0.0, 0.0};
    joint_state_.velocity = {0.0, 0.0, 0.0, 0.0};
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      target_twist_subscriber_;
  double rover_width_;
  double rover_length_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  sensor_msgs::msg::JointState joint_state_;

  void cache_parameters() {
    this->get_parameter("rover_width", rover_width_);
    this->get_parameter("rover_length", rover_length_);
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    update_joint_state(*msg);
    joint_state_publisher_->publish(joint_state_);
  }

  void update_joint_state(const geometry_msgs::msg::Twist &twist) {
    double wheel_positions[4][2] = {
        {-rover_width_ / 2.0, rover_length_ / 2.0},
        {rover_width_ / 2.0, rover_length_ / 2.0},
        {-rover_width_ / 2.0, -rover_length_ / 2.0},
        {rover_width_ / 2.0, -rover_length_ / 2.0},
    };

    double omega = twist.angular.z;

    double wheel_velocities[4][2];

    for (int i = 0; i < 4; i++) {
      double vx = twist.linear.x;
      double vy = twist.linear.y;

      wheel_velocities[i][0] = vx - omega * wheel_positions[i][1];
      wheel_velocities[i][1] = vy + omega * wheel_positions[i][0];
    }

    double steering_angles[4];
    double wheel_speeds[4];

    for (int i = 0; i < 4; i++) {
      double vx = wheel_velocities[i][0];
      double vy = wheel_velocities[i][1];

      steering_angles[i] =
          std::remainder(std::atan2(vy, vx) - M_PI / 2.0, M_PI * 2.0);

      wheel_speeds[i] = std::sqrt(vx * vx + vy * vy);

      if (std::abs(steering_angles[i]) > M_PI / 2.0) {
        steering_angles[i] =
            std::remainder(steering_angles[i] + M_PI, M_PI * 2.0);
        wheel_speeds[i] *= -1.0;
      }

      if (vx == 0.0 && vy == 0.0) {
        steering_angles[i] = 0.0;
        wheel_speeds[i] = 0.0;
      }
    }

    joint_state_.position[0] = steering_angles[0];
    joint_state_.position[1] = steering_angles[1];
    joint_state_.position[2] = steering_angles[2];
    joint_state_.position[3] = steering_angles[3];

    joint_state_.velocity[0] = wheel_speeds[0];
    joint_state_.velocity[1] = wheel_speeds[1];
    joint_state_.velocity[2] = wheel_speeds[2];
    joint_state_.velocity[3] = wheel_speeds[3];
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringNode>());
  rclcpp::shutdown();
  return 0;
}
