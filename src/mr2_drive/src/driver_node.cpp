#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

const std::string SWITCH_MODE_SERVICE = "/driving_node/switch_mode";
const std::string JOYSTICK_TOPIC = "/gamepad";
const std::string MODE_TOPIC = "/driving_node/is_autonomous";

class DrivingNode : public rclcpp::Node {
public:
  DrivingNode() : Node("driving_node") {
    // Create a service to switch modes (autonomous or manual)
    mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "switch_mode", std::bind(&DrivingNode::switch_mode_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // Create a publisher to broadcast the current mode
    mode_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>(MODE_TOPIC, 10);

    // Create a publisher to broadcast the target twist
    target_twist_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("target_twist", 10);

    // Subscribe to the joystick topic
    joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOYSTICK_TOPIC, 10,
        std::bind(&DrivingNode::joystick_callback, this,
                  std::placeholders::_1));

    // Initialize the timer
    last_joystick_time_ = now();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Check every 100ms
        std::bind(&DrivingNode::timeout_check, this));

    // Publish the initial mode
    autonomous_.data = false;
    mode_publisher_->publish(autonomous_);

    RCLCPP_INFO(this->get_logger(),
                "DrivingNode initialized in MANUAL mode (autonomous=false).");
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_joystick_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      target_twist_publisher_;
  geometry_msgs::msg::Twist target_twist_;
  std_msgs::msg::Bool
      autonomous_; // Current mode: true = autonomous, false = manual

  const int throttle_timeout_ms_ = 200;

  // Service callback for switching modes
  void switch_mode_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
      // Switch to autonomous mode
      autonomous_.data = true;
      mode_publisher_->publish(autonomous_);
      RCLCPP_INFO(this->get_logger(), "Switched to AUTONOMOUS mode.");
    } else {
      // Switch to manual mode
      autonomous_.data = false;
      mode_publisher_->publish(autonomous_);
      RCLCPP_INFO(this->get_logger(), "Switched to MANUAL mode.");
    }

    response->success = true;
    response->message = "Mode switch successful.";
  }

  // Callback function for joystick input
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    last_joystick_time_ = now();
    // Axis Indices
    const size_t THROTTLE_INDEX = 5; // R2
    const size_t BRAKE_INDEX = 4;    // L2
    const size_t STEERING_INDEX = 0; // Left stick x axis
    const size_t VX_INDEX = 2;       // Right stick x axis
    const size_t VY_INDEX = 3;       // Right stick y axis

    // Button Indices
    const size_t ABORT_BUTTON_INDEX = 5; // R1

    // Check if the joystick button is pressed
    double throttle = msg->axes[THROTTLE_INDEX];
    double brake = msg->axes[BRAKE_INDEX];
    double steering = msg->axes[STEERING_INDEX];
    double vx_offset = msg->axes[VX_INDEX];
    double vy_offset = -msg->axes[VY_INDEX];

    target_twist_.linear.x = vx_offset;
    target_twist_.linear.y = throttle - brake + vy_offset;
    target_twist_.angular.z = -steering;
  }

  void timeout_check() {
    // Check if the throttle has timed out
    auto time_since_last_joystick =
        (now() - last_joystick_time_).nanoseconds() /
        1e6; // Convert to milliseconds
    if (time_since_last_joystick > throttle_timeout_ms_) {
      target_twist_.linear.x = 0.0;
      target_twist_.linear.y = 0.0;
      target_twist_.angular.z = 0.0;

      RCLCPP_WARN(this->get_logger(), "Throttle timeout! Decelerating...");
      publish_velocity();
    }
  }

  void publish_velocity() { target_twist_publisher_->publish(target_twist_); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrivingNode>());
  rclcpp::shutdown();
  return 0;
}
