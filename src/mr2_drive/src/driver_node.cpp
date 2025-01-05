#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

const std::string SWITCH_MODE_SERVICE = "/driving_node/switch_mode";
const std::string JOYSTICK_TOPIC = "/gamepad";
const std::string MODE_TOPIC = "/driving_node/is_autonomous";
const std::string TARGET_TWIST_TOPIC = "/driving_node/target_twist";
const std::string CURRENT_TWIST_TOPIC = "/driving_node/current_twist";

// TODO: Implement autonomous mode

class DrivingNode : public rclcpp::Node {
public:
  DrivingNode() : Node("driving_node") {

    this->declare_parameter("throttle_index", 5);         // R2
    this->declare_parameter("brake_index", 4);            // L2
    this->declare_parameter("steering_index", 0);         // Left stick x axis
    this->declare_parameter("vx_index", 2);               // Right stick x axis
    this->declare_parameter("vy_index", 3);               // Right stick y axis
    this->declare_parameter("throttle_timeout_ms", 1000); // ms
    this->declare_parameter("max_vx", 5.0);               // m/s
    this->declare_parameter("max_vy", 10.0);              // m/s
    this->declare_parameter("max_omega", 5.0);            // rad/s
    this->declare_parameter("max_x_acceleration", 10.0);  // m/s^2
    this->declare_parameter("max_y_acceleration", 10.0);  // m/s^2
    this->declare_parameter("max_lateral_acceleration", 10.0); // m/s^2
    this->declare_parameter("control_interval", 10);           // ms

    // Create a service to switch modes (autonomous or manual)
    mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "switch_mode", std::bind(&DrivingNode::switch_mode_callback, this,
                                 std::placeholders::_1, std::placeholders::_2));

    // Create a publisher to broadcast the current mode
    mode_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>(MODE_TOPIC, 10);

    // Create a publisher to broadcast the target twist
    target_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        TARGET_TWIST_TOPIC, 10);

    // Create a publisher to broadcast the current twist
    current_twist_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(CURRENT_TWIST_TOPIC,
                                                          10);

    // Subscribe to the joystick topic
    joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOYSTICK_TOPIC, 10,
        std::bind(&DrivingNode::joystick_callback, this,
                  std::placeholders::_1));

    // Initialize the timer
    last_joystick_time_ = now();
    joystick_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // Check every 100ms
        std::bind(&DrivingNode::timeout_check, this));

    // Initialize Control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("control_interval").as_int()),
        std::bind(&DrivingNode::control_callback, this));

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
  rclcpp::TimerBase::SharedPtr joystick_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Time last_joystick_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      target_twist_publisher_,
      current_twist_publisher_;
  geometry_msgs::msg::Twist target_twist_, current_twist_;
  bool throttle_timed_out_ = false;
  std_msgs::msg::Bool
      autonomous_; // Current mode: true = autonomous, false = manual

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
    if (!autonomous_.data) {
      throttle_timed_out_ = false;
      last_joystick_time_ = now();
      // Axis Indices
      size_t throttle_index = this->get_parameter("throttle_index").as_int();
      size_t brake_index = this->get_parameter("brake_index").as_int();
      size_t steering_index = this->get_parameter("steering_index").as_int();
      size_t vx_index = this->get_parameter("vx_index").as_int();
      size_t vy_index = this->get_parameter("vy_index").as_int();
      float max_vx = this->get_parameter("max_vx").as_double();
      float max_vy = this->get_parameter("max_vy").as_double();
      float max_omega = this->get_parameter("max_omega").as_double();

      // Check if the joystick button is pressed
      double throttle = msg->axes[throttle_index];
      double brake = msg->axes[brake_index];
      double steering = msg->axes[steering_index];
      double vx_offset = msg->axes[vx_index];
      double vy_offset = -msg->axes[vy_index];

      target_twist_.linear.x = vx_offset * max_vx;
      target_twist_.linear.y =
          std::clamp(throttle - brake + vy_offset, -1.0, 1.0) * max_vy;
      target_twist_.angular.z = -steering * max_omega;

      publish_velocity();
    }
  }

  // Control timer callback
  void control_callback() {
    float dt = this->get_parameter("control_interval").as_int() / 1000.0;
    float max_x_acceleration =
        this->get_parameter("max_x_acceleration").as_double();
    float max_y_acceleration =
        this->get_parameter("max_y_acceleration").as_double();
    float max_lateral_acceleration =
        this->get_parameter("max_lateral_acceleration").as_double();

    float target_vx = target_twist_.linear.x;
    float target_vy = target_twist_.linear.y;
    float target_omega = target_twist_.angular.z;

    // TODO: Implement control logic
  }

  void timeout_check() {
    // Check if the throttle has timed out
    int throttle_timeout_ms =
        this->get_parameter("throttle_timeout_ms").as_int();
    auto time_since_last_joystick =
        (now() - last_joystick_time_).nanoseconds() /
        1e6; // Convert to milliseconds
    if (time_since_last_joystick > throttle_timeout_ms) {

      target_twist_.linear.x = 0.0;
      target_twist_.linear.y = 0.0;
      target_twist_.angular.z = 0.0;

      if (!throttle_timed_out_) {
        throttle_timed_out_ = true;
        RCLCPP_WARN(this->get_logger(), "Throttle timeout! Decelerating...");
      }

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
