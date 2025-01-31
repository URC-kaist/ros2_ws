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

    this->declare_parameter("throttle_index", 5);        // R2
    this->declare_parameter("brake_index", 4);           // L2
    this->declare_parameter("steering_index", 0);        // Left stick x axis
    this->declare_parameter("vx_index", 2);              // Right stick x axis
    this->declare_parameter("vy_index", 3);              // Right stick y axis
    this->declare_parameter("throttle_timeout", 1000);   // ms
    this->declare_parameter("max_vx", 5.0);              // m/s
    this->declare_parameter("max_vy", 10.0);             // m/s
    this->declare_parameter("max_omega", 5.0);           // rad/s
    this->declare_parameter("max_x_acceleration", 10.0); // m/s^2
    this->declare_parameter("max_y_acceleration", 10.0); // m/s^2
    this->declare_parameter("max_lateral_acceleration", 10.0); // m/s^2
    this->declare_parameter("control_interval", 20);           // ms
                                                               //
    cache_parameters();

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
        std::chrono::milliseconds(control_interval_),
        std::bind(&DrivingNode::control_callback, this));

    // Setup parameter callback for dynamic reconfiguration
    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
        &DrivingNode::parameters_callback, this, std::placeholders::_1));

    // Publish the initial mode
    autonomous_.data = false;
    mode_publisher_->publish(autonomous_);

    RCLCPP_INFO(this->get_logger(),
                "DrivingNode initialized in MANUAL mode (autonomous=false).");
  }

private:
  int throttle_index_;
  int brake_index_;
  int steering_index_;
  int vx_index_;
  int vy_index_;
  int throttle_timeout_;
  double max_vx_;
  double max_vy_;
  double max_omega_;
  double max_x_acceleration_;
  double max_y_acceleration_;
  double max_lateral_acceleration_;
  int control_interval_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

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
  std::mutex parameter_mutex_; // Mutex for thread safety

  void cache_parameters() {
    std::lock_guard<std::mutex> lock(parameter_mutex_);
    this->get_parameter("throttle_index", throttle_index_);
    this->get_parameter("brake_index", brake_index_);
    this->get_parameter("steering_index", steering_index_);
    this->get_parameter("vx_index", vx_index_);
    this->get_parameter("vy_index", vy_index_);
    this->get_parameter("throttle_timeout", throttle_timeout_);
    this->get_parameter("max_vx", max_vx_);
    this->get_parameter("max_vy", max_vy_);
    this->get_parameter("max_omega", max_omega_);
    this->get_parameter("max_x_acceleration", max_x_acceleration_);
    this->get_parameter("max_y_acceleration", max_y_acceleration_);
    this->get_parameter("max_lateral_acceleration", max_lateral_acceleration_);
    this->get_parameter("control_interval", control_interval_);
  }

  // Parameter callback for dynamic updates
  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    std::lock_guard<std::mutex> lock(parameter_mutex_);

    for (const auto &param : parameters) {
      if (param.get_name() == "throttle_index") {
        throttle_index_ = param.as_int();
      } else if (param.get_name() == "brake_index") {
        brake_index_ = param.as_int();
      } else if (param.get_name() == "steering_index") {
        steering_index_ = param.as_int();
      } else if (param.get_name() == "vx_index") {
        vx_index_ = param.as_int();
      } else if (param.get_name() == "vy_index") {
        vy_index_ = param.as_int();
      } else if (param.get_name() == "throttle_timeout") {
        throttle_timeout_ = param.as_int();
      } else if (param.get_name() == "max_vx") {
        max_vx_ = param.as_double();
      } else if (param.get_name() == "max_vy") {
        max_vy_ = param.as_double();
      } else if (param.get_name() == "max_omega") {
        max_omega_ = param.as_double();
      } else if (param.get_name() == "max_x_acceleration") {
        max_x_acceleration_ = param.as_double();
      } else if (param.get_name() == "max_y_acceleration") {
        max_y_acceleration_ = param.as_double();
      } else if (param.get_name() == "max_lateral_acceleration") {
        max_lateral_acceleration_ = param.as_double();
      } else if (param.get_name() == "control_interval") {
        control_interval_ = param.as_int();
        // Reset the control timer with the new interval
        control_timer_->cancel();
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_interval_),
            std::bind(&DrivingNode::control_callback, this));
      }
    }

    return result;
  }
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

      // Check if the joystick button is pressed
      double throttle = msg->axes[throttle_index_];
      double brake = msg->axes[brake_index_];
      double steering = msg->axes[steering_index_];
      double vx_offset = msg->axes[vx_index_];
      double vy_offset = -msg->axes[vy_index_];

      target_twist_.linear.x = vx_offset * max_vx_;
      target_twist_.linear.y =
          std::clamp(throttle - brake + vy_offset, -1.0, 1.0) * max_vy_;
      target_twist_.angular.z = -steering * max_omega_;

      publish_velocity();
    }
  }

  // Control timer callback
  void control_callback() {
    float dt = control_interval_ / 1000.0f; // Convert milliseconds to seconds

    // Current velocities
    double current_vx = current_twist_.linear.x;
    double current_vy = current_twist_.linear.y;
    double current_omega = current_twist_.angular.z;

    // Target velocities
    double target_vx = target_twist_.linear.x;
    double target_vy = target_twist_.linear.y;
    double target_omega = target_twist_.angular.z;

    // Calculate desired changes in velocity
    double delta_vx = target_vx - current_vx;
    double delta_vy = target_vy - current_vy;
    double delta_omega = target_omega - current_omega;

    // Clamp the changes based on maximum accelerations
    delta_vx = std::clamp(delta_vx, -max_x_acceleration_ * dt,
                          max_x_acceleration_ * dt);
    delta_vy = std::clamp(delta_vy, -max_y_acceleration_ * dt,
                          max_y_acceleration_ * dt);
    delta_omega = std::clamp(delta_omega, -max_lateral_acceleration_ * dt,
                             max_lateral_acceleration_ * dt);

    // Update current velocities
    current_vx += delta_vx;
    current_vy += delta_vy;
    current_omega += delta_omega;

    // Clamp the current velocities to their maximum limits
    current_vx = std::clamp(current_vx, -max_vx_, max_vx_);
    current_vy = std::clamp(current_vy, -max_vy_, max_vy_);
    current_omega = std::clamp(current_omega, -max_omega_, max_omega_);

    // Update the current_twist_ message
    current_twist_.linear.x = current_vx;
    current_twist_.linear.y = current_vy;
    current_twist_.angular.z = current_omega;

    // Publish the updated current twist
    current_twist_publisher_->publish(current_twist_);
  }

  void timeout_check() {
    // Check if the throttle has timed out
    auto time_since_last_joystick =
        (now() - last_joystick_time_).nanoseconds() /
        1e6; // Convert to milliseconds
    if (time_since_last_joystick > throttle_timeout_) {

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
