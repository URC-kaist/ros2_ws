#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mr2_action_interface/action/gnss_only.hpp"
#include <iostream>
#include <string>

#include "std_msgs/msg/int32.hpp"
#include <chrono>
using namespace std::chrono_literals;

class GnssOnlyClient : public rclcpp::Node
{
public:
  using GnssOnly = mr2_action_interface::action::GnssOnly;
  using GoalHandleGnssOnly = rclcpp_action::ClientGoalHandle<GnssOnly>;

  explicit GnssOnlyClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gnss_only_client", options)
  {
    this->action_client_ = rclcpp_action::create_client<GnssOnly>(
      this, "gnss_only");
    
    // Parameter for topic name (keeps old topic stable if you set it)
    // e.g., ros2 run ... --ros-args -p led_topic:=/led/status
    std::string led_topic = this->declare_parameter<std::string>("led_topic", "/led/status");
    led_pub_ = this->create_publisher<std_msgs::msg::Int32>(led_topic, rclcpp::QoS(1).best_effort());

    // Timer to reflect server availability when idle (no active goal)
    led_timer_ = this->create_wall_timer(250ms, [this]() {
      bool connected = action_client_->wait_for_action_server(0ms);
      if (connected != server_connected_) {
        server_connected_ = connected;
        if (!active_goal_) publish_led(connected ? IDLE : DISCONNECTED);
      }
      // Also ensure idle state is periodically asserted even if nothing happens
      if (!active_goal_) publish_led(connected ? IDLE : DISCONNECTED);
    });

    RCLCPP_INFO(this->get_logger(), "=== GNSS Navigation Client Ready ===");
    RCLCPP_INFO(this->get_logger(), "Commands:");
    RCLCPP_INFO(this->get_logger(), "  goto <lat> <lon> - Send navigation goal");
    RCLCPP_INFO(this->get_logger(), "  cancel           - Cancel current navigation");
    RCLCPP_INFO(this->get_logger(), "  status           - Show current status");
    RCLCPP_INFO(this->get_logger(), "  quit             - Exit client");
    
    // Start command input thread
    command_thread_ = std::thread(&GnssOnlyClient::command_loop, this);
  }

  ~GnssOnlyClient()
  {
    if (command_thread_.joinable()) {
      command_thread_.join();
    }
  }

  void send_goal(double latitude, double longitude)
  {
    if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "❌ Action server not available after waiting");
      return;
    }

    auto goal_msg = GnssOnly::Goal();
    goal_msg.target_latitude = latitude;
    goal_msg.target_longitude = longitude;

    RCLCPP_INFO(this->get_logger(), "🎯 Sending goal: lat=%.6f, lon=%.6f", latitude, longitude);

    auto send_goal_options = rclcpp_action::Client<GnssOnly>::SendGoalOptions();
    
    // Goal response callback
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleGnssOnly> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "❌ Goal was REJECTED by server");
          active_goal_ = false;
          publish_led(ABORTED);   // rejected -> treat as failure
        } else {
          RCLCPP_INFO(this->get_logger(), "✅ Goal ACCEPTED by server");
          current_goal_handle_ = goal_handle;
          active_goal_ = true;
          publish_led(ACTIVE);
        }
      };

    // Feedback callback - This is what LED node also listens to!
    send_goal_options.feedback_callback =
      [this](GoalHandleGnssOnly::SharedPtr,
              const std::shared_ptr<const GnssOnly::Feedback> feedback) {
        if (feedback->bt_status == 1) {
          RCLCPP_INFO(this->get_logger(), "🟡 BT Status: RUNNING (Navigation active)");
        } else {
          RCLCPP_INFO(this->get_logger(), "⚪ BT Status: NOT RUNNING (Navigation idle)");
        }
        if (active_goal_) publish_led(ACTIVE);
        current_bt_status_ = feedback->bt_status;
      };

    // Result callback
    send_goal_options.result_callback =
      [this](const GoalHandleGnssOnly::WrappedResult & result) {
        current_goal_handle_.reset();
        
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            if (result.result->navigation_result == 1) {
              RCLCPP_INFO(this->get_logger(), "🎉 Navigation SUCCEEDED!");
              publish_led(SUCCEEDED);
            } else {
              RCLCPP_WARN(this->get_logger(), "⚠️  Navigation completed but marked as NOT SUCCESS");
              publish_led(ABORTED)
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "❌ Navigation ABORTED");
            publish_led(ABORTED);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "🛑 Navigation CANCELLED");
            publish_led(CANCELED);
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "❓ Unknown result code");
            publish_led(ABORTED);
            break;
        }
      };

    this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_goal()
  {
    if (!current_goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "⚠️  No active goal to cancel");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "🛑 Cancelling current navigation...");
    auto future = this->action_client_->async_cancel_goal(current_goal_handle_);
    
    // You could wait for cancellation result if needed
    RCLCPP_INFO(this->get_logger(), "🛑 Cancel request sent - robot should stop safely");
    active_goal_ = false;
  }

  void show_status()
  {
    RCLCPP_INFO(this->get_logger(), "=== Current Status ===");
    if (current_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Goal: ACTIVE");
      RCLCPP_INFO(this->get_logger(), "BT Status: %s", 
                  current_bt_status_ == 1 ? "RUNNING" : "NOT RUNNING");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal: NONE");
      RCLCPP_INFO(this->get_logger(), "BT Status: IDLE");
    }
    RCLCPP_INFO(this->get_logger(), "=====================");
  }

private:
  rclcpp_action::Client<GnssOnly>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleGnssOnly> current_goal_handle_;
  int current_bt_status_ = 0;
  std::thread command_thread_;

  // --- LED publisher & connection monitoring ---
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr led_pub_;
  rclcpp::TimerBase::SharedPtr led_timer_;
  bool server_connected_{false};
  bool active_goal_{false};
  int last_led_{-999};  // avoid spamming identical publishes

  // publish helper
  void publish_led(int code) {
    if (code == last_led_) return;
    std_msgs::msg::Int32 m;
    m.data = code;
    led_pub_->publish(m);
    last_led_ = code;
  }

  // map status to LED codes (tune to your scheme)
  enum LedCode {
    DISCONNECTED = 0, // not connected to server; manual mode
    IDLE        = 1, // connected to server but not running; manual mode
    ACTIVE      = 2, // running; navigation mode
    SUCCEEDED   = 3, // success, target reached
    CANCELED    = 4, // cancel; switch to manual mode
    ABORTED     = 5 // unexpected case when server cannot complete; switch to manual mode
  };

  void command_loop()
  {
    std::string command;
    
    while (rclcpp::ok()) {
      std::cout << "\ngnss_client> ";
      std::getline(std::cin, command);
      
      if (command.empty()) continue;
      
      std::istringstream iss(command);
      std::string action;
      iss >> action;
      
      if (action == "goto") {
        double lat, lon;
        if (iss >> lat >> lon) {
          send_goal(lat, lon);
        } else {
          std::cout << "Usage: goto <latitude> <longitude>\n";
          std::cout << "Example: goto 37.7749 -122.4194\n";
        }
      }
      else if (action == "cancel") {
        cancel_goal();
      }
      else if (action == "status") {
        show_status();
      }
      else if (action == "quit" || action == "exit") {
        RCLCPP_INFO(this->get_logger(), "👋 Goodbye!");
        rclcpp::shutdown();
        break;
      }
      else {
        std::cout << "Unknown command: " << action << "\n";
        std::cout << "Available: goto, cancel, status, quit\n";
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GnssOnlyClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}