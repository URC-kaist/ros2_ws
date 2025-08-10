#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mr2_action_interface/action/gnss_only.hpp"
#include "mr2_action_interface/action/cover_vision.hpp"
#include <iostream>
#include <string>
#include "std_msgs/msg/int32.hpp"
#include <chrono>
using namespace std::chrono_literals;

class MultiMissionClient : public rclcpp::Node
{
public:
  using GnssOnly = mr2_action_interface::action::GnssOnly;
  using CoverVision = mr2_action_interface::action::CoverVision;
  using GnssGoalHandle = rclcpp_action::ClientGoalHandle<GnssOnly>;
  using CoverGoalHandle = rclcpp_action::ClientGoalHandle<CoverVision>;

  explicit MultiMissionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("multi_mission_client", options)
  {
    // Create action clients for both mission types
    this->gnss_client_ = rclcpp_action::create_client<GnssOnly>(this, "gnss_only_mission");
    this->cover_client_ = rclcpp_action::create_client<CoverVision>(this, "cover_vision_mission");
    // Parameter for topic name (keeps old topic stable if you set it)
    // e.g., ros2 run ... --ros-args -p led_topic:=/led/status

    std::string led_topic = this->declare_parameter<std::string>("led_topic", "/led/status");
    led_pub_ = this->create_publisher<std_msgs::msg::Int32>(led_topic, rclcpp::QoS(1).best_effort());

    // Timer to reflect server availability when idle (no active goal)
    led_timer_ = this->create_wall_timer(250ms, [this]() {
      bool connected = gnss_client_->wait_for_action_server(0ms) || cover_client_->wait_for_action_server(0ms);
      if (connected != server_connected_) {
        server_connected_ = connected;
        if (!active_goal_) publish_led(connected ? IDLE : DISCONNECTED);
      }
      // Also ensure idle state is periodically asserted even if nothing happens
      if (!active_goal_) publish_led(connected ? IDLE : DISCONNECTED);
    });

    RCLCPP_INFO(this->get_logger(), "=== Multi-Mission Client Ready ===");
    RCLCPP_INFO(this->get_logger(), "Commands:");
    RCLCPP_INFO(this->get_logger(), "  gnss <lat> <lon>     - GNSS navigation mission");
    // RCLCPP_INFO(this->get_logger(), "  cover <lat> <lon>    - Coverage mission (generates waypoints)");
    RCLCPP_INFO(this->get_logger(), "  cancel               - Cancel current mission");
    RCLCPP_INFO(this->get_logger(), "  status               - Show current status");
    RCLCPP_INFO(this->get_logger(), "  quit                 - Exit client");
    
    // Start command input thread
    command_thread_ = std::thread(&MultiMissionClient::command_loop, this);
  }

  ~MultiMissionClient()
  {
    if (command_thread_.joinable()) {
      command_thread_.join();
    }
  }

  void send_gnss_goal(double latitude, double longitude)
  {
    if (!this->gnss_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "❌ GNSS action server not available");
      return;
    }

    auto goal_msg = GnssOnly::Goal();
    goal_msg.target_latitude = latitude;
    goal_msg.target_longitude = longitude;

    RCLCPP_INFO(this->get_logger(), "🎯 Sending GNSS goal: lat=%.6f, lon=%.6f", latitude, longitude);

    auto send_goal_options = rclcpp_action::Client<GnssOnly>::SendGoalOptions();
    
    // Goal response callback
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GnssGoalHandle> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "❌ GNSS goal was REJECTED");
          active_goal_ = false;
          publish_led(ABORTED);   // rejected -> treat as failure
        } else {
          RCLCPP_INFO(this->get_logger(), "✅ GNSS goal ACCEPTED");
          current_gnss_goal_ = goal_handle;
          current_mission_type_ = MissionType::GNSS;
          active_goal_ = true;
          publish_led(ACTIVE);
        }
      };

    // Feedback callback - This is what LED node also listens to!
    send_goal_options.feedback_callback =
      [this](std::shared_ptr<GnssGoalHandle>, const std::shared_ptr<const GnssOnly::Feedback> feedback) {
        if (feedback->bt_status == 1) {
          RCLCPP_INFO(this->get_logger(), "🟡 GNSS BT: RUNNING");
        } else {
          RCLCPP_INFO(this->get_logger(), "⚪ GNSS BT: IDLE");
        }
        if (active_goal_) publish_led(ACTIVE);
        current_bt_status_ = feedback->bt_status;
      };

    // Result callback
    send_goal_options.result_callback =
      [this](const GnssGoalHandle::WrappedResult & result) {
        current_gnss_goal_.reset();
        current_mission_type_ = MissionType::NONE;
        
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "🎉 GNSS Mission SUCCEEDED!");
            publish_led(SUCCEEDED);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "❌ GNSS Mission ABORTED");
            publish_led(ABORTED);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "🛑 GNSS Mission CANCELLED");
            publish_led(CANCELED);
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "❓ GNSS Mission unknown result");
            publish_led(ABORTED);
            break;
        }
      };

    this->gnss_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void send_cover_goal(double latitude, double longitude)
  {
    if (!this->cover_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "❌ Cover Vision action server not available");
      return;
    }

    auto goal_msg = CoverVision::Goal();
    goal_msg.target_latitude = latitude;
    goal_msg.target_longitude = longitude;

    RCLCPP_INFO(this->get_logger(), "🎯 Sending Coverage goal: lat=%.6f, lon=%.6f", latitude, longitude);
    RCLCPP_INFO(this->get_logger(), "   (Waypoints will be generated automatically)");

    auto send_goal_options = rclcpp_action::Client<CoverVision>::SendGoalOptions();
    
    // Goal response callback
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<CoverGoalHandle> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "❌ Coverage goal was REJECTED");
          active_goal_ = false;
          publish_led(ABORTED);   // rejected -> treat as failure
        } else {
          RCLCPP_INFO(this->get_logger(), "✅ Coverage goal ACCEPTED");
          current_cover_goal_ = goal_handle;
          current_mission_type_ = MissionType::COVER;
          active_goal_ = true;
          publish_led(ACTIVE);
        }
      };

    // Feedback callback - This is what LED node also listens to!
    send_goal_options.feedback_callback =
      [this](std::shared_ptr<CoverGoalHandle>, const std::shared_ptr<const CoverVision::Feedback> feedback) {
        if (feedback->bt_status == 1) {
          RCLCPP_INFO(this->get_logger(), "🟡 Coverage BT: RUNNING (Waypoint %d/%d)", 
                      feedback->current_waypoint_index + 1, feedback->total_waypoints);
        } else {
          RCLCPP_INFO(this->get_logger(), "⚪ Coverage BT: IDLE");
        }
        if (active_goal_) publish_led(ACTIVE);
        current_bt_status_ = feedback->bt_status;
      };

    // Result callback
    send_goal_options.result_callback =
      [this](const CoverGoalHandle::WrappedResult & result) {
        current_cover_goal_.reset();
        current_mission_type_ = MissionType::NONE;
        
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "🎉 Coverage Mission SUCCEEDED!");
            RCLCPP_INFO(this->get_logger(), "   Covered %d waypoints total", result.result->waypoints_completed);
            publish_led(SUCCEEDED);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "❌ Coverage Mission ABORTED");
            publish_led(ABORTED);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "🛑 Coverage Mission CANCELLED");
            publish_led(CANCELED);
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "❓ Coverage Mission unknown result");
            publish_led(ABORTED);
            break;
        }
      };

    this->cover_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_current_goal()
  {
    switch (current_mission_type_) {
      case MissionType::GNSS:
        if (current_gnss_goal_) {
          RCLCPP_INFO(this->get_logger(), "🛑 Cancelling GNSS mission...");
          this->gnss_client_->async_cancel_goal(current_gnss_goal_);
        } else {
          RCLCPP_WARN(this->get_logger(), "⚠️  No active GNSS mission to cancel");
        }
        break;
        
      case MissionType::COVER:
        if (current_cover_goal_) {
          RCLCPP_INFO(this->get_logger(), "🛑 Cancelling Coverage mission...");
          this->cover_client_->async_cancel_goal(current_cover_goal_);
        } else {
          RCLCPP_WARN(this->get_logger(), "⚠️  No active Coverage mission to cancel");
        }
        break;
        
      case MissionType::NONE:
        RCLCPP_WARN(this->get_logger(), "⚠️  No active mission to cancel");
        break;
    }
  }

  void show_status()
  {
    RCLCPP_INFO(this->get_logger(), "=== Current Status ===");
    
    switch (current_mission_type_) {
      case MissionType::GNSS:
        RCLCPP_INFO(this->get_logger(), "Mission: GNSS Navigation");
        break;
      case MissionType::COVER:
        RCLCPP_INFO(this->get_logger(), "Mission: Coverage Mission");
        break;
      case MissionType::NONE:
        RCLCPP_INFO(this->get_logger(), "Mission: NONE");
        break;
    }
    
    RCLCPP_INFO(this->get_logger(), "BT Status: %s", 
                current_bt_status_ == 1 ? "RUNNING" : "IDLE");
    RCLCPP_INFO(this->get_logger(), "=====================");
  }

private:
  enum class MissionType { NONE, GNSS, COVER };
  
  rclcpp_action::Client<GnssOnly>::SharedPtr gnss_client_;
  rclcpp_action::Client<CoverVision>::SharedPtr cover_client_;
  
  std::shared_ptr<GnssGoalHandle> current_gnss_goal_;
  std::shared_ptr<CoverGoalHandle> current_cover_goal_;
  
  MissionType current_mission_type_ = MissionType::NONE;
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
      std::cout << "\nmission> ";
      std::getline(std::cin, command);
      
      if (command.empty()) continue;
      
      std::istringstream iss(command);
      std::string action;
      iss >> action;
      
      if (action == "gnss") {
        double lat, lon;
        if (iss >> lat >> lon) {
          send_gnss_goal(lat, lon);
        } else {
          std::cout << "Usage: gnss <latitude> <longitude>\n";
          std::cout << "Example: gnss 37.7749 -122.4194\n";
        }
      }
      else if (action == "cover") {
        double lat, lon;
        if (iss >> lat >> lon) {
          send_cover_goal(lat, lon);
        } else {
          std::cout << "Usage: cover <latitude> <longitude>\n";
          std::cout << "Example: cover 37.7749 -122.4194\n";
        }
      }
      else if (action == "cancel") {
        cancel_current_goal();
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
        std::cout << "Available: gnss, cover, cancel, status, quit\n";
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiMissionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}