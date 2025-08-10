#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "your_interfaces/action/gnss_navigate.hpp"
#include <iostream>
#include <string>

class GnssNavigationClient : public rclcpp::Node
{
public:
  using GnssNavigate = your_interfaces::action::GnssNavigate;
  using GoalHandleGnssNavigate = rclcpp_action::ClientGoalHandle<GnssNavigate>;

  explicit GnssNavigationClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gnss_navigation_client", options)
  {
    this->action_client_ = rclcpp_action::create_client<GnssNavigate>(
      this, "gnss_navigate");

    RCLCPP_INFO(this->get_logger(), "=== GNSS Navigation Client Ready ===");
    RCLCPP_INFO(this->get_logger(), "Commands:");
    RCLCPP_INFO(this->get_logger(), "  goto <lat> <lon> - Send navigation goal");
    RCLCPP_INFO(this->get_logger(), "  cancel           - Cancel current navigation");
    RCLCPP_INFO(this->get_logger(), "  status           - Show current status");
    RCLCPP_INFO(this->get_logger(), "  quit             - Exit client");
    
    // Start command input thread
    command_thread_ = std::thread(&GnssNavigationClient::command_loop, this);
  }

  ~GnssNavigationClient()
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

    auto goal_msg = GnssNavigate::Goal();
    goal_msg.target_latitude = latitude;
    goal_msg.target_longitude = longitude;

    RCLCPP_INFO(this->get_logger(), "🎯 Sending goal: lat=%.6f, lon=%.6f", latitude, longitude);

    auto send_goal_options = rclcpp_action::Client<GnssNavigate>::SendGoalOptions();
    
    // Goal response callback
    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleGnssNavigate> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "❌ Goal was REJECTED by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "✅ Goal ACCEPTED by server");
          current_goal_handle_ = goal_handle;
        }
      };

    // Feedback callback - This is what LED node also listens to!
    send_goal_options.feedback_callback =
      [this](GoalHandleGnssNavigate::SharedPtr,
              const std::shared_ptr<const GnssNavigate::Feedback> feedback) {
        if (feedback->bt_status == 1) {
          RCLCPP_INFO(this->get_logger(), "🟡 BT Status: RUNNING (Navigation active)");
        } else {
          RCLCPP_INFO(this->get_logger(), "⚪ BT Status: NOT RUNNING (Navigation idle)");
        }
        current_bt_status_ = feedback->bt_status;
      };

    // Result callback
    send_goal_options.result_callback =
      [this](const GoalHandleGnssNavigate::WrappedResult & result) {
        current_goal_handle_.reset();
        
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            if (result.result->navigation_result == 1) {
              RCLCPP_INFO(this->get_logger(), "🎉 Navigation SUCCEEDED!");
            } else {
              RCLCPP_WARN(this->get_logger(), "⚠️  Navigation completed but marked as NOT SUCCESS");
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "❌ Navigation ABORTED");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "🛑 Navigation CANCELLED");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "❓ Unknown result code");
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
  rclcpp_action::Client<GnssNavigate>::SharedPtr action_client_;
  std::shared_ptr<GoalHandleGnssNavigate> current_goal_handle_;
  int current_bt_status_ = 0;
  std::thread command_thread_;

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
  auto node = std::make_shared<GnssNavigationClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}