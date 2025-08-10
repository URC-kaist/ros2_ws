#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "your_interfaces/action/gnss_navigate.hpp"  // Your custom action interface
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <thread>

class GnssNavigationActionServer : public rclcpp::Node
{
public:
  using GnssNavigate = your_interfaces::action::GnssNavigate;
  using GoalHandleGnssNavigate = rclcpp_action::ServerGoalHandle<GnssNavigate>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit GnssNavigationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gnss_navigation_action_server", options)
  {
    // Create the action server for GNSS navigation
    this->action_server_ = rclcpp_action::create_server<GnssNavigate>(
      this,
      "gnss_navigate",
      std::bind(&GnssNavigationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GnssNavigationActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&GnssNavigationActionServer::handle_accepted, this, std::placeholders::_1));

    // Create action client for Nav2's NavigateToPose (which will use your GnssOnly.xml BT)
    this->nav2_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    // Publisher for emergency stop
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "GNSS Navigation Action Server initialized");
  }

private:
  rclcpp_action::Server<GnssNavigate>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GnssNavigate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), 
                "Received GNSS navigation goal: lat=%.6f, lon=%.6f", 
                goal->target_latitude, goal->target_longitude);
    
    // Basic validation
    if (std::abs(goal->target_latitude) > 90.0 || std::abs(goal->target_longitude) > 180.0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid GPS coordinates");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Check if Nav2 is available
    if (!nav2_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGnssNavigate> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request - stopping robot safely");
    
    // Send zero velocity commands to stop robot smoothly
    auto stop_cmd = geometry_msgs::msg::Twist();
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    
    // Publish stop command multiple times for safety
    for (int i = 0; i < 5; ++i) {
      cmd_vel_pub_->publish(stop_cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Cancel the underlying Nav2 action if active
    if (current_nav2_goal_handle_) {
      auto cancel_result = nav2_action_client_->async_cancel_goal(current_nav2_goal_handle_);
      RCLCPP_INFO(this->get_logger(), "Cancelled Nav2 navigation goal");
    }
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGnssNavigate> goal_handle)
  {
    // Execute in separate thread to avoid blocking
    std::thread{std::bind(&GnssNavigationActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGnssNavigate> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GnssNavigate::Feedback>();
    auto result = std::make_shared<GnssNavigate::Result>();
    
    RCLCPP_INFO(this->get_logger(), "Starting GNSS navigation execution");
    
    try {
      // Convert GPS coordinates to map coordinates (you'll need to implement this)
      auto target_pose = convert_gps_to_map_pose(goal->target_latitude, goal->target_longitude);
      
      // Create Nav2 goal with your custom BT
      auto nav2_goal = NavigateToPose::Goal();
      nav2_goal.pose = target_pose;
      nav2_goal.behavior_tree = get_package_share_directory("your_package") + "/bt_trees/GnssOnly.xml";
      
      // Set BT running feedback
      feedback->bt_status = 1;  // RUNNING
      goal_handle->publish_feedback(feedback);
      
      // Send goal to Nav2
      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      
      // Set up callbacks for Nav2 action
      send_goal_options.goal_response_callback = 
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected");
          } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted");
            current_nav2_goal_handle_ = goal_handle;
          }
        };
      
      send_goal_options.feedback_callback =
        [this, goal_handle, feedback](
          std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>,
          const std::shared_ptr<const NavigateToPose::Feedback> nav2_feedback) {
          
          // Keep reporting BT as running while Nav2 is active
          feedback->bt_status = 1;  // RUNNING
          goal_handle->publish_feedback(feedback);
          
          RCLCPP_DEBUG(this->get_logger(), "Nav2 feedback received");
        };
      
      send_goal_options.result_callback =
        [this, goal_handle, feedback, result](
          const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & nav2_result) {
          
          // BT finished running
          feedback->bt_status = 0;  // NOT RUNNING
          goal_handle->publish_feedback(feedback);
          
          // Process Nav2 result
          if (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "GNSS navigation succeeded");
            result->navigation_result = 1;  // SUCCESS
            goal_handle->succeed(result);
          } else if (nav2_result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_INFO(this->get_logger(), "GNSS navigation was cancelled");
            result->navigation_result = 0;  // NOT SUCCESS
            goal_handle->canceled(result);
          } else {
            RCLCPP_ERROR(this->get_logger(), "GNSS navigation failed");
            result->navigation_result = 0;  // NOT SUCCESS
            goal_handle->abort(result);
          }
          
          current_nav2_goal_handle_.reset();
        };
      
      // Send the goal to Nav2
      auto future = nav2_action_client_->async_send_goal(nav2_goal, send_goal_options);
      
      // Wait for the action to complete (this is handled by callbacks)
      // The execution thread will continue until Nav2 completes or is cancelled
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in GNSS navigation: %s", e.what());
      
      feedback->bt_status = 0;  // NOT RUNNING
      goal_handle->publish_feedback(feedback);
      
      result->navigation_result = 0;  // NOT SUCCESS
      goal_handle->abort(result);
    }
  }

  // Helper function to convert GPS to map coordinates
  geometry_msgs::msg::PoseStamped convert_gps_to_map_pose(double latitude, double longitude)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    
    // TODO: Implement proper GPS to map coordinate conversion
    // This depends on your map's origin and coordinate system
    // For now, using a placeholder conversion
    
    // Example: Simple UTM conversion or local coordinate transformation
    pose.pose.position.x = longitude * 111320.0;  // Rough conversion - replace with proper transform
    pose.pose.position.y = latitude * 110540.0;   // Rough conversion - replace with proper transform
    pose.pose.position.z = 0.0;
    
    // Set orientation to face forward (you might want to calculate based on path)
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);  // Roll, Pitch, Yaw
    pose.pose.orientation = tf2::toMsg(quat);
    
    return pose;
  }

  std::string get_package_share_directory(const std::string& package_name)
  {
    // You'll need to implement this or use ament_index_python equivalent for C++
    // For now, return a placeholder path
    return "/path/to/your/package/share";  // Replace with actual path resolution
  }

  // Store current Nav2 goal handle for cancellation
  std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> current_nav2_goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GnssNavigationActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}