#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mr2_action_interface/action/cover_vision.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <thread>
#include <chrono>

class CoverVisionActionServer : public rclcpp::Node
{
public:
  using CoverVision = mr2_action_interface::action::CoverVision;
  using GoalHandleCover = rclcpp_action::ServerGoalHandle<CoverVision>;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;

  explicit CoverVisionActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cover_vision_action_server", options)
  {
    // Create the action server
    this->action_server_ = rclcpp_action::create_server<CoverVision>(
      this,
      "cover_vision_mission",
      std::bind(&CoverVisionActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CoverVisionActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CoverVisionActionServer::handle_accepted, this, std::placeholders::_1));

    // Create action client for Nav2's NavigateThroughPoses (uses CoverVision.xml BT)
    this->nav2_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");

    // Publisher for emergency stop
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Parameters for coverage pattern
    this->declare_parameter("coverage_spacing", 5.0);  // meters between coverage lines
    this->declare_parameter("coverage_area_size", 20.0);  // square area size in meters

    RCLCPP_INFO(this->get_logger(), "🎯 CoverVision Action Server initialized");
    RCLCPP_INFO(this->get_logger(), "   Generates waypoints for coverage missions");
  }

private:
  rclcpp_action::Server<CoverVision>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav2_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CoverVision::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "🎯 Received Coverage mission goal");
    RCLCPP_INFO(this->get_logger(), "   Target: lat=%.6f, lon=%.6f", 
                goal->target_latitude, goal->target_longitude);
    
    // Basic validation
    if (std::abs(goal->target_latitude) > 90.0 || std::abs(goal->target_longitude) > 180.0) {
      RCLCPP_ERROR(this->get_logger(), "❌ Invalid GPS coordinates");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Check if Nav2 is available
    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "❌ Nav2 NavigateThroughPoses server not available");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Coverage goal accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCover> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "🛑 Coverage mission cancel requested");
    
    // Send stop commands
    auto stop_cmd = geometry_msgs::msg::Twist();
    for (int i = 0; i < 5; ++i) {
      cmd_vel_pub_->publish(stop_cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // Cancel the underlying Nav2 action if active
    if (current_nav2_goal_handle_) {
      auto cancel_result = nav2_client_->async_cancel_goal(current_nav2_goal_handle_);
      RCLCPP_INFO(this->get_logger(), "🛑 Cancelled Nav2 navigation goal");
    }
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCover> goal_handle)
  {
    // Execute in separate thread
    std::thread{std::bind(&CoverVisionActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCover> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CoverVision::Feedback>();
    auto result = std::make_shared<CoverVision::Result>();
    
    RCLCPP_INFO(this->get_logger(), "🚀 Starting Coverage mission execution");
    
    try {
      // Step 1: Generate waypoints from the target GPS coordinate
      auto waypoints = generate_coverage_waypoints(goal->target_latitude, goal->target_longitude);
      
      RCLCPP_INFO(this->get_logger(), "📍 Generated %zu waypoints for coverage", waypoints.size());
      
      // Update feedback with waypoint info
      feedback->bt_status = 1;  // RUNNING
      feedback->total_waypoints = waypoints.size();
      feedback->current_waypoint_index = 0;
      goal_handle->publish_feedback(feedback);
      
      // Step 2: Create Nav2 goal with generated waypoints and CoverVision.xml BT
      auto nav2_goal = NavigateThroughPoses::Goal();
      nav2_goal.poses = waypoints;
      nav2_goal.behavior_tree = ament_index_cpp::get_package_share_directory("mr2_rover_auto") + "/behavior_trees/CoverVision.xml";  // Your custom BT
      
      // Step 3: Set up Nav2 action callbacks
      auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
      
      send_goal_options.goal_response_callback = 
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateThroughPoses>> nav2_handle) {
          if (!nav2_handle) {
            RCLCPP_ERROR(this->get_logger(), "❌ Nav2 goal was rejected");
          } else {
            RCLCPP_INFO(this->get_logger(), "✅ Nav2 accepted coverage waypoints");
            current_nav2_goal_handle_ = nav2_handle;
          }
        };
      
      send_goal_options.feedback_callback =
        [this, goal_handle, feedback](
          std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateThroughPoses>>,
          const std::shared_ptr<const NavigateThroughPoses::Feedback> nav2_feedback) {
          
          // Update our feedback with Nav2's progress
          feedback->bt_status = 1;  // RUNNING
          // feedback->current_waypoint_index = nav2_feedback->current_waypoint;
          goal_handle->publish_feedback(feedback);
          
          // RCLCPP_INFO(this->get_logger(), "📊 Coverage progress: waypoint %d/%zu", 
          //             nav2_feedback->current_waypoint + 1, feedback->total_waypoints);
        };
      
      send_goal_options.result_callback =
        [this, goal_handle, feedback, result](
          const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult & nav2_result) {
          
          // Coverage mission finished
          feedback->bt_status = 0;  // NOT RUNNING
          goal_handle->publish_feedback(feedback);
          
          if (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "🎉 Coverage mission succeeded");
            result->mission_result = 1;  // SUCCESS
            result->waypoints_completed = feedback->total_waypoints;
            goal_handle->succeed(result);
          } else if (nav2_result.code == rclcpp_action::ResultCode::CANCELED) {
            RCLCPP_INFO(this->get_logger(), "🛑 Coverage mission was cancelled");
            result->mission_result = 0;  // NOT SUCCESS
            result->waypoints_completed = feedback->current_waypoint_index;
            goal_handle->canceled(result);
          } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Coverage mission failed");
            result->mission_result = 0;  // NOT SUCCESS
            result->waypoints_completed = feedback->current_waypoint_index;
            goal_handle->abort(result);
          }
          
          current_nav2_goal_handle_.reset();
        };
      
      // Send the goal to Nav2
      auto future = nav2_client_->async_send_goal(nav2_goal, send_goal_options);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "💥 Exception in coverage mission: %s", e.what());
      
      feedback->bt_status = 0;  // NOT RUNNING
      goal_handle->publish_feedback(feedback);
      
      result->mission_result = 0;  // NOT SUCCESS
      result->waypoints_completed = 0;
      goal_handle->abort(result);
    }
  }

  // Generate coverage waypoints in a lawn mower pattern
  std::vector<geometry_msgs::msg::PoseStamped> generate_coverage_waypoints(double center_lat, double center_lon)
  {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    
    // Get parameters
    double spacing = this->get_parameter("coverage_spacing").as_double();
    double area_size = this->get_parameter("coverage_area_size").as_double();
    
    // Convert center GPS to map coordinates (placeholder - implement your conversion)
    double center_x = center_lon * 111320.0;  // Rough conversion
    double center_y = center_lat * 110540.0;  // Rough conversion
    
    // Generate lawn mower pattern
    double half_size = area_size / 2.0;
    bool going_right = true;
    
    for (double y = center_y - half_size; y <= center_y + half_size; y += spacing) {
      if (going_right) {
        // Left to right
        for (double x = center_x - half_size; x <= center_x + half_size; x += spacing) {
          waypoints.push_back(create_waypoint(x, y, 0.0));  // 0 degrees heading
        }
      } else {
        // Right to left
        for (double x = center_x + half_size; x >= center_x - half_size; x -= spacing) {
          waypoints.push_back(create_waypoint(x, y, 3.14159));  // 180 degrees heading
        }
      }
      going_right = !going_right;  // Alternate direction
    }
    
    RCLCPP_INFO(this->get_logger(), "🗺️  Generated lawn mower pattern: %zu waypoints", waypoints.size());
    return waypoints;
  }

  geometry_msgs::msg::PoseStamped create_waypoint(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = "map";
    waypoint.header.stamp = this->now();
    
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.position.z = 0.0;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    waypoint.pose.orientation = tf2::toMsg(quat);
    
    return waypoint;
  }

  std::string get_bt_xml_path(const std::string& bt_filename)
  {
    // Return path to your BT XML file
    // You might want to use ament_index_cpp to find package path
    return "/path/to/your/package/behavior_trees/" + bt_filename;  // Placeholder
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateThroughPoses>> current_nav2_goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoverVisionActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}