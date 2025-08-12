#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mr2_action_interface/action/cover_vision.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <vector>
#include <thread>
#include <chrono>

class CoverVisionActionServer : public rclcpp::Node
{
public:
  using CoverVision = mr2_action_interface::action::CoverVision;
  using GoalHandleCover = rclcpp_action::ServerGoalHandle<CoverVision>;

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

    // Publisher for emergency stop
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Parameters for coverage pattern
    this->declare_parameter("coverage_spacing", 5.0);
    this->declare_parameter("coverage_area_size", 20.0);

    // Initialize BehaviorTree factory
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();
    
    // Register Nav2 BT nodes (you'll need to include nav2_behavior_tree plugins)
    // This registers all the standard Nav2 BT nodes like NavigateToPose, etc.
    // You might need to manually register the nodes you're using
    nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
      ::registerFromPlugin(factory_, "NavigateToPose");
    nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateThroughPoses>
      ::registerFromPlugin(factory_, "NavigateThroughPoses");
    
    RCLCPP_INFO(this->get_logger(), "🎯 CoverVision Action Server initialized");
  }

private:
  rclcpp_action::Server<CoverVision>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  std::shared_ptr<BT::Tree> current_tree_;

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
    
    // Stop the behavior tree
    if (current_tree_) {
      // Halt the tree execution
      current_tree_->haltTree();
    }
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCover> goal_handle)
  {
    std::thread{std::bind(&CoverVisionActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCover> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CoverVision::Feedback>();
    auto result = std::make_shared<CoverVision::Result>();
    
    RCLCPP_INFO(this->get_logger(), "🚀 Starting Coverage mission execution");
    
    try {
      // Step 1: Generate waypoints
      auto waypoints = generate_coverage_waypoints(goal->target_latitude, goal->target_longitude);
      RCLCPP_INFO(this->get_logger(), "📍 Generated %zu waypoints for coverage", waypoints.size());
      
      // Step 2: Set up blackboard with mission data
      BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
      
      // Convert target GPS to pose for initial approach
      auto target_pose = convert_gps_to_map_pose(goal->target_latitude, goal->target_longitude);
      blackboard->set<geometry_msgs::msg::PoseStamped>("goal", target_pose);
      blackboard->set<std::vector<geometry_msgs::msg::PoseStamped>>("goals", waypoints);
      
      // Set server names (these should match your Nav2 configuration)
      blackboard->set<std::string>("navtopose_server", "navigate_to_pose");
      blackboard->set<std::string>("navtoposes_server", "navigate_through_poses");
      blackboard->set<std::string>("server_timeout", "10");
      
      // You might want to set different BT files for sub-actions, or use default
      blackboard->set<std::string>("navtopose_bt", "");  // Empty = use default
      blackboard->set<std::string>("navtoposes_bt", ""); // Empty = use default
      
      // Step 3: Load and execute your behavior tree
      std::string bt_xml_file = ament_index_cpp::get_package_share_directory("mr2_rover_auto") + 
                               "/behavior_trees/CoverVision.xml";
      
      current_tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromFile(bt_xml_file, blackboard));
      
      // Step 4: Execute the behavior tree
      feedback->bt_status = 1;  // RUNNING
      feedback->total_waypoints = waypoints.size();
      goal_handle->publish_feedback(feedback);
      
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      
      while (status == BT::NodeStatus::RUNNING) {
        status = current_tree_->tickRootWhileRunning(std::chrono::milliseconds(10));
        
        // Check if goal was cancelled
        if (goal_handle->is_canceling()) {
          current_tree_->haltTree();
          result->mission_result = 0;
          result->waypoints_completed = 0;
          goal_handle->canceled(result);
          return;
        }
        
        // Update feedback
        feedback->bt_status = (status == BT::NodeStatus::RUNNING) ? 1 : 0;
        goal_handle->publish_feedback(feedback);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      
      // Step 5: Handle completion
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "🎉 Coverage mission succeeded");
        result->mission_result = 1;
        result->waypoints_completed = waypoints.size();
        goal_handle->succeed(result);
      } else {
        RCLCPP_ERROR(this->get_logger(), "❌ Coverage mission failed");
        result->mission_result = 0;
        result->waypoints_completed = 0;
        goal_handle->abort(result);
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "💥 Exception in coverage mission: %s", e.what());
      
      feedback->bt_status = 0;
      goal_handle->publish_feedback(feedback);
      
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
    }
    
    current_tree_.reset();
  }

  // Generate coverage waypoints in a lawn mower pattern
  std::vector<geometry_msgs::msg::PoseStamped> generate_coverage_waypoints(double center_lat, double center_lon)
  {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    
    double spacing = this->get_parameter("coverage_spacing").as_double();
    double area_size = this->get_parameter("coverage_area_size").as_double();
    
    // Convert center GPS to map coordinates
    double center_x = center_lon * 111320.0;  // Replace with proper conversion
    double center_y = center_lat * 110540.0;  // Replace with proper conversion
    
    // Generate lawn mower pattern
    double half_size = area_size / 2.0;
    bool going_right = true;
    
    for (double y = center_y - half_size; y <= center_y + half_size; y += spacing) {
      if (going_right) {
        for (double x = center_x - half_size; x <= center_x + half_size; x += spacing) {
          waypoints.push_back(create_waypoint(x, y, 0.0));
        }
      } else {
        for (double x = center_x + half_size; x >= center_x - half_size; x -= spacing) {
          waypoints.push_back(create_waypoint(x, y, 3.14159));
        }
      }
      going_right = !going_right;
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

  geometry_msgs::msg::PoseStamped convert_gps_to_map_pose(double latitude, double longitude)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    
    // TODO: Implement proper GPS to map coordinate conversion
    pose.pose.position.x = longitude * 111320.0;  // Replace with proper transform
    pose.pose.position.y = latitude * 110540.0;   // Replace with proper transform
    pose.pose.position.z = 0.0;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(quat);
    
    return pose;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CoverVisionActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}