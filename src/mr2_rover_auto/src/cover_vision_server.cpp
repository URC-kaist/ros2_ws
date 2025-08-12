#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <mr2_action_interface/action/cover_vision.hpp>
#include "mr2_rover_auto/gps_to_map.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

using CoverVision = mr2_action_interface::action::CoverVision;
using NavToPose = nav2_msgs::action::NavigateToPose;

static std::vector<geometry_msgs::msg::PoseStamped>
make_ring_waypoints(const geometry_msgs::msg::PoseStamped &center, double radius, int n,
                    const std::string &frame = "map")
{
  std::vector<geometry_msgs::msg::PoseStamped> pts; pts.reserve(n);
  for (int k=0;k<n;++k){
    double th = 2.0*M_PI*double(k)/double(n);
    geometry_msgs::msg::PoseStamped p; p.header.frame_id = frame; p.header.stamp = center.header.stamp;
    p.pose.position.x = center.pose.position.x + radius*std::cos(th);
    p.pose.position.y = center.pose.position.y + radius*std::sin(th);
    p.pose.position.z = center.pose.position.z;
    p.pose.orientation = center.pose.orientation; // yaw-align later if desired
    pts.push_back(p);
  }
  return pts;
}

class CoverVisionServer : public rclcpp::Node {
public:
  CoverVisionServer() : Node("cover_vision_server"), gps_conv_(this) {
    nav_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
    action_server_ = rclcpp_action::create_server<CoverVision>(
      this, "cover_vision",
      std::bind(&CoverVisionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CoverVisionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CoverVisionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<CoverVision>::SharedPtr action_server_;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;
  GpsConverter gps_conv_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CoverVision::Goal> goal)
  {
    if (!(std::isfinite(goal->target_latitude) && std::isfinite(goal->target_longitude) &&
          std::isfinite(goal->target_radius) && goal->target_radius > 0.0))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CoverVision>>)
  { return rclcpp_action::CancelResponse::ACCEPT; }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CoverVision>> gh) {
    std::thread([this, gh](){
      auto res = std::make_shared<CoverVision::Result>();
      CoverVision::Feedback fb; fb.bt_status = 1;

      // 1) Center pose
      geometry_msgs::msg::PoseStamped center;
      if (!gps_conv_.to_map_pose(gh->get_goal()->target_latitude, gh->get_goal()->target_longitude, center)) {
        res->mission_result = 0; res->waypoints_completed = 0; gh->abort(res); return;
      }

      // 2) Build coverage waypoint set (start with ring-only; you can add lawnmower later)
      auto waypoints = make_ring_waypoints(center, gh->get_goal()->target_radius, /*n=*/12, center.header.frame_id);
      fb.total_waypoints = static_cast<int32_t>(waypoints.size());
      gh->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));

      // 3) Iterate waypoints with Nav2
      if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "Nav2 navigate_to_pose not available");
        res->mission_result = 0; res->waypoints_completed = 0; gh->abort(res); return;
      }

      std::string bt_xml = declare_parameter<std::string>(
        "cover_vision_bt_xml", "install/mr2_rover_auto/share/mr2_rover_auto/behavior_trees/cover_vision.xml");

      int completed = 0;
      for (size_t i=0;i<waypoints.size();++i) {
        if (gh->is_canceling()) { res->mission_result = 0; res->waypoints_completed = completed; gh->canceled(res); return; }

        fb.current_waypoint_index = static_cast<int32_t>(i);
        gh->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));

        NavToPose::Goal g; g.pose = waypoints[i]; g.behavior_tree = bt_xml;
        auto send = nav_client_->async_send_goal(g,
          rclcpp_action::Client<NavToPose>::SendGoalOptions{
            .goal_response_callback = [this](auto){},
            .feedback_callback = [this,gh](auto, auto){ CoverVision::Feedback f; f.bt_status=1; gh->publish_feedback(std::make_shared<CoverVision::Feedback>(f)); },
            .result_callback = [](auto){}
          });

        // Wait for result (you can add a per-waypoint timeout here if desired)
        auto goal_handle = send.get();
        auto result_future = nav_client_->async_get_result(goal_handle);
        result_future.wait();
        auto nav_result = result_future.get();
        if (nav_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          completed++;
        } else {
          // Keep going or break depending on policy; here we continue
        }

        // TODO (future): check an object-detection flag to early-exit and navigate to object pose
      }

      res->mission_result = 1;  // success-of-mission policy is yours; this assumes completion
      res->waypoints_completed = completed;
      gh->succeed(res);
    }).detach();
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverVisionServer>());
  rclcpp::shutdown();
  return 0;
}
