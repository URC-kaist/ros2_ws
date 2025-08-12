#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <mr2_action_interface/action/cover_vision.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <cmath>
#include "mr2_rover_auto/gps_to_map.hpp"

using CoverVision = mr2_action_interface::action::CoverVision;
using NavToPose = nav2_msgs::action::NavigateToPose;
using GH_NavToPose = rclcpp_action::ClientGoalHandle<NavToPose>;

static std::vector<geometry_msgs::msg::PoseStamped>
make_ring(const geometry_msgs::msg::PoseStamped & center, double radius, int n)
{
  std::vector<geometry_msgs::msg::PoseStamped> v; v.reserve(n);
  for (int k = 0; k < n; ++k) {
    const double th = 2.0 * M_PI * (static_cast<double>(k) / static_cast<double>(n));
    geometry_msgs::msg::PoseStamped p;
    p.header = center.header;
    p.pose.position.x = center.pose.position.x + radius * std::cos(th);
    p.pose.position.y = center.pose.position.y + radius * std::sin(th);
    p.pose.position.z = center.pose.position.z;
    p.pose.orientation = center.pose.orientation; // yaw-align later if desired
    v.push_back(p);
  }
  return v;
}

class CoverVisionServer : public rclcpp::Node {
public:
  CoverVisionServer() : rclcpp::Node("cover_vision_server"), gps_conv_(this) {
    nav_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
    server_ = rclcpp_action::create_server<CoverVision>(
      this, "cover_vision",
      std::bind(&CoverVisionServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CoverVisionServer::on_cancel, this, std::placeholders::_1),
      std::bind(&CoverVisionServer::on_accept, this, std::placeholders::_1));
    bt_path_ = this->declare_parameter<std::string>(
      "cover_vision_bt_xml",
      "install/mr2_rover_auto/share/mr2_rover_auto/behavior_trees/cover_vision.xml");
  }

private:
  rclcpp_action::Server<CoverVision>::SharedPtr server_;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;
  GpsConverter gps_conv_;
  std::string bt_path_;

  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &,
                                      std::shared_ptr<const CoverVision::Goal> g) {
    if (!(std::isfinite(g->target_latitude) && std::isfinite(g->target_longitude) &&
          std::isfinite(g->target_radius) && g->target_radius > 0.0)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CoverVision>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accept(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CoverVision>> gh) {
    std::thread([this, gh]() {
      auto res = std::make_shared<CoverVision::Result>();
      CoverVision::Feedback fb;
      fb.bt_status = 1; fb.total_waypoints = 0; fb.current_waypoint_index = -1;
      gh->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));

      // 1) Center pose
      geometry_msgs::msg::PoseStamped center;
      if (!gps_conv_.to_map_pose(gh->get_goal()->target_latitude,
                                 gh->get_goal()->target_longitude, center)) {
        res->mission_result = 0; res->waypoints_completed = 0; gh->abort(res); return;
      }
      // 2) Build coverage waypoint set (start with ring-only; you can add lawnmower later)
      auto waypoints = make_ring(center, gh->get_goal()->target_radius, 12);
      fb.total_waypoints = static_cast<int32_t>(waypoints.size());
      gh->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));

      // 3) Iterate waypoints with Nav2
      if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "navigate_to_pose server unavailable");
        res->mission_result = 0; res->waypoints_completed = 0; gh->abort(res); return;
      }

      int completed = 0;
      for (size_t i = 0; i < waypoints.size(); ++i) {
        if (gh->is_canceling()) {
          res->mission_result = 0; res->waypoints_completed = completed; gh->canceled(res); return;
        }

        fb.current_waypoint_index = static_cast<int32_t>(i);
        gh->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));

        NavToPose::Goal g; g.pose = waypoints[i]; g.behavior_tree = bt_path_;

        rclcpp_action::Client<NavToPose>::SendGoalOptions opts;
        opts.goal_response_callback =
          [this](std::shared_ptr<GH_NavToPose> gh_nav) {
            if (!gh_nav) RCLCPP_ERROR(this->get_logger(), "Nav goal rejected");
          };
        opts.feedback_callback =
          [this, gh](std::shared_ptr<GH_NavToPose>, const std::shared_ptr<const NavToPose::Feedback> &) {
            CoverVision::Feedback f; f.bt_status = 1; gh->publish_feedback(std::make_shared<CoverVision::Feedback>(f));
          };
        opts.result_callback =
          [](const GH_NavToPose::WrappedResult &) {};

        auto gh_future = nav_client_->async_send_goal(g, opts);
        if (gh_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
          RCLCPP_ERROR(this->get_logger(), "Nav goal_handle timeout");
          continue;
        }
        auto gh_nav = gh_future.get();
        if (!gh_nav) { continue; }

        auto res_future = nav_client_->async_get_result(gh_nav);
        while (rclcpp::ok()) {
          if (gh->is_canceling()) {
            nav_client_->async_cancel_goal(gh_nav);
            res->mission_result = 0; res->waypoints_completed = completed; gh->canceled(res); return;
          }
          if (res_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
            auto wr = res_future.get();
            if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) ++completed;
            break;
          }
          rclcpp::sleep_for(std::chrono::milliseconds(50));
        }

        // TODO: early-exit switch to object pose when your OpenCV node asserts a detection
      }

      res->mission_result = 1;  // success-of-mission policy is yours; this assumes completion
      res->waypoints_completed = completed;
      gh->succeed(res);
    }).detach();
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverVisionServer>());
  rclcpp::shutdown();
  return 0;
}
