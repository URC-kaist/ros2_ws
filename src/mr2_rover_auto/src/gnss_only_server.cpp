#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <mr2_action_interface/action/gnss_only.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include "mr2_rover_auto/gps_to_map.hpp"

using GnssOnly = mr2_action_interface::action::GnssOnly;
using NavToPose = nav2_msgs::action::NavigateToPose;
using GH_NavToPose = rclcpp_action::ClientGoalHandle<NavToPose>;

class GnssOnlyServer : public rclcpp::Node {
public:
  GnssOnlyServer() : rclcpp::Node("gnss_only_server"), gps_conv_(this) {
    nav_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
    server_ = rclcpp_action::create_server<GnssOnly>(
      this, "gnss_only",
      std::bind(&GnssOnlyServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GnssOnlyServer::on_cancel, this, std::placeholders::_1),
      std::bind(&GnssOnlyServer::on_accept, this, std::placeholders::_1));
    bt_path_ = this->declare_parameter<std::string>(
      "gnss_only_bt_xml",
      "install/mr2_rover_auto/share/mr2_rover_auto/behavior_trees/gnss_only.xml");
  }

private:
  rclcpp_action::Server<GnssOnly>::SharedPtr server_;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;
  GpsConverter gps_conv_;
  std::string bt_path_;

  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID &,
                                      std::shared_ptr<const GnssOnly::Goal> g) {
    if (!std::isfinite(g->target_latitude) || !std::isfinite(g->target_longitude))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GnssOnly>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accept(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GnssOnly>> gh) {
    std::thread([this, gh]() {
      auto result = std::make_shared<GnssOnly::Result>();
      auto fb = std::make_shared<GnssOnly::Feedback>();
      fb->bt_status = 1; gh->publish_feedback(fb);

      // 1) Build map pose
      geometry_msgs::msg::PoseStamped target;
      if (!gps_conv_.to_map_pose(gh->get_goal()->target_latitude,
                                 gh->get_goal()->target_longitude, target)) {
        result->navigation_result = 0; gh->abort(result); return;
      }

      // 2) Send Nav2 goal with BT override
      if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "Nav2 navigate_to_pose not available");
        result->navigation_result = 0; gh->abort(result); return;
      }

      NavToPose::Goal nav_goal;
      nav_goal.pose = target;
      nav_goal.behavior_tree = bt_path_;

      // 3)
      rclcpp_action::Client<NavToPose>::SendGoalOptions opts;
      opts.goal_response_callback =
        [this](std::shared_ptr<GH_NavToPose> gh_nav) {
          if (!gh_nav) RCLCPP_ERROR(this->get_logger(), "Nav goal rejected");
        };
      opts.feedback_callback =
        [this, gh](std::shared_ptr<GH_NavToPose>, const std::shared_ptr<const NavToPose::Feedback> &) {
          auto f = std::make_shared<GnssOnly::Feedback>(); f->bt_status = 1; gh->publish_feedback(f);
        };
      // We'll wait on result explicitly; we still set a no-op callback to satisfy API.
      opts.result_callback =
        [](const GH_NavToPose::WrappedResult &) {};

      auto goal_handle_future = nav_client_->async_send_goal(nav_goal, opts);

      // Wait for GH and monitor cancel/result
      std::shared_ptr<GH_NavToPose> gh_nav;
      if (goal_handle_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        gh_nav = goal_handle_future.get();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Nav goal_handle timeout");
        result->navigation_result = 0; gh->abort(result); return;
      }
      if (!gh_nav) { result->navigation_result = 0; gh->abort(result); return; }

      auto result_future = nav_client_->async_get_result(gh_nav);

      while (rclcpp::ok()) {
        if (gh->is_canceling()) {
          nav_client_->async_cancel_goal(gh_nav);
          result->navigation_result = 0; gh->canceled(result); return;
        }
        if (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
          auto nav_wrapped = result_future.get();
          const bool ok = (nav_wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
          result->navigation_result = ok ? 1 : 0;
          if (ok) gh->succeed(result); else gh->abort(result);
          return;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
      result->navigation_result = 0; gh->abort(result);
    }).detach();
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssOnlyServer>());
  rclcpp::shutdown();
  return 0;
}
