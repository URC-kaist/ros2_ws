#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <mr2_action_interface/action/gnss_only.hpp>
#include "mr2_rover_auto/gps_to_map.hpp"

using GnssOnly = mr2_action_interface::action::GnssOnly;
using NavToPose = nav2_msgs::action::NavigateToPose;

class GnssOnlyServer : public rclcpp::Node {
public:
  GnssOnlyServer() : Node("gnss_only_server"), gps_conv_(this) {
    nav_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
    action_server_ = rclcpp_action::create_server<GnssOnly>(
      this, "gnss_only",
      std::bind(&GnssOnlyServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GnssOnlyServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&GnssOnlyServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<GnssOnly>::SharedPtr action_server_;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;
  GpsConverter gps_conv_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GnssOnly::Goal> goal)
  {
    if (!std::isfinite(goal->target_latitude) || !std::isfinite(goal->target_longitude))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GnssOnly>>)
  { return rclcpp_action::CancelResponse::ACCEPT; }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GnssOnly>> gh) {
    std::thread([this, gh](){
      auto result = std::make_shared<GnssOnly::Result>();
      auto feedback = std::make_shared<GnssOnly::Feedback>();
      feedback->bt_status = 1;  // RUNNING
      gh->publish_feedback(feedback);

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
      nav_goal.behavior_tree = declare_parameter<std::string>(
        "gnss_only_bt_xml", "install/mr2_rover_auto/share/mr2_rover_auto/behavior_trees/gnss_only.xml");

      auto nav_future = nav_client_->async_send_goal(nav_goal,
        rclcpp_action::Client<NavToPose>::SendGoalOptions{
          .goal_response_callback =
            [this](auto){ RCLCPP_INFO(get_logger(), "Nav goal accepted"); },
          .feedback_callback =
            [gh,this](auto, const std::shared_ptr<const NavToPose::Feedback> &){
              auto fb = std::make_shared<GnssOnly::Feedback>(); fb->bt_status = 1; gh->publish_feedback(fb);
            },
          .result_callback = [](auto){} // handled below
        });

      // 3) Monitor until done or cancel
      while (rclcpp::ok()) {
        if (gh->is_canceling()) {
          nav_client_->async_cancel_all_goals();
          result->navigation_result = 0; gh->canceled(result); return;
        }
        auto status = nav_client_->async_get_result(nav_future.get().get()).wait_for(std::chrono::milliseconds(50));
        if (status == std::future_status::ready) {
          auto nav_res = nav_future.get().get_result();
          result->navigation_result = (nav_res->result == NavToPose::Result::SUCCEEDED) ? 1 : 0;
          if (result->navigation_result) gh->succeed(result); else gh->abort(result);
          return;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }
      result->navigation_result = 0; gh->abort(result);
    }).detach();
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssOnlyServer>());
  rclcpp::shutdown();
  return 0;
}
