// gnss_only_server.cpp
//
// Skeleton: GnssOnly action server that delegates to Nav2's NavigateToPose.
// ROS 2 Humble style. Fill in the WGS84->map(ENU) conversion where marked.
//
// Adjust the include for your action package path (e.g., <my_pkg/action/gnss_only.hpp>).

#include <memory>
#include <thread>
#include <chrono>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mr2_action_interface/action/gnss_only.hpp"
#include "mr2_rover_auto/gps_to_map.hpp"

using namespace std::chrono_literals;

class GnssOnlyServer : public rclcpp::Node
{
public:
  using GnssOnly      = mr2_action_interface::action::GnssOnly;
  using GoalHandleGO  = rclcpp_action::ServerGoalHandle<GnssOnly>;
  using NavToPose     = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavToPose>;

  GnssOnlyServer()
  : Node("gnss_only_server"), gps_conv_(this)
  {
    nav_action_client_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");

    action_server_ = rclcpp_action::create_server<GnssOnly>(
      this,
      "gnss_only",
      std::bind(&GnssOnlyServer::handle_goal,      this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GnssOnlyServer::handle_cancel,    this, std::placeholders::_1),
      std::bind(&GnssOnlyServer::handle_accepted,  this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "GnssOnly action server is up. Waiting for goals...");
  }

private:
  // ----- Action server handlers -----
  GpsConverter gps_conv_;
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & /*uuid*/,
      std::shared_ptr<const GnssOnly::Goal> goal)
  {
    // Basic validation on lat/lon; you may add stricter checks.
    if (!std::isfinite(goal->target_latitude) || !std::isfinite(goal->target_longitude)) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: non-finite lat/lon");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGO> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel GnssOnly goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGO> goal_handle)
  {
    // Execute on a new thread to avoid blocking the executor.
    std::thread{std::bind(&GnssOnlyServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  // ----- Execution: delegate to Nav2 NavigateToPose -----

  void execute(const std::shared_ptr<GoalHandleGO> goal_handle)
  {
    const auto goal     = goal_handle->get_goal();
    auto       feedback = std::make_shared<GnssOnly::Feedback>();
    auto       result   = std::make_shared<GnssOnly::Result>();

    RCLCPP_INFO(get_logger(),
                "GnssOnly: received lat=%.9f, lon=%.9f",
                goal->target_latitude, goal->target_longitude);

    // 1) Prepare the NavigateToPose goal
    NavToPose::Goal nav_goal;
    // - Convert (goal->target_latitude, goal->target_longitude) to map-frame PoseStamped.
    geometry_msgs::msg::PoseStamped target;
    if (!gps_conv_.to_map_pose(goal_handle->get_goal()->target_latitude,
                                goal_handle->get_goal()->target_longitude, target)) {
    result->navigation_result = 0; goal_handle->abort(result); return;
    }
    nav_goal.pose = target; // Use the converted map pose directly

    // 2) Ensure Nav2 action server is available
    if (!nav_action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Nav2 NavigateToPose action server not available.");
      result->navigation_result = 0;
      goal_handle->abort(result);
      return;
    }

    // 3) Send goal to Nav2 with feedback + result callbacks
    auto send_goal_opts = rclcpp_action::Client<NavToPose>::SendGoalOptions();

    send_goal_opts.goal_response_callback =
      [this](NavGoalHandle::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Nav2 NavigateToPose goal was rejected.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Nav2 NavigateToPose goal accepted.");
        }
      };

    send_goal_opts.feedback_callback =
      [this, goal_handle](NavGoalHandle::SharedPtr /*unused*/,
                          const std::shared_ptr<const NavToPose::Feedback> & fb)
      {
        // Stream BT running status outward.
        auto out = std::make_shared<GnssOnly::Feedback>();
        out->bt_status = 1; // RUNNING

        // (Optional) You could inspect fb->distance_remaining, fb->current_pose, etc.
        // and add your own logging/telemetry here.
        RCLCPP_DEBUG(this->get_logger(),
                     "Nav2 feedback: dist_remaining=%.2f, recoveries=%u, nav_time=%.2f",
                     fb->distance_remaining,
                     fb->number_of_recoveries,
                     fb->navigation_time.sec + fb->navigation_time.nanosec * 1e-9);

        goal_handle->publish_feedback(out);
      };

    send_goal_opts.result_callback =
      [this, goal_handle](const NavGoalHandle::WrappedResult & nav_result)
      {
        auto res = std::make_shared<GnssOnly::Result>();

        switch (nav_result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Nav2 NavigateToPose succeeded.");
            res->navigation_result = 1; // SUCCESS
            goal_handle->succeed(res);
            break;

          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Nav2 NavigateToPose aborted.");
            res->navigation_result = 0;
            goal_handle->abort(res);
            break;

          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Nav2 NavigateToPose canceled.");
            res->navigation_result = 0;
            goal_handle->canceled(res);
            break;

          default:
            RCLCPP_ERROR(this->get_logger(), "Nav2 NavigateToPose unknown result code.");
            res->navigation_result = 0;
            goal_handle->abort(res);
            break;
        }
      };

    // Publish initial feedback to indicate BT is starting
    feedback->bt_status = 1; // RUNNING
    goal_handle->publish_feedback(feedback);

    // 4) Handle cancellation requests from *our* action while Nav2 is running
    //    We send the goal, then wait on the future in this thread.
    auto future_handle = nav_action_client_->async_send_goal(nav_goal, send_goal_opts);

    // Poll for cancel from our action server
    // (You can refine with condition variables or events; this is a simple loop.)
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(get_logger(), "Forwarding cancel to Nav2 NavigateToPose...");
        if (auto h = future_handle.get()) {
          nav_action_client_->async_cancel_goal(h);
        }
        // The result callback will deliver canceled->canceled() to our client.
        return;
      }
      // Small sleep to yield; actual feedback/result handled by callbacks.
      std::this_thread::sleep_for(50ms);
      // You can also break when Nav2 goal handle is done; kept minimal here.
    }
  }

  // ----- Members -----
  rclcpp_action::Server<GnssOnly>::SharedPtr action_server_;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssOnlyServer>());
  rclcpp::shutdown();
  return 0;
}
