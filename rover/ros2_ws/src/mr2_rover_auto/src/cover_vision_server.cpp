#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mr2_action_interface/action/cover_vision.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mr2_rover_auto/gps_to_map.hpp"

using namespace std::chrono_literals;

using CoverVision = mr2_action_interface::action::CoverVision;
using NavToPose = nav2_msgs::action::NavigateToPose;
using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;

static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

static void assign_tangent_orientations(std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (poses.size() < 2) {
    return;
  }
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    const double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    poses[i].pose.orientation = yaw_to_quat(std::atan2(dy, dx));
  }
  poses.back().pose.orientation = poses[poses.size() - 2].pose.orientation;
}

static std::vector<geometry_msgs::msg::PoseStamped> make_archimedean_spiral(
  const geometry_msgs::msg::PoseStamped & center,
  double pitch_m,
  double max_radius_m,
  double point_spacing_m,
  size_t max_points)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  if (!(std::isfinite(pitch_m) && pitch_m > 0.0 &&
        std::isfinite(max_radius_m) && max_radius_m >= 0.0 &&
        std::isfinite(point_spacing_m) && point_spacing_m > 0.0)) {
    return poses;
  }

  // r = a * theta where a = pitch / (2*pi). pitch is the radial spacing after 2*pi.
  const double a = pitch_m / (2.0 * M_PI);
  const double theta_max = max_radius_m / a;

  poses.reserve(std::min(max_points, static_cast<size_t>(std::ceil(theta_max * 10.0)) + 2));

  geometry_msgs::msg::PoseStamped p0 = center;
  p0.pose.position.z = 0.0;
  poses.push_back(p0);

  double theta = 0.0;
  double r = 0.0;
  while (poses.size() < max_points) {
    const double denom = std::sqrt(r * r + a * a);
    const double dtheta = point_spacing_m / std::max(denom, 1e-6);
    theta += dtheta;
    if (theta > theta_max) {
      break;
    }

    r = a * theta;
    geometry_msgs::msg::PoseStamped p = center;
    p.pose.position.x = center.pose.position.x + r * std::cos(theta);
    p.pose.position.y = center.pose.position.y + r * std::sin(theta);
    p.pose.position.z = 0.0;
    p.pose.orientation.w = 1.0;
    poses.push_back(p);
  }

  assign_tangent_orientations(poses);
  return poses;
}

struct DetectionState
{
  std::mutex mutex;
  bool detected{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped pose_map{};
};

class CoverVisionServer : public rclcpp::Node
{
public:
  CoverVisionServer()
  : rclcpp::Node("cover_vision_server"),
    gps_conv_(this)
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    coverage_path_topic_ =
      this->declare_parameter<std::string>("coverage_path_topic", "cover_vision/coverage_path");

    nav_action_name_ = this->declare_parameter<std::string>("navigate_action_name", "navigate_to_pose");
    nav_through_action_name_ =
      this->declare_parameter<std::string>("navigate_through_poses_action_name", "navigate_through_poses");

    spiral_pitch_m_ = this->declare_parameter<double>("spiral_pitch_m", 3.0);
    spiral_point_spacing_m_ = this->declare_parameter<double>("spiral_point_spacing_m", 2.4);
    spiral_max_points_ = static_cast<size_t>(this->declare_parameter<int>("spiral_max_points", 5000));

    detection_stale_sec_ = this->declare_parameter<double>("detection_stale_sec", 0.75);
    detection_pose_topic_ =
      this->declare_parameter<std::string>("detection_pose_topic", "cover_vision/object_pose");

    nav_client_ = rclcpp_action::create_client<NavToPose>(this, nav_action_name_);
    nav_through_client_ = rclcpp_action::create_client<NavThroughPoses>(this, nav_through_action_name_);

    coverage_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      coverage_path_topic_,
      rclcpp::QoS(1).transient_local().reliable());

    server_ = rclcpp_action::create_server<CoverVision>(
      this,
      "cover_vision",
      std::bind(&CoverVisionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CoverVisionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CoverVisionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CoverVision action server is up. Waiting for goals...");
  }

private:
  using GoalHandleCV = rclcpp_action::ServerGoalHandle<CoverVision>;

  enum DetectionMethod : uint8_t {
    DET_NONE = 0,
    DET_ARUCO = 1,
    DET_YOLO = 2
  };

  rclcpp_action::Server<CoverVision>::SharedPtr server_;
  rclcpp_action::Client<NavToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<NavThroughPoses>::SharedPtr nav_through_client_;
  GpsConverter gps_conv_;

  std::string map_frame_;
  std::string coverage_path_topic_;
  std::string nav_action_name_;
  std::string nav_through_action_name_;
  double spiral_pitch_m_{3.0};
  double spiral_point_spacing_m_{2.4};
  size_t spiral_max_points_{5000};
  double detection_stale_sec_{0.75};
  std::string detection_pose_topic_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr coverage_path_pub_;

  static bool is_fresh(const rclcpp::Time & stamp, const rclcpp::Time & now, double max_age_sec)
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    const double age = (now - stamp).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= max_age_sec;
  }

  bool try_get_detection_map_pose(
    const std::shared_ptr<DetectionState> & det,
    const rclcpp::Time & now,
    geometry_msgs::msg::PoseStamped & out_pose_map)
  {
    std::lock_guard<std::mutex> lock(det->mutex);
    if (!det->detected) {
      return false;
    }
    if (!is_fresh(det->stamp, now, detection_stale_sec_)) {
      return false;
    }
    out_pose_map = det->pose_map;
    return true;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CoverVision::Goal> goal)
  {
    if (!std::isfinite(goal->target_latitude) || !std::isfinite(goal->target_longitude)) {
      RCLCPP_WARN(get_logger(), "Rejecting CoverVision goal: non-finite lat/lon");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->target_radius) || goal->target_radius < 0.0) {
      RCLCPP_WARN(get_logger(), "Rejecting CoverVision goal: invalid target_radius");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCV>)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel CoverVision goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCV> goal_handle)
  {
    std::thread{std::bind(&CoverVisionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  bool wait_for_action_servers()
  {
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server unavailable");
      return false;
    }
    if (!nav_through_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "navigate_through_poses action server unavailable");
      return false;
    }
    return true;
  }

  rclcpp_action::ResultCode run_navigate_to_pose(
    const std::shared_ptr<GoalHandleCV> & goal_handle,
    const geometry_msgs::msg::PoseStamped & pose_map)
  {
    NavToPose::Goal nav_goal;
    nav_goal.pose = pose_map;

    auto gh_future = nav_client_->async_send_goal(nav_goal);
    if (gh_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose goal handle timeout");
      return rclcpp_action::ResultCode::ABORTED;
    }

    auto nav_gh = gh_future.get();
    if (!nav_gh) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose goal rejected");
      return rclcpp_action::ResultCode::ABORTED;
    }

    auto res_future = nav_client_->async_get_result(nav_gh);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        nav_client_->async_cancel_goal(nav_gh);
        return rclcpp_action::ResultCode::CANCELED;
      }

      if (res_future.wait_for(50ms) == std::future_status::ready) {
        return res_future.get().code;
      }
      rclcpp::sleep_for(50ms);
    }
    return rclcpp_action::ResultCode::ABORTED;
  }

  void execute(const std::shared_ptr<GoalHandleCV> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<CoverVision::Result>();

    auto det = std::make_shared<DetectionState>();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detection_sub;
    if (goal->detection_method != DET_NONE) {
      detection_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        detection_pose_topic_, 10,
        [this, det](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          if (!msg) {
            return;
          }
          if (msg->header.frame_id != map_frame_) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "Ignoring detection pose in frame '%s' (expected '%s')",
              msg->header.frame_id.c_str(), map_frame_.c_str());
            return;
          }

          geometry_msgs::msg::PoseStamped pose_map = *msg;
          pose_map.pose.position.z = 0.0;
          pose_map.pose.orientation.w = 1.0;

          std::lock_guard<std::mutex> lock(det->mutex);
          det->detected = true;
          det->stamp = rclcpp::Time(msg->header.stamp);
          det->pose_map = pose_map;
        });
      RCLCPP_INFO(get_logger(), "CoverVision: listening for mission detection pose on %s",
                  detection_pose_topic_.c_str());
    }

    CoverVision::Feedback fb;
    fb.bt_status = 1;
    fb.total_waypoints = 0;
    fb.current_waypoint_index = -1;
    goal_handle->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));

    if (!wait_for_action_servers()) {
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
      return;
    }

    // 1) Convert GNSS center to map pose
    geometry_msgs::msg::PoseStamped center;
    if (!gps_conv_.to_map_pose(goal->target_latitude, goal->target_longitude, center, map_frame_)) {
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
      return;
    }

    // 2) Generate spiral poses in map frame.
    // Publish a Path for dashboard/geo conversion,
    // even though NavigateThroughPoses consumes PoseStamped[] goals.
    const double max_radius_m = std::max(0.0, goal->target_radius) + 0.5 * spiral_pitch_m_;
    const auto poses =
      make_archimedean_spiral(center, spiral_pitch_m_, max_radius_m, spiral_point_spacing_m_, spiral_max_points_);

    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = map_frame_;
    path.poses = poses;
    coverage_path_pub_->publish(path);

    const int32_t total_waypoints = static_cast<int32_t>(poses.size());
    fb.total_waypoints = total_waypoints;
    fb.current_waypoint_index = -1;
    goal_handle->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));

    if (poses.size() < 2) {
      // Treat "no coverage" as success per mission policy.
      result->mission_result = 1;
      result->waypoints_completed = 0;
      goal_handle->succeed(result);
      return;
    }

    // 3) NavigateToPose to center
    const auto nav_center_rc = run_navigate_to_pose(goal_handle, center);
    if (nav_center_rc == rclcpp_action::ResultCode::CANCELED) {
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->canceled(result);
      return;
    }
    if (nav_center_rc != rclcpp_action::ResultCode::SUCCEEDED) {
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
      return;
    }

    geometry_msgs::msg::PoseStamped detected_pose_map;

    // NavigateThroughPoses loop with mission-level detection checks.
    auto remaining = std::make_shared<std::atomic<int32_t>>(-1);
    auto nav_through_goal = NavThroughPoses::Goal{};
    nav_through_goal.poses = poses;

    auto goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
    goal_options.feedback_callback =
      [remaining](rclcpp_action::ClientGoalHandle<NavThroughPoses>::SharedPtr,
                  const std::shared_ptr<const NavThroughPoses::Feedback> feedback)
      {
        if (!feedback) {
          return;
        }
        remaining->store(static_cast<int32_t>(feedback->number_of_poses_remaining),
                         std::memory_order_relaxed);
      };

    auto gh_future = nav_through_client_->async_send_goal(nav_through_goal, goal_options);
    if (gh_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "NavigateThroughPoses goal handle timeout");
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
      return;
    }

    auto ntp_gh = gh_future.get();
    if (!ntp_gh) {
      RCLCPP_ERROR(get_logger(), "NavigateThroughPoses goal rejected");
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
      return;
    }

    auto res_future = nav_through_client_->async_get_result(ntp_gh);
    bool detected = false;
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        nav_through_client_->async_cancel_goal(ntp_gh);
        result->mission_result = 0;
        result->waypoints_completed = 0;
        goal_handle->canceled(result);
        return;
      }

      const int32_t poses_remaining = remaining->load(std::memory_order_relaxed);
      if (poses_remaining >= 0 && total_waypoints > 0) {
        int32_t current_index = total_waypoints - poses_remaining;
        if (poses_remaining <= 0) {
          current_index = total_waypoints - 1;
        }
        current_index = std::clamp(current_index, 0, total_waypoints - 1);
        if (current_index != fb.current_waypoint_index) {
          fb.current_waypoint_index = current_index;
          goal_handle->publish_feedback(std::make_shared<CoverVision::Feedback>(fb));
        }
      }

      const auto now = this->now();
      if (!detected) {
        detected = try_get_detection_map_pose(det, now, detected_pose_map);
        if (detected) {
          RCLCPP_INFO(get_logger(), "CoverVision: object detected, canceling coverage NavigateThroughPoses");
          nav_through_client_->async_cancel_goal(ntp_gh);
        }
      }

      if (res_future.wait_for(50ms) == std::future_status::ready) {
        const auto wr = res_future.get();
        if (!detected) {
          if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
            // Coverage completed without detection => SUCCESS per mission policy.
            result->mission_result = 1;
            result->waypoints_completed = static_cast<int32_t>(poses.size());
            goal_handle->succeed(result);
            return;
          }
          if (wr.code == rclcpp_action::ResultCode::CANCELED) {
            result->mission_result = 0;
            result->waypoints_completed = 0;
            goal_handle->canceled(result);
            return;
          }
          result->mission_result = 0;
          result->waypoints_completed = 0;
          goal_handle->abort(result);
          return;
        }

        // Detected object: regardless of NavigateThroughPoses result, attempt approach.
        break;
      }

      rclcpp::sleep_for(50ms);
    }

    if (!detected) {
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
      return;
    }

    // 4) Approach detected pose (NavigateToPose).
    detected_pose_map.header.stamp = this->now();
    detected_pose_map.header.frame_id = map_frame_;
    detected_pose_map.pose.position.z = 0.0;
    detected_pose_map.pose.orientation.w = 1.0;

    const auto nav_obj_rc = run_navigate_to_pose(goal_handle, detected_pose_map);
    if (nav_obj_rc == rclcpp_action::ResultCode::CANCELED) {
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->canceled(result);
      return;
    }
    if (nav_obj_rc != rclcpp_action::ResultCode::SUCCEEDED) {
      result->mission_result = 0;
      result->waypoints_completed = 0;
      goal_handle->abort(result);
      return;
    }

    result->mission_result = 1;
    result->waypoints_completed = static_cast<int32_t>(poses.size());
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverVisionServer>());
  rclcpp::shutdown();
  return 0;
}
