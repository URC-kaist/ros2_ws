#include <rclcpp/rclcpp.hpp>

#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mr2_action_interface/msg/mission_status.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

class CoverVisionArucoAdapter : public rclcpp::Node
{
public:
  CoverVisionArucoAdapter()
  : rclcpp::Node("cover_vision_aruco_adapter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this, true)
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    mission_status_topic_ =
      this->declare_parameter<std::string>("mission_status_topic", "mission_status");
    output_topic_ =
      this->declare_parameter<std::string>("output_topic", "cover_vision/object_pose");

    aruco_detections_topic_ =
      this->declare_parameter<std::string>("aruco_detections_topic", "aruco_detections");
    prefer_boards_ = this->declare_parameter<bool>("prefer_boards", true);
    preferred_board_name_ = this->declare_parameter<std::string>("preferred_board_name", "aruco_post");
    preferred_marker_id_ = this->declare_parameter<int>("preferred_marker_id", -1);

    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.2);
    min_publish_interval_sec_ = this->declare_parameter<double>("min_publish_interval_sec", 0.1);
    filter_window_ = static_cast<size_t>(this->declare_parameter<int>("filter_window", 5));
    max_jump_m_ = this->declare_parameter<double>("max_jump_m", 2.0);

    force_enable_ = this->declare_parameter<bool>("force_enable", false);

    out_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);

    status_sub_ = this->create_subscription<mr2_action_interface::msg::MissionStatus>(
      mission_status_topic_, 10,
      std::bind(&CoverVisionArucoAdapter::on_status, this, std::placeholders::_1));

    if (force_enable_) {
      std::lock_guard<std::mutex> lock(mutex_);
      enabled_ = true;
      ensure_subscription_locked();
      RCLCPP_INFO(get_logger(), "CoverVision ArUco adapter: force enabled");
    }

    RCLCPP_INFO(get_logger(), "CoverVision ArUco adapter ready");
  }

private:
  // Match the enums documented in MissionSpec.msg / README.md.
  static constexpr uint8_t STATE_RUNNING = 1;
  static constexpr uint8_t MISSION_COVER_VISION = 2;
  static constexpr uint8_t DETECTION_ARUCO = 1;

  static double median(std::vector<double> v)
  {
    if (v.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    double m = v[mid];
    if ((v.size() % 2U) == 0U) {
      const auto it = std::max_element(v.begin(), v.begin() + mid);
      m = 0.5 * (m + *it);
    }
    return m;
  }

  bool should_enable_for_status(const mr2_action_interface::msg::MissionStatus & st) const
  {
    return (st.state == STATE_RUNNING &&
            st.active_mission.mission_type == MISSION_COVER_VISION &&
            st.active_mission.detection_method == DETECTION_ARUCO);
  }

  std::optional<geometry_msgs::msg::PoseStamped> select_pose_in(
    const aruco_opencv_msgs::msg::ArucoDetection & msg,
    std::optional<int> marker_id_override)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg.header;

    if (prefer_boards_ && !msg.boards.empty()) {
      if (!preferred_board_name_.empty()) {
        for (const auto & b : msg.boards) {
          if (b.board_name == preferred_board_name_) {
            ps.pose = b.pose;
            return ps;
          }
        }
      }
      ps.pose = msg.boards.front().pose;
      return ps;
    }

    if (!msg.markers.empty()) {
      const int effective_marker_id = marker_id_override.value_or(preferred_marker_id_);
      if (effective_marker_id >= 0) {
        const auto want = static_cast<uint16_t>(effective_marker_id);
        for (const auto & m : msg.markers) {
          if (m.marker_id == want) {
            ps.pose = m.pose;
            return ps;
          }
        }
      }
      ps.pose = msg.markers.front().pose;
      return ps;
    }

    return std::nullopt;
  }

  void ensure_subscription_locked()
  {
    if (aruco_sub_) {
      return;
    }
    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      aruco_detections_topic_, 10,
      std::bind(&CoverVisionArucoAdapter::on_aruco_detections, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "CoverVision ArUco adapter: subscribed to %s", aruco_detections_topic_.c_str());
  }

  void disable_locked()
  {
    enabled_ = false;
    active_mission_marker_id_.reset();
    aruco_sub_.reset();
    recent_xy_.clear();
    last_pub_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_pub_xy_.reset();
  }

  void on_status(const mr2_action_interface::msg::MissionStatus::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (force_enable_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const bool enable_now = should_enable_for_status(*msg);
    if (!enable_now) {
      if (enabled_) {
        RCLCPP_INFO(get_logger(), "CoverVision ArUco adapter: disabled");
      }
      disable_locked();
      return;
    }

    enabled_ = true;
    ensure_subscription_locked();

    // Allow overriding marker selection from mission spec if desired.
    if (enabled_ && preferred_marker_id_ < 0 && msg->active_mission.object_type >= 0) {
      active_mission_marker_id_ = msg->active_mission.object_type;
    } else {
      active_mission_marker_id_.reset();
    }
  }

  void on_aruco_detections(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!enabled_) {
        return;
      }
    }

    std::optional<int> marker_override;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      marker_override = active_mission_marker_id_;
    }

    const auto in_opt = select_pose_in(*msg, marker_override);
    if (!in_opt.has_value()) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_map;
    try {
      if (in_opt->header.frame_id.empty()) {
        return;
      }
      if (in_opt->header.frame_id == map_frame_) {
        pose_map = *in_opt;
      } else {
        pose_map = tf_buffer_.transform(*in_opt, map_frame_, tf2::durationFromSec(tf_timeout_sec_));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CoverVision ArUco adapter: TF transform failed (%s -> %s): %s",
        in_opt->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return;
    }

    const auto now = this->now();
    pose_map.header.stamp = now;
    pose_map.header.frame_id = map_frame_;
    pose_map.pose.position.z = 0.0;
    pose_map.pose.orientation.w = 1.0;

    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) {
      return;
    }

    if (filter_window_ == 0) {
      filter_window_ = 1;
    }

    recent_xy_.emplace_back(pose_map.pose.position.x, pose_map.pose.position.y);
    while (recent_xy_.size() > filter_window_) {
      recent_xy_.pop_front();
    }

    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(recent_xy_.size());
    ys.reserve(recent_xy_.size());
    for (const auto & xy : recent_xy_) {
      xs.push_back(xy.first);
      ys.push_back(xy.second);
    }
    const double fx = median(std::move(xs));
    const double fy = median(std::move(ys));
    if (!std::isfinite(fx) || !std::isfinite(fy)) {
      return;
    }

    if (last_pub_time_.nanoseconds() != 0) {
      const double dt = (now - last_pub_time_).seconds();
      if (dt < min_publish_interval_sec_) {
        return;
      }
    }

    if (last_pub_xy_.has_value() && std::isfinite(max_jump_m_) && max_jump_m_ > 0.0) {
      const double dx = fx - last_pub_xy_->first;
      const double dy = fy - last_pub_xy_->second;
      const double d = std::hypot(dx, dy);
      if (d > max_jump_m_) {
        return;
      }
    }

    geometry_msgs::msg::PoseStamped out = pose_map;
    out.pose.position.x = fx;
    out.pose.position.y = fy;
    out_pub_->publish(out);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "CoverVision ArUco adapter: detection published (x=%.2f, y=%.2f)",
      fx, fy);

    last_pub_time_ = now;
    last_pub_xy_ = std::make_pair(fx, fy);
  }

  std::mutex mutex_;
  bool enabled_{false};
  std::optional<int> active_mission_marker_id_;

  std::string map_frame_;
  std::string mission_status_topic_;
  std::string output_topic_;
  std::string aruco_detections_topic_;

  bool force_enable_{false};

  bool prefer_boards_{true};
  std::string preferred_board_name_;
  int preferred_marker_id_{-1};

  double tf_timeout_sec_{0.2};
  double min_publish_interval_sec_{0.1};
  size_t filter_window_{5};
  double max_jump_m_{2.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr out_pub_;
  rclcpp::Subscription<mr2_action_interface::msg::MissionStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;

  std::deque<std::pair<double, double>> recent_xy_;
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
  std::optional<std::pair<double, double>> last_pub_xy_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverVisionArucoAdapter>());
  rclcpp::shutdown();
  return 0;
}
