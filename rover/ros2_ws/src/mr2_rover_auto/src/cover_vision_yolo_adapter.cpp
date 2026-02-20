#include <rclcpp/rclcpp.hpp>

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

class CoverVisionYoloAdapter : public rclcpp::Node
{
public:
  CoverVisionYoloAdapter()
  : rclcpp::Node("cover_vision_yolo_adapter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this, true)
  {
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    mission_status_topic_ =
      this->declare_parameter<std::string>("mission_status_topic", "mission_status");
    output_topic_ =
      this->declare_parameter<std::string>("output_topic", "cover_vision/object_pose");
    yolo_pose_topic_prefix_ =
      this->declare_parameter<std::string>("yolo_pose_topic_prefix", "yolo/object_pose");

    force_enable_ = this->declare_parameter<bool>("force_enable", false);
    forced_class_id_ = this->declare_parameter<int>("forced_class_id", 0);

    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.2);
    min_publish_interval_sec_ = this->declare_parameter<double>("min_publish_interval_sec", 0.1);
    filter_window_ = static_cast<size_t>(this->declare_parameter<int>("filter_window", 5));
    max_jump_m_ = this->declare_parameter<double>("max_jump_m", 2.0);
    max_detection_distance_m_ = this->declare_parameter<double>("max_detection_distance_m", 0.0);

    out_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    status_sub_ = this->create_subscription<mr2_action_interface::msg::MissionStatus>(
      mission_status_topic_, 10,
      std::bind(&CoverVisionYoloAdapter::on_status, this, std::placeholders::_1));

    if (force_enable_) {
      const int class_id = std::max(0, forced_class_id_);
      std::lock_guard<std::mutex> lock(mutex_);
      enabled_ = true;
      ensure_subscription_locked(class_id);
      RCLCPP_INFO(get_logger(), "CoverVision YOLO adapter: force enabled (class_id=%d)", class_id);
    }

    RCLCPP_INFO(get_logger(), "CoverVision YOLO adapter ready");
  }

private:
  // Match the enums documented in MissionSpec.msg / README.md.
  static constexpr uint8_t STATE_RUNNING = 1;
  static constexpr uint8_t MISSION_COVER_VISION = 2;
  static constexpr uint8_t DETECTION_YOLO = 2;

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
            st.active_mission.detection_method == DETECTION_YOLO);
  }

  bool is_within_distance_limit(const geometry_msgs::msg::PoseStamped & pose_map)
  {
    if (!std::isfinite(max_detection_distance_m_) || max_detection_distance_m_ <= 0.0) {
      return true;
    }

    geometry_msgs::msg::PoseStamped pose_base;
    try {
      if (pose_map.header.frame_id.empty()) {
        return false;
      }
      if (pose_map.header.frame_id == base_frame_) {
        pose_base = pose_map;
      } else {
        pose_base = tf_buffer_.transform(
          pose_map, base_frame_, tf2::durationFromSec(tf_timeout_sec_));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CoverVision YOLO adapter: distance TF failed (%s -> %s): %s",
        pose_map.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    const double dx = pose_base.pose.position.x;
    const double dy = pose_base.pose.position.y;
    const double distance = std::hypot(dx, dy);
    return distance <= max_detection_distance_m_;
  }

  void disable_locked()
  {
    enabled_ = false;
    active_class_id_ = -1;
    yolo_sub_.reset();
    recent_xy_.clear();
    last_pub_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_pub_xy_.reset();
  }

  void ensure_subscription_locked(int class_id)
  {
    if (class_id == active_class_id_ && yolo_sub_) {
      return;
    }

    active_class_id_ = class_id;
    recent_xy_.clear();
    last_pub_xy_.reset();
    last_pub_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    const std::string topic = yolo_pose_topic_prefix_ + "/class_" + std::to_string(class_id);
    yolo_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, 10,
      std::bind(&CoverVisionYoloAdapter::on_yolo_pose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CoverVision YOLO adapter: subscribed to %s", topic.c_str());
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
    if (!should_enable_for_status(*msg)) {
      if (enabled_) {
        RCLCPP_INFO(get_logger(), "CoverVision YOLO adapter: disabled");
      }
      disable_locked();
      return;
    }

    enabled_ = true;

    const int class_id = std::max(0, msg->active_mission.object_type);
    ensure_subscription_locked(class_id);
  }

  void on_yolo_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

    geometry_msgs::msg::PoseStamped pose_map;
    try {
      if (msg->header.frame_id.empty()) {
        return;
      }
      if (msg->header.frame_id == map_frame_) {
        pose_map = *msg;
      } else {
        pose_map = tf_buffer_.transform(*msg, map_frame_, tf2::durationFromSec(tf_timeout_sec_));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CoverVision YOLO adapter: TF transform failed (%s -> %s): %s",
        msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return;
    }

    const auto now = this->now();
    pose_map.header.stamp = now;
    pose_map.header.frame_id = map_frame_;
    pose_map.pose.position.z = 0.0;
    pose_map.pose.orientation.w = 1.0;

    if (!is_within_distance_limit(pose_map)) {
      return;
    }

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
      "CoverVision YOLO adapter: detection published (class_id=%d, x=%.2f, y=%.2f)",
      active_class_id_, fx, fy);

    last_pub_time_ = now;
    last_pub_xy_ = std::make_pair(fx, fy);
  }

  std::mutex mutex_;
  bool enabled_{false};
  int active_class_id_{-1};

  std::string map_frame_;
  std::string base_frame_;
  std::string mission_status_topic_;
  std::string output_topic_;
  std::string yolo_pose_topic_prefix_;

  bool force_enable_{false};
  int forced_class_id_{0};

  double tf_timeout_sec_{0.2};
  double min_publish_interval_sec_{0.1};
  size_t filter_window_{5};
  double max_jump_m_{2.0};
  double max_detection_distance_m_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr out_pub_;
  rclcpp::Subscription<mr2_action_interface::msg::MissionStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr yolo_sub_;

  std::deque<std::pair<double, double>> recent_xy_;
  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
  std::optional<std::pair<double, double>> last_pub_xy_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverVisionYoloAdapter>());
  rclcpp::shutdown();
  return 0;
}
