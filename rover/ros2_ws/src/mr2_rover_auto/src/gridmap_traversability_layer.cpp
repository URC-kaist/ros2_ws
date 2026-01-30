#include "mr2_rover_auto/gridmap_traversability_layer.hpp"

#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mr2_rover_auto
{

void GridMapTraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("GridMapTraversabilityLayer: failed to lock node.");
  }
  const std::string prefix = name_ + ".";

  node->declare_parameter(prefix + "enabled", true);
  node->declare_parameter(prefix + "grid_map_topic", std::string("/traversability_gridmap"));
  node->declare_parameter(prefix + "layer", std::string("traversability"));
  node->declare_parameter(prefix + "min_value", 0.0);
  node->declare_parameter(prefix + "max_value", 1.0);
  node->declare_parameter(prefix + "invert", true);
  node->declare_parameter(prefix + "unknown_cost", static_cast<int>(nav2_costmap_2d::NO_INFORMATION));
  node->declare_parameter(prefix + "lethal_threshold", -1.0);
  node->declare_parameter(prefix + "flip_x", false);
  node->declare_parameter(prefix + "flip_y", false);
  node->declare_parameter(prefix + "clearable", false);
  node->declare_parameter(prefix + "qos_reliable", true);
  node->declare_parameter(prefix + "qos_transient_local", true);
  node->declare_parameter(prefix + "tf_timeout", 0.1);

  node->get_parameter(prefix + "enabled", enabled_);
  node->get_parameter(prefix + "grid_map_topic", grid_map_topic_);
  node->get_parameter(prefix + "layer", layer_);
  node->get_parameter(prefix + "min_value", min_value_);
  node->get_parameter(prefix + "max_value", max_value_);
  node->get_parameter(prefix + "invert", invert_);
  node->get_parameter(prefix + "unknown_cost", unknown_cost_);
  node->get_parameter(prefix + "lethal_threshold", lethal_threshold_);
  node->get_parameter(prefix + "flip_x", flip_x_);
  node->get_parameter(prefix + "flip_y", flip_y_);
  node->get_parameter(prefix + "clearable", clearable_);
  node->get_parameter(prefix + "qos_reliable", qos_reliable_);
  node->get_parameter(prefix + "qos_transient_local", qos_transient_local_);
  double tf_timeout_sec = 0.1;
  node->get_parameter(prefix + "tf_timeout", tf_timeout_sec);
  tf_timeout_ = rclcpp::Duration::from_seconds(tf_timeout_sec);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  if (qos_reliable_) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  if (qos_transient_local_) {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }
  sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic_, qos,
    std::bind(&GridMapTraversabilityLayer::gridMapCallback, this, std::placeholders::_1));

  current_ = true;
}

void GridMapTraversabilityLayer::gridMapCallback(
  const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  grid_map::GridMap local_map;
  if (!grid_map::GridMapRosConverter::fromMessage(*msg, local_map)) {
    RCLCPP_WARN(
      node->get_logger(), "GridMapTraversabilityLayer: Failed to convert GridMap message.");
    return;
  }

  if (!local_map.exists(layer_)) {
    RCLCPP_WARN(
      node->get_logger(),
      "GridMapTraversabilityLayer: Layer '%s' not found in GridMap.", layer_.c_str());
    return;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  if (!global_frame.empty() && msg->header.frame_id != global_frame) {
    RCLCPP_DEBUG_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "GridMapTraversabilityLayer: transforming GridMap frame '%s' -> costmap frame '%s'.",
      msg->header.frame_id.c_str(), global_frame.c_str());
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map_ = std::move(local_map);
    map_frame_ = msg->header.frame_id;
    last_map_stamp_ = rclcpp::Time(msg->header.stamp);
    has_map_ = true;
  }
}

void GridMapTraversabilityLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_map_) {
    return;
  }

  const grid_map::Position center = grid_map_.getPosition();
  const grid_map::Length length = grid_map_.getLength();
  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  const std::string map_frame = map_frame_;
  const rclcpp::Time stamp = (last_map_stamp_.nanoseconds() == 0) ?
    rclcpp::Time(0) : last_map_stamp_;

  if (!global_frame.empty() && !map_frame.empty() && map_frame != global_frame) {
    tf2::Transform T;
    if (!lookupTransformWithFallback(global_frame, map_frame, stamp, T)) {
      return;
    }

    const double half_x = 0.5 * length.x();
    const double half_y = 0.5 * length.y();
    const std::array<tf2::Vector3, 4> corners = {
      tf2::Vector3(center.x() - half_x, center.y() - half_y, 0.0),
      tf2::Vector3(center.x() - half_x, center.y() + half_y, 0.0),
      tf2::Vector3(center.x() + half_x, center.y() - half_y, 0.0),
      tf2::Vector3(center.x() + half_x, center.y() + half_y, 0.0)
    };

    for (const auto & corner : corners) {
      const tf2::Vector3 p = T * corner;
      *min_x = std::min(*min_x, p.x());
      *min_y = std::min(*min_y, p.y());
      *max_x = std::max(*max_x, p.x());
      *max_y = std::max(*max_y, p.y());
    }
    return;
  }

  const double half_x = 0.5 * length.x();
  const double half_y = 0.5 * length.y();

  *min_x = std::min(*min_x, center.x() - half_x);
  *min_y = std::min(*min_y, center.y() - half_y);
  *max_x = std::max(*max_x, center.x() + half_x);
  *max_y = std::max(*max_y, center.y() + half_y);
}

void GridMapTraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_map_ || !grid_map_.exists(layer_)) {
    return;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  const std::string map_frame = map_frame_;
  const rclcpp::Time stamp = (last_map_stamp_.nanoseconds() == 0) ?
    rclcpp::Time(0) : last_map_stamp_;
  tf2::Transform T;
  bool use_transform = false;

  if (!global_frame.empty() && !map_frame.empty() && map_frame != global_frame) {
    if (!lookupTransformWithFallback(global_frame, map_frame, stamp, T)) {
      return;
    }
    use_transform = true;
  }

  const auto size = grid_map_.getSize();
  for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
    grid_map::Index data_index = *it;
    if (flip_x_) {
      data_index(0) = size(0) - 1 - data_index(0);
    }
    if (flip_y_) {
      data_index(1) = size(1) - 1 - data_index(1);
    }

    float value = grid_map_.at(layer_, data_index);
    if (!std::isfinite(value)) {
      value = std::numeric_limits<float>::quiet_NaN();
    }

    grid_map::Position pos;
    grid_map_.getPosition(*it, pos);
    double wx = pos.x();
    double wy = pos.y();
    if (use_transform) {
      const tf2::Vector3 p = T * tf2::Vector3(pos.x(), pos.y(), 0.0);
      wx = p.x();
      wy = p.y();
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!master_grid.worldToMap(wx, wy, mx, my)) {
      continue;
    }

    if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
      static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }

    master_grid.setCost(mx, my, valueToCost(value));
  }
}

bool GridMapTraversabilityLayer::lookupTransformWithFallback(
  const std::string & target, const std::string & source,
  const rclcpp::Time & stamp, tf2::Transform & out_tf)
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }

  // First try at message stamp; if it fails, fall back to latest.
  for (int attempt = 0; attempt < 2; ++attempt) {
    const rclcpp::Time query_time = (attempt == 0) ? stamp : rclcpp::Time(0);
    try {
      const auto tf_msg = tf_buffer_->lookupTransform(target, source, query_time, tf_timeout_);
      tf2::fromMsg(tf_msg.transform, out_tf);
      return true;
    } catch (const tf2::TransformException & ex) {
      if (attempt == 0) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "GridMapTraversabilityLayer: TF lookup failed (%s -> %s @ stamp). Will try latest. Error: %s",
          source.c_str(), target.c_str(), ex.what());
      } else {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "GridMapTraversabilityLayer: TF lookup failed (%s -> %s @ latest). Error: %s",
          source.c_str(), target.c_str(), ex.what());
      }
    }
  }
  return false;
}

unsigned char GridMapTraversabilityLayer::valueToCost(float value) const
{
  if (!std::isfinite(value)) {
    return static_cast<unsigned char>(unknown_cost_);
  }

  if (lethal_threshold_ >= 0.0 && value >= lethal_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const double span = max_value_ - min_value_;
  double norm = (std::abs(span) < 1e-9) ? 0.0 : (value - min_value_) / span;
  norm = std::clamp(norm, 0.0, 1.0);
  if (invert_) {
    norm = 1.0 - norm;
  }

  const double scaled = norm * static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE);
  return static_cast<unsigned char>(std::lround(scaled));
}

void GridMapTraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_map_ = false;
  grid_map_.clearAll();
}

}  // namespace mr2_rover_auto

PLUGINLIB_EXPORT_CLASS(
  mr2_rover_auto::GridMapTraversabilityLayer,
  nav2_costmap_2d::Layer)
