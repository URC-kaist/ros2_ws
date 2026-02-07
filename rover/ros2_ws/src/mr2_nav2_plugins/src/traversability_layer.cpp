/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Modified by: Jaeuk Kim (traversability_layer, 2026. 01)
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/

#include "mr2_nav2_plugins/traversability_layer.hpp"

#include <stdexcept>
#include <algorithm>
#include <limits>
#include <cmath>
#include <array>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mr2_nav2_plugins
{

TraversabilityLayer::TraversabilityLayer()
: gridmap_topic_("traversability_gridmap"),
  gridmap_layer_("traversability"),
  rectangle_frame_("base_link"),
  x_forward_m_(3.0),
  y_width_m_(4.0),
  use_maximum_(true),
  persistence_mode_(PersistenceMode::EMA),
  ema_alpha_(0.1),
  tf_timeout_(0.1),
  publish_private_costmap_(false),
  footprint_clearing_enabled_(true),
  has_data_(false),
  pending_bounds_(false),
  pending_min_x_(0.0),
  pending_min_y_(0.0),
  pending_max_x_(0.0),
  pending_max_y_(0.0)
{}

void
TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer: failed to lock lifecycle node");
  }

  current_ = true;
  enabled_ = true;
  log_clock_ = node->get_clock();

  declareParameter("gridmap_topic", rclcpp::ParameterValue(gridmap_topic_));
  declareParameter("gridmap_layer", rclcpp::ParameterValue(gridmap_layer_));
  declareParameter("rectangle_frame", rclcpp::ParameterValue(rectangle_frame_));
  declareParameter("x_forward_m", rclcpp::ParameterValue(x_forward_m_));
  declareParameter("y_width_m", rclcpp::ParameterValue(y_width_m_));
  declareParameter("use_maximum", rclcpp::ParameterValue(use_maximum_));
  declareParameter("persistence_mode", rclcpp::ParameterValue(std::string("ema")));
  declareParameter("ema_alpha", rclcpp::ParameterValue(ema_alpha_));
  declareParameter("tf_timeout", rclcpp::ParameterValue(tf_timeout_));
  declareParameter("publish_private_costmap", rclcpp::ParameterValue(publish_private_costmap_));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(footprint_clearing_enabled_));

  node->get_parameter(getFullName("gridmap_topic"), gridmap_topic_);
  node->get_parameter(getFullName("gridmap_layer"), gridmap_layer_);
  node->get_parameter(getFullName("rectangle_frame"), rectangle_frame_);
  node->get_parameter(getFullName("x_forward_m"), x_forward_m_);
  node->get_parameter(getFullName("y_width_m"), y_width_m_);
  node->get_parameter(getFullName("use_maximum"), use_maximum_);
  node->get_parameter(getFullName("ema_alpha"), ema_alpha_);
  std::string persistence_mode_str = "ema";
  node->get_parameter(getFullName("persistence_mode"), persistence_mode_str);
  if (persistence_mode_str == "ema") {
    persistence_mode_ = PersistenceMode::EMA;
  } else if (persistence_mode_str == "max") {
    persistence_mode_ = PersistenceMode::MAX;
  } else if (persistence_mode_str == "overwrite") {
    persistence_mode_ = PersistenceMode::OVERWRITE;
  } else {
    RCLCPP_WARN(node->get_logger(),
      "TraversabilityLayer: unknown persistence_mode '%s', defaulting to 'ema'",
      persistence_mode_str.c_str());
    persistence_mode_ = PersistenceMode::EMA;
  }
  ema_alpha_ = std::clamp(ema_alpha_, 0.0, 1.0);
  node->get_parameter(getFullName("tf_timeout"), tf_timeout_);
  node->get_parameter(getFullName("publish_private_costmap"), publish_private_costmap_);
  node->get_parameter(getFullName("footprint_clearing_enabled"), footprint_clearing_enabled_);

  auto qos = rclcpp::SensorDataQoS();
  gridmap_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    gridmap_topic_, qos,
    std::bind(&TraversabilityLayer::gridMapCallback, this, std::placeholders::_1));
  if (publish_private_costmap_) {
    private_costmap_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/traversability_private", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  }

  matchSize();
}

void
TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!enabled_ || !has_data_) {
    return;
  }

  // Keep the private costmap rolling with the master so data stays centered
  // around the robot instead of remaining fixed at the odom origin.
  if (layered_costmap_->isRolling()) {
    const double half_x = 0.5 * getSizeInMetersX();
    const double half_y = 0.5 * getSizeInMetersY();
    updateOrigin(robot_x - half_x, robot_y - half_y);
  }

  if (pending_bounds_) {
    addExtraBounds(pending_min_x_, pending_min_y_, pending_max_x_, pending_max_y_);
    pending_bounds_ = false;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  auto node = node_.lock();
  if (!node) {
    return;
  }

  // Copy parameters for TF / geometry work outside the mutex to avoid blocking callbacks.
  const double x_forward = x_forward_m_;
  const double y_width = y_width_m_;
  const std::string rectangle_frame = rectangle_frame_;
  const double tf_timeout = tf_timeout_;
  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();

  lock.unlock();

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_->lookupTransform(
      costmap_frame, rectangle_frame,
      rclcpp::Time(0), rclcpp::Duration::from_seconds(tf_timeout));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node->get_logger(),
      "TraversabilityLayer: failed to get TF %s -> %s: %s",
      rectangle_frame.c_str(), costmap_frame.c_str(), ex.what());
    return;
  }

  const double half_w = 0.5 * y_width;
  std::array<geometry_msgs::msg::PointStamped, 4> corners{};
  // rectangle in sensor frame: origin at camera, forward +x, lateral +y
  corners[0].point.x = 0.0;         corners[0].point.y = -half_w;
  corners[1].point.x = x_forward;   corners[1].point.y = -half_w;
  corners[2].point.x = x_forward;   corners[2].point.y =  half_w;
  corners[3].point.x = 0.0;         corners[3].point.y =  half_w;
  for (auto & c : corners) {
    c.point.z = 0.0;
    c.header.frame_id = rectangle_frame;
  }

  // Transform rectangle corners with full pose so ROI follows the sensor frame,
  // not just its translation. This keeps bounds aligned when the camera rotates.
  for (auto & c : corners) {
    geometry_msgs::msg::PointStamped c_out;
    try {
      tf2::doTransform(c, c_out, tf_stamped);
      touch(c_out.point.x, c_out.point.y, min_x, min_y, max_x, max_y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node->get_logger(),
        "TraversabilityLayer: rectangle corner transform failed: %s", ex.what());
      return;
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void
TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || !has_data_) {
    return;
  }

  if (footprint_clearing_enabled_ && !transformed_footprint_.empty()) {
    setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
  }

  if (use_maximum_) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  }
}

void
TraversabilityLayer::matchSize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  CostmapLayer::matchSize();
}

void
TraversabilityLayer::onFootprintChanged()
{
  // No footprint-specific logic yet.
}

void
TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
  has_data_ = false;
  current_ = true;
}

void TraversabilityLayer::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node || !enabled_) {
    return;
  }

  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::fromMessage(*msg, map)) {
    RCLCPP_WARN(node->get_logger(), "TraversabilityLayer: failed to convert GridMap message.");
    return;
  }

  if (!map.exists(gridmap_layer_)) {
    RCLCPP_WARN(node->get_logger(), "TraversabilityLayer: layer '%s' not found in GridMap.", gridmap_layer_.c_str());
    return;
  }

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_->lookupTransform(
      costmap_frame, map.getFrameId(), msg->header.stamp,
      rclcpp::Duration::from_seconds(tf_timeout_));
  } catch (const tf2::TransformException & ex) {
    try {
      tf_stamped = tf_->lookupTransform(costmap_frame, map.getFrameId(), rclcpp::Time(0));
      RCLCPP_WARN(node->get_logger(),
        "TraversabilityLayer: TF lookup failed at stamp (%s). Using latest transform. Error: %s",
        map.getFrameId().c_str(), ex.what());
    } catch (const tf2::TransformException & ex2) {
      RCLCPP_WARN(node->get_logger(),
        "TraversabilityLayer: TF lookup failed (%s -> %s). Error: %s",
        map.getFrameId().c_str(), costmap_frame.c_str(), ex2.what());
      return;
    }
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = -std::numeric_limits<double>::max();
  double max_y = -std::numeric_limits<double>::max();

  {
    std::lock_guard<std::mutex> lock(mutex_);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const float value = map.at(gridmap_layer_, *it);

    grid_map::Position pos_in_map;
    map.getPosition(*it, pos_in_map);

      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.header.frame_id = map.getFrameId();
      p_in.point.x = pos_in_map.x();
      p_in.point.y = pos_in_map.y();
      p_in.point.z = 0.0;

      try {
        tf2::doTransform(p_in, p_out, tf_stamped);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(node->get_logger(),
          "TraversabilityLayer: point transform failed: %s", ex.what());
        continue;
      }

      unsigned int mx, my;
      if (worldToMap(p_out.point.x, p_out.point.y, mx, my)) {
        // Skip NaNs: keep old cost to preserve past obstacle evidence.
        if (std::isnan(value)) {
          continue;
        }

        unsigned char new_cost = convertToCost(value);
        const unsigned char old_cost = getCost(mx, my);

        unsigned char fused = new_cost;
        if (persistence_mode_ == PersistenceMode::EMA) {
          if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
            fused = new_cost;
          } else {
            fused = static_cast<unsigned char>(
              std::round(ema_alpha_ * static_cast<double>(new_cost) +
              (1.0 - ema_alpha_) * static_cast<double>(old_cost)));
          }
        } else if (persistence_mode_ == PersistenceMode::MAX) {
          // Treat NO_INFORMATION as absence of data so we don't lock in 255.
          fused = (old_cost == nav2_costmap_2d::NO_INFORMATION)
                    ? new_cost
                    : std::max(old_cost, new_cost);
        } else { // OVERWRITE
          fused = new_cost;
        }

        setCost(mx, my, fused);
        touch(p_out.point.x, p_out.point.y, &min_x, &min_y, &max_x, &max_y);
      }
    }

    if (min_x <= max_x && min_y <= max_y) {
      pending_min_x_ = min_x;
      pending_min_y_ = min_y;
      pending_max_x_ = max_x;
      pending_max_y_ = max_y;
      pending_bounds_ = true;
    } else {
      pending_bounds_ = false;
    }

    has_data_ = true;
    current_ = true;
  }

  // Publish a snapshot of the private costmap for debugging.
  if (publish_private_costmap_ && private_costmap_pub_ &&
      private_costmap_pub_->get_subscription_count() > 0) {
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = layered_costmap_->getGlobalFrameID();
    out.info.resolution = getResolution();
    out.info.width = getSizeInCellsX();
    out.info.height = getSizeInCellsY();
    out.info.origin.position.x = getOriginX();
    out.info.origin.position.y = getOriginY();
    out.info.origin.position.z = 0.0;
    out.info.origin.orientation.w = 1.0;

    const unsigned char * src = getCharMap();
    out.data.resize(out.info.width * out.info.height);
    for (size_t i = 0; i < out.data.size(); ++i) {
      const unsigned char c = src[i];
      if (c == nav2_costmap_2d::NO_INFORMATION) {
        out.data[i] = -1;
      } else {
        out.data[i] = static_cast<int8_t>(std::min<int>(100, (c * 100) / 254));
      }
    }
    private_costmap_pub_->publish(std::move(out));
  }

  // RCLCPP_INFO_THROTTLE(
  //   node->get_logger(), *log_clock_, 5000,
  //   "TraversabilityLayer wrote data: bounds [%.2f, %.2f] to [%.2f, %.2f]",
  //   min_x, min_y, max_x, max_y);
}

void
TraversabilityLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  transformed_footprint_.clear();
  nav2_costmap_2d::transformFootprint(
    robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
  for (const auto & pt : transformed_footprint_) {
    touch(pt.x, pt.y, min_x, min_y, max_x, max_y);
  }
}

unsigned char TraversabilityLayer::convertToCost(float value) const
{
  if (std::isnan(value)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  const float clamped = std::clamp(value, 0.0f, 1.0f);
  return static_cast<unsigned char>(clamped * 254.0f);
}

} // namespace mr2_nav2_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mr2_nav2_plugins::TraversabilityLayer, nav2_costmap_2d::Layer)
