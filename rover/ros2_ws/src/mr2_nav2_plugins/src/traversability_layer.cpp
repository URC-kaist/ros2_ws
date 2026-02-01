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
  tf_timeout_(0.1),
  has_data_(false)
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
  declareParameter("tf_timeout", rclcpp::ParameterValue(tf_timeout_));

  node->get_parameter(getFullName("gridmap_topic"), gridmap_topic_);
  node->get_parameter(getFullName("gridmap_layer"), gridmap_layer_);
  node->get_parameter(getFullName("rectangle_frame"), rectangle_frame_);
  node->get_parameter(getFullName("x_forward_m"), x_forward_m_);
  node->get_parameter(getFullName("y_width_m"), y_width_m_);
  node->get_parameter(getFullName("use_maximum"), use_maximum_);
  node->get_parameter(getFullName("tf_timeout"), tf_timeout_);

  auto qos = rclcpp::SensorDataQoS();
  gridmap_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    gridmap_topic_, qos,
    std::bind(&TraversabilityLayer::gridMapCallback, this, std::placeholders::_1));

  matchSize();
}

void
TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_ || !has_data_) {
    return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  auto node = node_.lock();
  if (!node) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), rectangle_frame_,
      rclcpp::Time(0), rclcpp::Duration::from_seconds(tf_timeout_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node->get_logger(),
      "TraversabilityLayer: failed to get TF %s -> %s: %s",
      rectangle_frame_.c_str(), layered_costmap_->getGlobalFrameID().c_str(), ex.what());
    return;
  }

  const double half_w = 0.5 * y_width_m_;
  std::array<geometry_msgs::msg::PointStamped, 4> corners{};
  // rectangle in sensor frame: origin at camera, forward +x, lateral +y
  corners[0].point.x = 0.0;         corners[0].point.y = -half_w;
  corners[1].point.x = x_forward_m_; corners[1].point.y = -half_w;
  corners[2].point.x = x_forward_m_; corners[2].point.y =  half_w;
  corners[3].point.x = 0.0;         corners[3].point.y =  half_w;
  for (auto & c : corners) {
    c.point.z = 0.0;
    c.header.frame_id = rectangle_frame_;
  }

  // We intentionally ignore the camera's orientation: ROI axes stay parallel to costmap axes.
  const auto & trans = tf_stamped.transform.translation;
  for (auto & c : corners) {
    const double wx = trans.x + c.point.x;
    const double wy = trans.y + c.point.y;
    touch(wx, wy, min_x, min_y, max_x, max_y);
  }
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

  if (use_maximum_) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  }
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
  std::lock_guard<std::mutex> lock(mutex_);

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
      setCost(mx, my, convertToCost(value));
      touch(p_out.point.x, p_out.point.y, &min_x, &min_y, &max_x, &max_y);
    }
  }

  if (min_x <= max_x && min_y <= max_y) {
    addExtraBounds(min_x, min_y, max_x, max_y);
  }

  has_data_ = true;
  current_ = true;

  RCLCPP_INFO_THROTTLE(
    node->get_logger(), *log_clock_, 5000,
    "TraversabilityLayer wrote data: bounds [%.2f, %.2f] to [%.2f, %.2f]",
    min_x, min_y, max_x, max_y);
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
