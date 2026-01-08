#pragma once

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

namespace mr2_rover_auto
{

class GridMapTraversabilityLayer : public nav2_costmap_2d::Layer
{
public:
  GridMapTraversabilityLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  unsigned char valueToCost(float value) const;

  grid_map::GridMap grid_map_;
  std::mutex mutex_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;

  std::string grid_map_topic_;
  std::string layer_;
  std::string map_frame_;

  double min_value_;
  double max_value_;
  double lethal_threshold_;
  bool invert_;
  bool flip_x_;
  bool flip_y_;
  int unknown_cost_;
  bool has_map_{false};
};

}  // namespace mr2_rover_auto
