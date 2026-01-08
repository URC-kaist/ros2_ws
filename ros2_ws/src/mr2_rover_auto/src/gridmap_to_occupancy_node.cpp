#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

class GridMapToOccupancyNode : public rclcpp::Node
{
public:
  GridMapToOccupancyNode()
  : rclcpp::Node("gridmap_to_occupancy")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/traversability_gridmap");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/traversability_occupancy");
    layer_ = this->declare_parameter<std::string>("layer", "traversability");
    min_value_ = this->declare_parameter<double>("min_value", 0.0);
    max_value_ = this->declare_parameter<double>("max_value", 1.0);
    invert_ = this->declare_parameter<bool>("invert", true);
    unknown_value_ = this->declare_parameter<int>("unknown_value", -1);

    rclcpp::QoS qos(1);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, rclcpp::QoS(1),
      std::bind(&GridMapToOccupancyNode::gridMapCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, qos);

    RCLCPP_INFO(
      this->get_logger(),
      "GridMap(%s) → OccupancyGrid on %s [range %.2f .. %.2f, invert=%s]",
      layer_.c_str(), output_topic_.c_str(), min_value_, max_value_,
      invert_ ? "true" : "false");
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap grid_map;
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, grid_map)) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert GridMap message.");
      return;
    }
    if (!grid_map.exists(layer_)) {
      RCLCPP_WARN(this->get_logger(), "Layer '%s' not found in GridMap.", layer_.c_str());
      return;
    }

    nav_msgs::msg::OccupancyGrid out;
    const float data_min = invert_ ? static_cast<float>(max_value_) : static_cast<float>(min_value_);
    const float data_max = invert_ ? static_cast<float>(min_value_) : static_cast<float>(max_value_);
    grid_map::GridMapRosConverter::toOccupancyGrid(
      grid_map, layer_, data_min, data_max, out);

    if (unknown_value_ != -1) {
      std::replace(out.data.begin(), out.data.end(), static_cast<int8_t>(-1),
        static_cast<int8_t>(unknown_value_));
    }

    out.header = msg->header;
    out.info.resolution = static_cast<float>(msg->info.resolution);

    const auto & pose = msg->info.pose;
    const double cx = pose.position.x;
    const double cy = pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    tf2::Matrix3x3 R(q);
    const tf2::Vector3 half(msg->info.length_x / 2.0, msg->info.length_y / 2.0, 0.0);
    const tf2::Vector3 offset = R * half;

    out.info.origin.position.x = cx - offset.x();
    out.info.origin.position.y = cy - offset.y();
    out.info.origin.position.z = pose.position.z;
    out.info.origin.orientation = pose.orientation;

    pub_->publish(out);
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string layer_;
  double min_value_;
  double max_value_;
  bool invert_;
  int unknown_value_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridMapToOccupancyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
