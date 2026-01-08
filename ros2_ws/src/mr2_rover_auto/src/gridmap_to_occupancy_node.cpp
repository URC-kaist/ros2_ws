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
    auto it = std::find(msg->layers.begin(), msg->layers.end(), layer_);
    if (it == msg->layers.end()) {
      RCLCPP_WARN(this->get_logger(), "Layer '%s' not found in GridMap.", layer_.c_str());
      return;
    }
    const std::size_t layer_idx = static_cast<std::size_t>(std::distance(msg->layers.begin(), it));
    const auto & arr = msg->data.at(layer_idx);

    std::size_t rows = 0;
    std::size_t cols = 0;
    if (arr.layout.dim.size() >= 2) {
      rows = arr.layout.dim[0].size;
      cols = arr.layout.dim[1].size;
    } else {
      rows = static_cast<std::size_t>(std::round(msg->info.length_y / msg->info.resolution));
      cols = static_cast<std::size_t>(std::round(msg->info.length_x / msg->info.resolution));
    }
    if (rows == 0 || cols == 0) {
      RCLCPP_WARN(this->get_logger(), "GridMap has zero-sized layout.");
      return;
    }

    std::vector<float> data(arr.data.begin(), arr.data.end());
    const std::size_t expected_size = rows * cols;
    if (data.size() < expected_size) {
      data.resize(expected_size, std::numeric_limits<float>::quiet_NaN());
    } else if (data.size() > expected_size) {
      data.resize(expected_size);
    }

    const double span = max_value_ - min_value_;
    const double scale = (std::abs(span) < 1e-6) ? 1.0 : (100.0 / span);

    std::vector<int8_t> occ(expected_size, static_cast<int8_t>(unknown_value_));
    for (std::size_t r = 0; r < rows; ++r) {
      for (std::size_t c = 0; c < cols; ++c) {
        const std::size_t src_r = rows - 1 - r;
        const std::size_t src_c = cols - 1 - c;
        const std::size_t src_idx = src_r * cols + src_c;
        const float v = data[src_idx];
        const std::size_t dst_idx = r * cols + c;

        if (!std::isfinite(v)) {
          occ[dst_idx] = static_cast<int8_t>(unknown_value_);
          continue;
        }

        double scaled = (static_cast<double>(v) - min_value_) * scale;
        scaled = std::clamp(scaled, 0.0, 100.0);
        if (invert_) {
          scaled = 100.0 - scaled;
        }
        scaled = std::clamp(scaled, -1.0, 100.0);
        occ[dst_idx] = static_cast<int8_t>(std::lround(scaled));
      }
    }

    nav_msgs::msg::OccupancyGrid out;
    out.header = msg->header;
    out.info.resolution = static_cast<float>(msg->info.resolution);
    out.info.width = static_cast<uint32_t>(cols);
    out.info.height = static_cast<uint32_t>(rows);

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

    out.data = std::move(occ);
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
