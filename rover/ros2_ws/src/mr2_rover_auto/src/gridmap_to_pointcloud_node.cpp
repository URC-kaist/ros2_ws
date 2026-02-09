#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <vector>
#include <limits>

namespace
{

class GridMapToPointCloudNode : public rclcpp::Node
{
public:
  GridMapToPointCloudNode()
  : rclcpp::Node("gridmap_to_pointcloud")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/traversability_gridmap");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/traversability_cloud");
    layer_ = this->declare_parameter<std::string>("layer", "traversability");

    auto qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, qos,
      std::bind(&GridMapToPointCloudNode::gridMapCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

    RCLCPP_INFO(
      this->get_logger(),
      "GridMap(%s) → PointCloud2 on %s", layer_.c_str(), output_topic_.c_str());
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

    // Reserve worst-case size.
    const int cols = grid_map.getSize()(0);
    const int rows = grid_map.getSize()(1);
    const std::size_t max_pts = static_cast<std::size_t>(cols * rows);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.fields.clear();
    auto make_field = [](const std::string & name, uint32_t offset) {
      sensor_msgs::msg::PointField f;
      f.name = name;
      f.offset = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      return f;
    };
    cloud.fields.push_back(make_field("x", 0));
    cloud.fields.push_back(make_field("y", 4));
    cloud.fields.push_back(make_field("z", 8));
    cloud.fields.push_back(make_field("intensity", 12));
    cloud.point_step = 16;
    cloud.data.resize(max_pts * cloud.point_step);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");

    std::size_t count = 0;
    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
      const float v = grid_map.at(layer_, *it);
      if (!std::isfinite(v)) {
        continue;
      }
      grid_map::Position pos;
      if (!grid_map.getPosition(*it, pos)) {
        continue;
      }
      *iter_x = static_cast<float>(pos.x());
      *iter_y = static_cast<float>(pos.y());
      *iter_z = v;          // encode layer value in z to get a voxel-like vertical signal
      *iter_i = v;          // mirror into intensity for coloring

      ++iter_x; ++iter_y; ++iter_z; ++iter_i;
      ++count;
    }

    cloud.width = static_cast<uint32_t>(count);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));  // shrink to actual used bytes

    pub_->publish(cloud);
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string layer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridMapToPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
