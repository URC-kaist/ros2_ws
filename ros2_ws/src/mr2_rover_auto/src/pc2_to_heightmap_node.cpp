#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{

class Pc2ToHeightmapNode : public rclcpp::Node
{
public:
  Pc2ToHeightmapNode()
  : rclcpp::Node("pc2_to_heightmap_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/rgbd_camera/points");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "height_gridmap");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "base_link");
    x_forward_ = this->declare_parameter<double>("x_forward_m", 5.0);
    y_width_ = this->declare_parameter<double>("y_width_m", 3.0);
    resolution_ = this->declare_parameter<double>("resolution", 0.05);
    layer_name_ = this->declare_parameter<std::string>("layer_name", "elevation");
    voxel_size_ = this->declare_parameter<double>("voxel_size_m", 0.0);
    roi_z_max_ = this->declare_parameter<double>("roi_z_max_m", 0.0);

    grid_cols_ = static_cast<int>(std::ceil(x_forward_ / resolution_));
    grid_rows_ = static_cast<int>(std::ceil(y_width_ / resolution_));
    y_min_ = -y_width_ / 2.0;
    total_cells_ = static_cast<std::size_t>(grid_cols_ * grid_rows_);

    sum_bins_.assign(total_cells_, 0.0f);
    count_bins_.assign(total_cells_, 0U);
    heightmap_buffer_.assign(total_cells_, std::numeric_limits<float>::quiet_NaN());

    gridmap_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic_, 1);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Pc2ToHeightmapNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "pc2_to_heightmap: %d x %d cells (res=%.3f m) publishing %s",
      grid_cols_, grid_rows_, resolution_, output_topic_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const rclcpp::Time stamp = msg->header.stamp;
    geometry_msgs::msg::TransformStamped tf_cam_map;
    geometry_msgs::msg::TransformStamped tf_cam_base;
    geometry_msgs::msg::TransformStamped tf_base_map;

    try {
      tf_cam_map = tf_buffer_.lookupTransform(
        map_frame_, msg->header.frame_id, stamp, tf_timeout_);
      tf_cam_base = tf_buffer_.lookupTransform(
        base_frame_, msg->header.frame_id, stamp, tf_timeout_);
      tf_base_map = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, stamp, tf_timeout_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
      return;
    }

    // ROI filter in camera frame + finite filter.
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> filtered;
    filtered.reserve(cloud.size());
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      if (roi_z_max_ > 0.0 && p.z >= roi_z_max_) {
        continue;
      }
      filtered.push_back(p);
    }

    if (filtered.empty()) {
      return;
    }

    if (voxel_size_ > 0.0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
      tmp->points = filtered;
      tmp->width = tmp->points.size();
      tmp->height = 1;

      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      voxel.setInputCloud(tmp);

      pcl::PointCloud<pcl::PointXYZ> down;
      voxel.filter(down);
      filtered.swap(down.points);
    }

    if (filtered.empty()) {
      return;
    }

    std::fill(sum_bins_.begin(), sum_bins_.end(), 0.0f);
    std::fill(count_bins_.begin(), count_bins_.end(), 0U);
    std::fill(
      heightmap_buffer_.begin(), heightmap_buffer_.end(),
      std::numeric_limits<float>::quiet_NaN());

    const Eigen::Isometry3d T_cam_map = tf2::transformToEigen(tf_cam_map);
    const Eigen::Isometry3d T_cam_base = tf2::transformToEigen(tf_cam_base);
    const Eigen::Isometry3d T_base_map = tf2::transformToEigen(tf_base_map);

    for (const auto & p : filtered) {
      const Eigen::Vector3d p_cam(p.x, p.y, p.z);
      const Eigen::Vector3d p_base = T_cam_base * p_cam;

      const double x = p_base.x();
      const double y = p_base.y();
      if (x <= 0.0) {
        continue;
      }
      if (y < y_min_ || y >= (y_min_ + y_width_)) {
        continue;
      }

      const int ix = static_cast<int>(std::floor(x / resolution_));
      const int iy = static_cast<int>(std::floor((y - y_min_) / resolution_));
      if (ix < 0 || ix >= grid_cols_ || iy < 0 || iy >= grid_rows_) {
        continue;
      }

      const Eigen::Vector3d p_map = T_cam_map * p_cam;
      const float z = static_cast<float>(p_map.z());
      const std::size_t flat = static_cast<std::size_t>(iy) * grid_cols_ +
        static_cast<std::size_t>(ix);
      sum_bins_[flat] += z;
      count_bins_[flat] += 1U;
    }

    std::size_t filled_cells = 0;
    for (int iy = 0; iy < grid_rows_; ++iy) {
      for (int ix = 0; ix < grid_cols_; ++ix) {
        const std::size_t flat = static_cast<std::size_t>(iy) * grid_cols_ +
          static_cast<std::size_t>(ix);
        const auto count = count_bins_[flat];
        if (count == 0U) {
          continue;
        }
        ++filled_cells;
        const float mean = sum_bins_[flat] / static_cast<float>(count);
        const int flip_r = grid_rows_ - 1 - iy;
        const int flip_c = grid_cols_ - 1 - ix;
        const std::size_t flip_idx =
          static_cast<std::size_t>(flip_r) * grid_cols_ +
          static_cast<std::size_t>(flip_c);
        heightmap_buffer_[flip_idx] = mean;
      }
    }

    publishGridMap(T_base_map, tf_base_map.transform.rotation, msg->header.stamp);

    RCLCPP_DEBUG(
      this->get_logger(), "pc2 -> heightmap: pts %zu, cells %zu / %zu",
      filtered.size(), filled_cells, total_cells_);
  }

  void publishGridMap(
    const Eigen::Isometry3d & T_base_map,
    const geometry_msgs::msg::Quaternion & orientation,
    const rclcpp::Time & stamp)
  {
    const Eigen::Vector3d center_base(x_forward_ / 2.0, y_width_ / 2.0 + y_min_, 0.0);
    const Eigen::Vector3d center_map = T_base_map * center_base;

    grid_map_msgs::msg::GridMap msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;

    msg.info.resolution = resolution_;
    msg.info.length_x = x_forward_;
    msg.info.length_y = y_width_;
    msg.info.pose.position.x = center_map.x();
    msg.info.pose.position.y = center_map.y();
    msg.info.pose.position.z = center_map.z();
    msg.info.pose.orientation = orientation;

    msg.layers = {layer_name_};
    msg.basic_layers = {layer_name_};

    msg.data.resize(1);
    auto & arr = msg.data.front();
    arr.layout.dim.resize(2);
    arr.layout.dim[0].label = "column_index";
    arr.layout.dim[0].size = static_cast<uint32_t>(grid_rows_);
    arr.layout.dim[0].stride = static_cast<uint32_t>(grid_rows_ * grid_cols_);
    arr.layout.dim[1].label = "row_index";
    arr.layout.dim[1].size = static_cast<uint32_t>(grid_cols_);
    arr.layout.dim[1].stride = static_cast<uint32_t>(grid_cols_);
    arr.data = heightmap_buffer_;

    msg.outer_start_index = 0;
    msg.inner_start_index = 0;

    gridmap_pub_->publish(msg);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::string cloud_topic_;
  std::string output_topic_;
  std::string base_frame_;
  std::string map_frame_;
  std::string layer_name_;
  double x_forward_;
  double y_width_;
  double resolution_;
  double voxel_size_;
  double roi_z_max_;
  double y_min_;
  int grid_cols_;
  int grid_rows_;
  std::size_t total_cells_;
  const rclcpp::Duration tf_timeout_{rclcpp::Duration::from_seconds(0.1)};

  std::vector<float> sum_bins_;
  std::vector<uint32_t> count_bins_;
  std::vector<float> heightmap_buffer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pc2ToHeightmapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
