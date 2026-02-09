#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <filters/filter_chain.hpp>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace
{

class TraversabilityFilterNode : public rclcpp::Node
{
public:
  TraversabilityFilterNode()
  : rclcpp::Node(
      "grid_map_filters",
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)),
    filter_chain_("grid_map::GridMap"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/height_gridmap");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/traversability_gridmap");
    filter_chain_prefix_ = this->declare_parameter<std::string>("filter_chain_prefix", "filters");
    gravity_source_ = this->declare_parameter<std::string>("gravity_source", "imu");
    gravity_topic_ = this->declare_parameter<std::string>("gravity_topic", "gravity");
    gravity_reference_frame_ = this->declare_parameter<std::string>("gravity_reference_frame", "map");
    gravity_target_frame_ = this->declare_parameter<std::string>("gravity_target_frame", "base_link");

    if (!filter_chain_.configure(
        filter_chain_prefix_, this->get_node_logging_interface(),
        this->get_node_parameters_interface()))
    {
      throw std::runtime_error("Failed to configure filter chain.");
    }

    auto qos = rclcpp::SensorDataQoS();  // Match depth→height publisher and Nav2 consumers.
    if (gravity_source_ == "imu") {
      gravity_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        gravity_topic_, qos,
        std::bind(&TraversabilityFilterNode::gravityCallback, this, std::placeholders::_1));
    }
    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, qos,
      std::bind(&TraversabilityFilterNode::gridMapCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic_, qos);

    RCLCPP_INFO(
      this->get_logger(), "GridMap filter chain listening on %s -> %s (prefix=%s)",
      input_topic_.c_str(), output_topic_.c_str(), filter_chain_prefix_.c_str());
  }

private:
  void gravityCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    geometry_msgs::msg::Vector3Stamped accel_in;
    accel_in.header = msg->header;
    accel_in.vector = msg->linear_acceleration;

    geometry_msgs::msg::Vector3Stamped accel_out;
    if (accel_in.header.frame_id.empty() ||
      accel_in.header.frame_id == gravity_target_frame_)
    {
      accel_out = accel_in;
    } else {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          gravity_target_frame_, accel_in.header.frame_id,
          accel_in.header.stamp, tf_timeout_);
        tf2::doTransform(accel_in, accel_out, tf);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Gravity TF lookup failed: %s", ex.what());
        return;
      }
    }

    const auto & a = accel_out.vector;
    const double norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (norm < 1e-6) {
      return;
    }
    const std::array<double, 3> g = {a.x / norm, a.y / norm, a.z / norm};
    {
      std::lock_guard<std::mutex> lock(gravity_mutex_);
      gravity_up_ = g;
    }
  }

  void injectGravityLayers(grid_map::GridMap & map)
  {
    std::array<double, 3> g = gravityFromSource(map.getTimestamp());

    if (!map.exists("gravity_x")) {
      map.add("gravity_x");
    }
    if (!map.exists("gravity_y")) {
      map.add("gravity_y");
    }
    if (!map.exists("gravity_z")) {
      map.add("gravity_z");
    }

    map["gravity_x"].setConstant(static_cast<float>(g[0]));
    map["gravity_y"].setConstant(static_cast<float>(g[1]));
    map["gravity_z"].setConstant(static_cast<float>(g[2]));
  }

  std::array<double, 3> gravityFromSource(const int64_t stamp_ns)
  {
    if (gravity_source_ != "tf") {
      std::lock_guard<std::mutex> lock(gravity_mutex_);
      return gravity_up_;
    }

    rclcpp::Time stamp(stamp_ns, this->get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      stamp = this->get_clock()->now();
    }

    if (gravity_reference_frame_.empty() ||
      gravity_reference_frame_ == gravity_target_frame_)
    {
      const std::array<double, 3> g = {0.0, 0.0, 1.0};
      std::lock_guard<std::mutex> lock(gravity_mutex_);
      gravity_up_ = g;
      return g;
    }

    geometry_msgs::msg::Vector3Stamped up_in;
    up_in.header.frame_id = gravity_reference_frame_;
    up_in.header.stamp = stamp;
    up_in.vector.x = 0.0;
    up_in.vector.y = 0.0;
    up_in.vector.z = 1.0;

    geometry_msgs::msg::Vector3Stamped up_out;
    try {
      const auto tf = tf_buffer_.lookupTransform(
        gravity_target_frame_, gravity_reference_frame_, stamp, tf_timeout_);
      tf2::doTransform(up_in, up_out, tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Gravity TF lookup failed: %s", ex.what());
      std::lock_guard<std::mutex> lock(gravity_mutex_);
      return gravity_up_;
    }

    const auto & v = up_out.vector;
    const double norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (norm < 1e-6) {
      std::lock_guard<std::mutex> lock(gravity_mutex_);
      return gravity_up_;
    }
    const std::array<double, 3> g = {v.x / norm, v.y / norm, v.z / norm};
    {
      std::lock_guard<std::mutex> lock(gravity_mutex_);
      gravity_up_ = g;
    }
    return g;
  }

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap input_map;
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, input_map)) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert GridMap message to object.");
      return;
    }

    // Provide gravity direction (unit vector) for slope computation.
    injectGravityLayers(input_map);

    grid_map::GridMap output_map;
    if (!filter_chain_.update(input_map, output_map)) {
      RCLCPP_WARN(this->get_logger(), "Filter chain update failed.");
      return;
    }

    auto out_msg = grid_map::GridMapRosConverter::toMessage(output_map);
    if (!out_msg) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert filtered GridMap to message.");
      return;
    }

    out_msg->header.stamp = msg->header.stamp;
    out_msg->header.frame_id = msg->header.frame_id;
    // Preserve orientation and elevation offset that GridMap geometry does not track.
    out_msg->info.pose.orientation = msg->info.pose.orientation;
    out_msg->info.pose.position.z = msg->info.pose.position.z;

    pub_->publish(std::move(*out_msg));
  }

  filters::FilterChain<grid_map::GridMap> filter_chain_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gravity_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string filter_chain_prefix_;
  std::string gravity_source_;
  std::string gravity_topic_;
  std::string gravity_reference_frame_;
  std::string gravity_target_frame_;
  const rclcpp::Duration tf_timeout_{rclcpp::Duration::from_seconds(0.05)};

  std::mutex gravity_mutex_;
  std::array<double, 3> gravity_up_{0.0, 0.0, 1.0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TraversabilityFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
