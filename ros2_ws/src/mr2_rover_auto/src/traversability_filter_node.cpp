#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <filters/filter_chain.hpp>
#include <pluginlib/class_loader.hpp>

#include <functional>
#include <memory>
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
    filter_chain_("grid_map::GridMap")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/height_gridmap");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/traversability_gridmap");
    filter_chain_prefix_ = this->declare_parameter<std::string>("filter_chain_prefix", "filters");

    if (!filter_chain_.configure(
        filter_chain_prefix_, this->get_node_logging_interface(),
        this->get_node_parameters_interface()))
    {
      throw std::runtime_error("Failed to configure filter chain.");
    }

    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic_, rclcpp::QoS(1),
      std::bind(&TraversabilityFilterNode::gridMapCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic_, rclcpp::QoS(1));

    RCLCPP_INFO(
      this->get_logger(), "GridMap filter chain listening on %s -> %s (prefix=%s)",
      input_topic_.c_str(), output_topic_.c_str(), filter_chain_prefix_.c_str());
  }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap input_map;
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, input_map)) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert GridMap message to object.");
      return;
    }

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
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string filter_chain_prefix_;
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
