#pragma once
#include <cmath>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <rclcpp/rclcpp.hpp>

class GpsConverter {
public:
  explicit GpsConverter(rclcpp::Node * node)
  : node_(node)
  {
    // robot_localization/navsat_transform_node provides a FromLL service that converts
    // WGS84 (lat/lon/alt) to the node's world frame (typically `map`).
    //
    // This is preferred over publishing fake NavSatFix messages into navsat_transform_node.
    from_ll_service_ =
      node_->declare_parameter<std::string>("from_ll_service", "fromLL");
    from_ll_client_ =
      node_->create_client<robot_localization::srv::FromLL>(from_ll_service_);
  }

  bool to_map_pose(double lat, double lon, geometry_msgs::msg::PoseStamped & out,
                   const std::string & map_frame = "map",
                   const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(1.0))
  {
    if (!std::isfinite(lat) || !std::isfinite(lon)) {
      return false;
    }

    const auto timeout_ns = std::chrono::nanoseconds(timeout.nanoseconds());
    if (!from_ll_client_->wait_for_service(timeout_ns)) {
      RCLCPP_WARN(node_->get_logger(),
                  "FromLL service not available: %s",
                  from_ll_service_.c_str());
      return false;
    }

    auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
    req->ll_point.latitude = lat;
    req->ll_point.longitude = lon;
    req->ll_point.altitude = 0.0;

    auto future = from_ll_client_->async_send_request(req);
    if (future.wait_for(timeout_ns) != std::future_status::ready) {
      RCLCPP_WARN(node_->get_logger(),
                  "FromLL request timed out (service=%s)",
                  from_ll_service_.c_str());
      return false;
    }

    auto resp = future.get();
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = node_->now();
    ps.header.frame_id = map_frame;
    ps.pose.position = resp->map_point;
    ps.pose.orientation.w = 1.0;
    out = ps;
    return true;
  }

private:
  rclcpp::Node * node_;
  std::string from_ll_service_;
  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_client_;
};
