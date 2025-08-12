#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GpsConverter {
public:
  GpsConverter(rclcpp::Node* node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    fix_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("query/fix", 1);
    gps_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "query/gps", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        last_gps_ = *msg;
      });
  }

  // Returns true on success
  bool to_map_pose(double lat, double lon, geometry_msgs::msg::PoseStamped &out,
                   const std::string &map_frame = "map",
                   const std::string &base_frame = "base_link",
                   const rclcpp::Duration &timeout = rclcpp::Duration::from_seconds(1.0))
  {
    // 1) Publish one-shot fix
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = node_->now();         // stamp used to correlate
    fix.header.frame_id = "gps_north_link";  // any; navsat_transform ignores it
    fix.latitude  = lat;
    fix.longitude = lon;
    fix.altitude  = 0.0; // ignored if zero_altitude true; else set if known
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix_pub_->publish(fix);

    // 2) Wait for query/gps to update (Odometry in ENU, usually in map/odom frame)
    auto start = node_->now();
    nav_msgs::msg::Odometry odom;
    bool got = false;
    while (rclcpp::ok() && (node_->now() - start) < timeout) {
      if (rclcpp::Time(last_gps_.header.stamp) >= rclcpp::Time(fix.header.stamp)) {
        odom = last_gps_; got = true; break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(10));
      rclcpp::spin_some(node_->get_node_base_interface());
    }
    if (!got) { RCLCPP_WARN(node_->get_logger(), "GPS query timeout"); return false; }

    // 3) Normalize to PoseStamped in map frame
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;  // frame is set by navsat_transform (e.g., "map")
    ps.pose   = odom.pose.pose;

    if (ps.header.frame_id != map_frame) {
      // Transform pose to map
      try {
        geometry_msgs::msg::TransformStamped T =
          tf_buffer_.lookupTransform(map_frame, ps.header.frame_id, ps.header.stamp, timeout);
        tf2::doTransform(ps, out, T);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "TF error: %s", ex.what());
        return false;
      }
    } else {
      out = ps;
    }
    // Orientation: keep yaw from current robot heading if desired; else zero
    return true;
  }

private:
  rclcpp::Node* node_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  nav_msgs::msg::Odometry last_gps_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
