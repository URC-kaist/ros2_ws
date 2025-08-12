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
  explicit GpsConverter(rclcpp::Node * node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_)
  {
    fix_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("query/fix", 1);
    gps_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "query/gps", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { last_gps_ = *msg; have_gps_ = true; });
  }

  bool to_map_pose(double lat, double lon, geometry_msgs::msg::PoseStamped & out,
                   const std::string & map_frame = "map",
                   const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(1.0))
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
    const auto t0 = node_->now();
    while (rclcpp::ok() && (node_->now() - t0) < timeout) {
      if (have_gps_ && rclcpp::Time(last_gps_.header.stamp) >= rclcpp::Time(fix.header.stamp)) break;
      rclcpp::sleep_for(std::chrono::milliseconds(10));
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_->shared_from_this());
      exec.spin_some();
    }
    if (!have_gps_ || rclcpp::Time(last_gps_.header.stamp) < rclcpp::Time(fix.header.stamp)) {
      RCLCPP_WARN(node_->get_logger(), "GPS query timeout");
      return false;
    }
    
    // 3) Normalize to PoseStamped in map frame
    geometry_msgs::msg::PoseStamped ps;
    ps.header = last_gps_.header;
    ps.pose   = last_gps_.pose.pose;

    if (ps.header.frame_id == map_frame) {
      out = ps;
      return true;
    }
    try {
      auto T = tf_buffer_.lookupTransform(map_frame, ps.header.frame_id, ps.header.stamp, timeout);
      tf2::doTransform(ps, out, T);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "TF error: %s", ex.what());
      return false;
    }
    return true;
  }

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
  nav_msgs::msg::Odometry last_gps_;
  bool have_gps_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
