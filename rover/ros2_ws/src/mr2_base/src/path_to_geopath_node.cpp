#include <GeographicLib/UTMUPS.hpp> //sudo apt install libgeographic-dev geographiclib-tools

#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "geographic_msgs/msg/geo_path.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PathToGeoPathNode : public rclcpp::Node {
public:
  PathToGeoPathNode()
  : rclcpp::Node("path_to_geopath"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "plan_smoothed");
    output_topic_ = declare_parameter<std::string>("output_topic", "plan_smoothed_geo");
    gps_topic_ = declare_parameter<std::string>("gps_topic", "gps/filtered");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    utm_frame_ = declare_parameter<std::string>("utm_frame", "utm");
    wgs84_frame_ = declare_parameter<std::string>("wgs84_frame", "wgs84");
    use_transient_local_ = declare_parameter<bool>("transient_local", false);

    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    if (use_transient_local_) {
      path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    }

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      input_topic_,
      path_qos,
      std::bind(&PathToGeoPathNode::handlePath, this, std::placeholders::_1));

    pub_ = create_publisher<geographic_msgs::msg::GeoPath>(output_topic_, path_qos);

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PathToGeoPathNode::handleFix, this, std::placeholders::_1));
  }

private:
  void handleFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return;
    }
    const double lat = msg->latitude;
    const double lon = msg->longitude;
    if (!std::isfinite(lat) || !std::isfinite(lon)) {
      return;
    }

    int zone = 0;
    bool northp = true;
    double x = 0.0;
    double y = 0.0;
    double gamma = 0.0;
    double k = 0.0;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y, gamma, k);

    bool updated = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!utm_zone_ || !utm_northp_ || zone != *utm_zone_ || northp != *utm_northp_) {
        utm_zone_ = zone;
        utm_northp_ = northp;
        updated = true;
      }
    }

    if (updated) {
      RCLCPP_INFO(get_logger(), "UTM zone locked: %d (%s hemisphere)", zone,
                  northp ? "north" : "south");
      nav_msgs::msg::Path::SharedPtr pending;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pending = last_path_;
      }
      if (pending) {
        processPath(pending);
      }
    }
  }

  void handlePath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_path_ = msg;
    }
    processPath(msg);
  }

  void processPath(const nav_msgs::msg::Path::SharedPtr & msg)
  {
    int zone = 0;
    bool northp = true;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!utm_zone_ || !utm_northp_) {
        if (!warned_no_fix_) {
          warned_no_fix_ = true;
          RCLCPP_WARN(get_logger(), "Waiting for valid GPS fix on %s to derive UTM zone.",
                      gps_topic_.c_str());
        }
        return;
      }
      zone = *utm_zone_;
      northp = *utm_northp_;
    }

    std::string source_frame = msg->header.frame_id;
    if (source_frame.empty()) {
      source_frame = map_frame_;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(utm_frame_, source_frame, tf2::TimePointZero);
      warned_no_tf_ = false;
    } catch (const tf2::TransformException & ex) {
      if (!warned_no_tf_) {
        warned_no_tf_ = true;
        RCLCPP_WARN(get_logger(), "TF lookup failed (%s -> %s): %s",
                    source_frame.c_str(), utm_frame_.c_str(), ex.what());
      }
      return;
    }

    tf2::Transform map_to_utm;
    tf2::fromMsg(tf.transform, map_to_utm);

    geographic_msgs::msg::GeoPath out;
    out.header = msg->header;
    out.header.frame_id = wgs84_frame_;
    out.poses.reserve(msg->poses.size());

    for (const auto & pose : msg->poses) {
      const auto & position = pose.pose.position;
      tf2::Vector3 map_point(position.x, position.y, position.z);
      tf2::Vector3 utm_point = map_to_utm * map_point;

      double lat = 0.0;
      double lon = 0.0;
      double gamma = 0.0;
      double k = 0.0;
      GeographicLib::UTMUPS::Reverse(zone, northp, utm_point.x(), utm_point.y(),
                                     lat, lon, gamma, k);

      geographic_msgs::msg::GeoPoseStamped geo_pose;
      geo_pose.header = pose.header;
      if (geo_pose.header.frame_id.empty()) {
        geo_pose.header = msg->header;
      }
      geo_pose.header.frame_id = wgs84_frame_;
      geo_pose.pose.position.latitude = lat;
      geo_pose.pose.position.longitude = lon;
      geo_pose.pose.position.altitude = utm_point.z();
      geo_pose.pose.orientation = pose.pose.orientation;
      out.poses.push_back(std::move(geo_pose));
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string gps_topic_;
  std::string map_frame_;
  std::string utm_frame_;
  std::string wgs84_frame_;
  bool use_transient_local_ = false;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPath>::SharedPtr pub_;

  std::mutex state_mutex_;
  std::optional<int> utm_zone_;
  std::optional<bool> utm_northp_;
  nav_msgs::msg::Path::SharedPtr last_path_;
  bool warned_no_fix_ = false;
  bool warned_no_tf_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToGeoPathNode>());
  rclcpp::shutdown();
  return 0;
}
