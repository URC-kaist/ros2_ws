#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;

class GpsHeadingImuNode : public rclcpp::Node
{
public:
  GpsHeadingImuNode()
      : Node("gps_heading_imu_node")
  {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/local_data", 10,
        std::bind(&GpsHeadingImuNode::imuCallback, this, _1));

    gps_north_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/rover_north/fix", 10,
        std::bind(&GpsHeadingImuNode::northCallback, this, _1));

    gps_south_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/rover_south/fix", 10,
        std::bind(&GpsHeadingImuNode::southCallback, this, _1));

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  }

private:
  // Last messages
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_north_fix_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_south_fix_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_north_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_south_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = msg;
    tryPublish();
  }

  void northCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_north_fix_ = msg;
    tryPublish();
  }

  void southCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_south_fix_ = msg;
    tryPublish();
  }

  void tryPublish()
  {
    if (!last_imu_ || !last_north_fix_ || !last_south_fix_)
    {
      return;
    }

    // 1. Convert lat/lon/alt to ECEF for both antennas
    Eigen::Vector3d ecef_n = geodeticToECEF(
        last_north_fix_->latitude,
        last_north_fix_->longitude,
        last_north_fix_->altitude);
    Eigen::Vector3d ecef_s = geodeticToECEF(
        last_south_fix_->latitude,
        last_south_fix_->longitude,
        last_south_fix_->altitude);

    // 2. Compute baseline in ENU at reference pos (north antenna)
    Eigen::Vector3d diff_ecef = ecef_n - ecef_s;
    Eigen::Vector3d enu = ecefToENU(
        diff_ecef,
        last_north_fix_->latitude,
        last_north_fix_->longitude);

    // 3. Calculate yaw from baseline
    double yaw = std::atan2(enu.y(), enu.x());

    // 4. Extract roll & pitch from incoming IMU
    tf2::Quaternion q_orig;
    tf2::fromMsg(last_imu_->orientation, q_orig);
    double roll, pitch, yaw_orig;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw_orig);

    // 5. Create new quaternion with fused yaw
    tf2::Quaternion q_new;
    q_new.setRPY(roll, pitch, yaw);

    // 6. Publish updated IMU
    auto imu_out = *last_imu_;
    imu_out.orientation = tf2::toMsg(q_new);
    imu_out.header.stamp = this->now();
    imu_pub_->publish(imu_out);
  }

  // WGS84 ellipsoid constants
  static constexpr double a_ = 6378137.0; // Semi major axis
  static constexpr double e2_ = 6.69437999014e-3; // e^2 = 1-b^2/a^2

  // Convert geodetic to ECEF
  Eigen::Vector3d geodeticToECEF(double lat_deg, double lon_deg, double alt)
  {
    double lat = lat_deg * M_PI / 180.0; // in radian
    double lon = lon_deg * M_PI / 180.0;
    double N = a_ / std::sqrt(1 - e2_ * std::sin(lat) * std::sin(lat));
    double Xe = (N + alt) * std::cos(lat) * std::cos(lon);
    double Ye = (N + alt) * std::cos(lat) * std::sin(lon);
    double Ze = ((1 - e2_) * N + alt) * std::sin(lat);
    return Eigen::Vector3d(Xe, Ye, Ze);
  }

  // Convert diff in ECEF to ENU at reference geodetic location
  Eigen::Vector3d ecefToENU(const Eigen::Vector3d &diff,
                            double ref_lat_deg,
                            double ref_lon_deg)
  {
    double p = ref_lat_deg * M_PI / 180.0; // latitude (phi) in radian
    double l = ref_lon_deg * M_PI / 180.0; // longitude (lambda)
    Eigen::Matrix3d R;
    // https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates
    R <<                -std::sin(l),                std::cos(l),           0,
          -std::sin(p) * std::cos(l), -std::sin(p) * std::sin(l), std::cos(p),
           std::cos(p) * std::cos(l),  std::cos(p) * std::sin(l), std::sin(p);
    return R * diff;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsHeadingImuNode>());
  rclcpp::shutdown();
  return 0;
}
