#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;

class GpsHeadingGpsNode : public rclcpp::Node
{
public:
  GpsHeadingGpsNode()
      : Node("gps_heading_gps_node")
  {
    // Node clock; for all time operations
    clk_ = this->get_clock();

    last_gps_time_ = rclcpp::Time(0, 0, clk_->get_clock_type());
    last_north_recv_time_ = rclcpp::Time(0, 0, clk_->get_clock_type());
    last_south_recv_time_ = rclcpp::Time(0, 0, clk_->get_clock_type());
    last_north_time_used_ = rclcpp::Time(0, 0, clk_->get_clock_type());
    last_south_time_used_ = rclcpp::Time(0, 0, clk_->get_clock_type());

    // ROS Parameters (can be tuned at runtime)
    yaw_offset_rad_      = declare_parameter<double>("yaw_offset_rad", -M_PI_2); // 90° left-mount (north-south = north)
    max_gps_age_         = declare_parameter<double>("max_gps_age", 1.0);     // [s] Ignore old GNSS yaw
    max_pair_skew_       = declare_parameter<double>("max_pair_skew", 0.2);   // [s] Threshold of simultaneity
    min_baseline_xy_     = declare_parameter<double>("min_baseline_xy", 0.4); // [m] Minimum allowed distance between GPS RX to reject huge error
    zero_alt_for_heading_= declare_parameter<bool>("zero_alt_for_heading", true); // True to ignore altitude difference / altitude noise; Earth is too big for this
    baseline_direction_  = declare_parameter<int>("baseline_direction", +1);  // +1: (north - south), -1 flips

    auto qos = rclcpp::SensorDataQoS(); // QoS tuned for sensors

    gps_north_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "rover_north/fix", qos,
        std::bind(&GpsHeadingGpsNode::northCallback, this, _1));

    gps_south_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "rover_south/fix", qos,
        std::bind(&GpsHeadingGpsNode::southCallback, this, _1));

    gps_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry/gps/raw", qos,
        std::bind(&GpsHeadingGpsNode::gpsOdomCallback, this, _1));

    gps_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/gps", qos);
  }

private:
  // Node clock
  rclcpp::Clock::SharedPtr clk_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_north_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_south_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gps_odom_pub_;

  // Cache last messages
  sensor_msgs::msg::NavSatFix::SharedPtr last_north_fix_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_south_fix_;

  // GNSS yaw cache
  double last_gps_yaw_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Time last_gps_time_; // default to zero
  rclcpp::Time last_north_recv_time_;
  rclcpp::Time last_south_recv_time_;
  rclcpp::Time last_north_time_used_;
  rclcpp::Time last_south_time_used_;

  // ROS Parameters
  double yaw_offset_rad_;
  double max_gps_age_;
  double max_pair_skew_;
  double min_baseline_xy_;
  bool   zero_alt_for_heading_;
  int    baseline_direction_; // +1/-1

  // WGS84 ellipsoid constants
  static constexpr double a_  = 6378137.0;            // semi-major axis
  static constexpr double e2_ = 6.69437999014e-3;     // eccentricity squared

  // Callbacks
  void northCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    last_north_fix_ = msg;
    last_north_recv_time_ = clk_->now();
    tryUpdateGpsYaw();
  }

  void southCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    last_south_fix_ = msg;
    last_south_recv_time_ = clk_->now();
    tryUpdateGpsYaw();
  }

  void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!std::isfinite(last_gps_yaw_)) return;
    const rclcpp::Time now = clk_->now();
    const double age = (now - last_gps_time_).seconds();
    if (age > max_gps_age_) return;

    double roll = 0.0, pitch = 0.0, yaw_unused = 0.0;
    tf2::Quaternion q_in;
    tf2::fromMsg(msg->pose.pose.orientation, q_in);
    tf2::Matrix3x3(q_in).getRPY(roll, pitch, yaw_unused);

    tf2::Quaternion q_new;
    q_new.setRPY(roll, pitch, last_gps_yaw_);

    auto odom_out = *msg;
    odom_out.pose.pose.orientation = tf2::toMsg(q_new);
    gps_odom_pub_->publish(odom_out);
  }

  // Compute or store GNSS yaw if both north and south fixes are available and healthy
  void tryUpdateGpsYaw()
  {
    // 0. Need both fixes and valid stamps
    if (!last_north_fix_ || !last_south_fix_) return;
    const auto &N = *last_north_fix_;
    const auto &S = *last_south_fix_;
    const rclcpp::Time tN = last_north_recv_time_;
    const rclcpp::Time tS = last_south_recv_time_;
    const rclcpp::Time zero(0, 0, clk_->get_clock_type());
    if (tN <= zero || tS <= zero) return;

    // 0.a Basic validity (finite lat/lon; optional: check status/covariance type)
    if (!std::isfinite(N.latitude) || !std::isfinite(N.longitude) ||
        !std::isfinite(S.latitude) || !std::isfinite(S.longitude)) return;

    // 0.b Enforce small pair skew (they should be nearly simultaneous)
    if (std::abs((tN - tS).seconds()) > max_pair_skew_) return;
    if (tN <= last_north_time_used_ || tS <= last_south_time_used_) return;

    // 1. Convert geodetic to ECEF (optionally zero altitude to reduce vertical noise)
    const double altN = zero_alt_for_heading_ ? 0.0 : N.altitude;
    const double altS = zero_alt_for_heading_ ? 0.0 : S.altitude;

    Eigen::Vector3d ecef_n = geodeticToECEF(N.latitude, N.longitude, altN);
    Eigen::Vector3d ecef_s = geodeticToECEF(S.latitude, S.longitude, altS);

    // 2. Compute baseline in ECEF and map to ENU at reference (north antenna)
    //    NOTE: if your mounting uses the opposite direction, set baseline_direction_ = -1
    Eigen::Vector3d diff_ecef = baseline_direction_ * (ecef_n - ecef_s);
    Eigen::Vector3d enu = ecefToENU(diff_ecef, N.latitude, N.longitude);

    // 3. Guard on horizontal baseline length
    const double baseline_xy = std::hypot(enu.x(), enu.y());
    if (baseline_xy < min_baseline_xy_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Baseline too small for reliable heading (%.3f m < %.3f m)",
        baseline_xy, min_baseline_xy_);
      return;
    }

    // 4. Calculate world yaw from baseline and apply mount offset (e.g., -π/2 for left-mount)
    const double yaw_world = std::atan2(enu.y(), enu.x());
    const double yaw_body  = wrap(yaw_world + yaw_offset_rad_);

    // 5. Store (do not publish here). Use the more recent of the two GPS times.
    last_gps_yaw_  = yaw_body;
    last_gps_time_ = (tN > tS) ? tN : tS;
    last_north_time_used_ = tN;
    last_south_time_used_ = tS;
  }

  // Math helper functions:

  // Wrap angle to (-pi, pi]
  static double wrap(double a) {
    while (a <= -M_PI) a += 2.0*M_PI;
    while (a >   M_PI) a -= 2.0*M_PI;
    return a;
  }

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
  rclcpp::init(argc, argv); // init rclcpp
  rclcpp::spin(std::make_shared<GpsHeadingGpsNode>()); // share pointer, start loop
  rclcpp::shutdown();
  return 0;
}
