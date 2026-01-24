#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geographic_msgs/msg/geo_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/set_datum.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_svin.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);
constexpr double kWgs84E2 = 1.0 - (kWgs84B * kWgs84B) / (kWgs84A * kWgs84A);

struct Llh {
  double lat_deg;
  double lon_deg;
  double alt_m;
};

Llh ecef_to_llh(double x, double y, double z) {
  const double p = std::hypot(x, y);
  double lat = std::atan2(z, p * (1.0 - kWgs84E2));
  for (int i = 0; i < 10; ++i) {
    const double sin_lat = std::sin(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    const double h = p / std::cos(lat) - n;
    const double next_lat =
        std::atan2(z, p * (1.0 - (kWgs84E2 * n) / (n + h)));
    if (std::fabs(next_lat - lat) < 1e-12) {
      lat = next_lat;
      break;
    }
    lat = next_lat;
  }
  const double sin_lat = std::sin(lat);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  const double h = p / std::cos(lat) - n;
  const double lon = std::atan2(y, x);
  return {
      lat * 180.0 / kPi,
      lon * 180.0 / kPi,
      h,
  };
}

}  // namespace

class BaseDatumSetter : public rclcpp::Node {
 public:
  BaseDatumSetter()
      : rclcpp::Node("base_datum_setter"),
        svin_topic_(
            declare_parameter<std::string>("svin_topic", "/base/ubx_nav_svin")),
        require_svin_complete_(
            declare_parameter<bool>("require_svin_complete", true)),
        allow_provisional_(
            declare_parameter<bool>("allow_provisional", false)),
        navsat_service_(
            declare_parameter<std::string>("navsat_service",
                                           "/navsat_transform/set_datum")),
        navsat_query_service_(
            declare_parameter<std::string>("navsat_query_service",
                                           "/navsat_transform_query/set_datum")) {
    navsat_client_ = create_client<robot_localization::srv::SetDatum>(
        navsat_service_);
    navsat_query_client_ = create_client<robot_localization::srv::SetDatum>(
        navsat_query_service_);

    auto svin_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    svin_sub_ = create_subscription<ublox_ubx_msgs::msg::UBXNavSvin>(
        svin_topic_, svin_qos,
        [this](const ublox_ubx_msgs::msg::UBXNavSvin::SharedPtr msg) {
          handle_svin_(msg);
        });

    retry_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() { try_set_datum_(); });
  }

 private:
  void handle_svin_(const ublox_ubx_msgs::msg::UBXNavSvin::SharedPtr &msg) {
    if (!msg || datum_set_) {
      return;
    }
    if (!msg->valid && !allow_provisional_) {
      return;
    }
    if (require_svin_complete_ && msg->active) {
      return;
    }

    const double mean_x = static_cast<double>(msg->mean_x);
    const double mean_y = static_cast<double>(msg->mean_y);
    const double mean_z = static_cast<double>(msg->mean_z);
    const double mean_x_hp = static_cast<double>(msg->mean_x_hp);
    const double mean_y_hp = static_cast<double>(msg->mean_y_hp);
    const double mean_z_hp = static_cast<double>(msg->mean_z_hp);

    const double x_m = (mean_x + 0.01 * mean_x_hp) / 100.0;
    const double y_m = (mean_y + 0.01 * mean_y_hp) / 100.0;
    const double z_m = (mean_z + 0.01 * mean_z_hp) / 100.0;
    const auto llh = ecef_to_llh(x_m, y_m, z_m);

    if (!navsat_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for %s service", navsat_service_.c_str());
      return;
    }
    if (!navsat_query_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for %s service",
                           navsat_query_service_.c_str());
      return;
    }

    latest_llh_ = llh;
    latest_valid_ = msg->valid;
    latest_active_ = msg->active;
    if (!svin_seen_) {
      svin_seen_ = true;
      RCLCPP_INFO(get_logger(),
                  "Received base SVIN (valid=%s active=%s); waiting to set datum",
                  msg->valid ? "true" : "false",
                  msg->active ? "true" : "false");
    }
    try_set_datum_();
  }

  void try_set_datum_() {
    if (datum_set_ || !latest_llh_) {
      return;
    }
    if (!navsat_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for %s service", navsat_service_.c_str());
      return;
    }
    if (!navsat_query_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for %s service",
                           navsat_query_service_.c_str());
      return;
    }

    auto req = std::make_shared<robot_localization::srv::SetDatum::Request>();
    req->geo_pose.position.latitude = latest_llh_->lat_deg;
    req->geo_pose.position.longitude = latest_llh_->lon_deg;
    req->geo_pose.position.altitude = latest_llh_->alt_m;
    req->geo_pose.orientation.w = 1.0;

    navsat_client_->async_send_request(req);
    navsat_query_client_->async_send_request(req);

    datum_set_ = true;
    RCLCPP_INFO(get_logger(),
                "Datum set from base survey-in (valid=%s active=%s): lat=%.8f lon=%.8f alt=%.3f",
                latest_valid_ ? "true" : "false",
                latest_active_ ? "true" : "false",
                latest_llh_->lat_deg, latest_llh_->lon_deg, latest_llh_->alt_m);
  }

  std::string svin_topic_;
  bool require_svin_complete_;
  bool allow_provisional_;
  std::string navsat_service_;
  std::string navsat_query_service_;
  bool datum_set_{false};
  bool svin_seen_{false};
  std::optional<Llh> latest_llh_;
  bool latest_valid_{false};
  bool latest_active_{false};

  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavSvin>::SharedPtr svin_sub_;
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr navsat_client_;
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr
      navsat_query_client_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseDatumSetter>());
  rclcpp::shutdown();
  return 0;
}
