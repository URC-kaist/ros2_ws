#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mr2_battery_monitor/msg/pack_telemetry.hpp"
#include "mr2_sik_bridge/packets.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_svin.hpp"
#include "rtcm_msgs/msg/message.hpp"

namespace mr2_sik_bridge {

using mr2_sik_bridge::CmdArmTwist;
using mr2_sik_bridge::CmdDrive;
using mr2_sik_bridge::Frame;
using mr2_sik_bridge::Heartbeat;
using mr2_sik_bridge::TelemBattery;
using mr2_sik_bridge::TelemNav;

class SikBridgeNode : public rclcpp::Node {
 public:
  SikBridgeNode()
      : rclcpp::Node("sik_bridge"),
        device_(declare_parameter<std::string>("device", "/dev/ttySIK")),
        baud_(declare_parameter<int>("baud", 57600)),
        heartbeat_timeout_ms_(
            declare_parameter<int>("heartbeat_timeout_ms", 500)),
        zero_publish_rate_hz_(
            declare_parameter<double>("zero_publish_rate_hz", 20.0)),
        battery_tx_rate_hz_(
            declare_parameter<double>("battery_tx_rate_hz", 1.0)),
        heartbeat_tx_rate_hz_(
            declare_parameter<double>("heartbeat_tx_rate_hz", 2.0)),
        nav_tx_rate_hz_(declare_parameter<double>("nav_tx_rate_hz", 2.0)),
        cmd_vel_topic_(
            declare_parameter<std::string>("cmd_vel_topic", "/base/cmd_vel")),
        arm_twist_topic_(declare_parameter<std::string>(
            "arm_twist_topic", "/moveit_servo/delta_twist_cmds")),
        arm_frame_id_(declare_parameter<std::string>("arm_frame_id", "base_link")),
        nav_fix_topic_(
            declare_parameter<std::string>("nav_fix_topic", "/gps/filtered")),
        odom_topic_(declare_parameter<std::string>(
        "odom_topic", "/odometry/filtered/global")),
        battery_1_topic_(declare_parameter<std::string>(
            "battery_1_topic", "battery_1/telemetry")),
        battery_2_topic_(declare_parameter<std::string>(
            "battery_2_topic", "battery_2/telemetry")),
        base_svin_topic_(declare_parameter<std::string>(
            "base_svin_topic", "/base/ubx_nav_svin")),
        base_rtcm_topic_(declare_parameter<std::string>(
            "base_rtcm_topic", "/base/rtcm")),
        log_frames_(declare_parameter<bool>("log_frames", false)) {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    arm_twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(arm_twist_topic_, 10);
    base_svin_pub_ = create_publisher<ublox_ubx_msgs::msg::UBXNavSvin>(
        base_svin_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    base_rtcm_pub_ = create_publisher<rtcm_msgs::msg::Message>(
        base_rtcm_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    battery_sub_1_ = create_subscription<mr2_battery_monitor::msg::PackTelemetry>(
        battery_1_topic_, rclcpp::SensorDataQoS(),
        [this](const mr2_battery_monitor::msg::PackTelemetry::SharedPtr msg) {
          battery_cb(msg, 1);
        });
    battery_sub_2_ = create_subscription<mr2_battery_monitor::msg::PackTelemetry>(
        battery_2_topic_, rclcpp::SensorDataQoS(),
        [this](const mr2_battery_monitor::msg::PackTelemetry::SharedPtr msg) {
          battery_cb(msg, 2);
        });

    nav_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        nav_fix_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          nav_fix_cb_(msg);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_cb_(msg); });

    open_serial_();
    start_reader_();

    const auto zero_period =
        std::chrono::duration<double>(1.0 / std::max(1.0, zero_publish_rate_hz_));
    zero_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(zero_period),
        std::bind(&SikBridgeNode::zero_check, this));

    if (heartbeat_tx_rate_hz_ > 0.0) {
      const auto heartbeat_period = std::chrono::duration<double>(
          1.0 / std::max(heartbeat_tx_rate_hz_, 0.1));
      heartbeat_tx_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
          std::bind(&SikBridgeNode::send_heartbeat_, this));
    }

    if (nav_tx_rate_hz_ > 0.0) {
      const auto nav_period =
          std::chrono::duration<double>(1.0 / std::max(nav_tx_rate_hz_, 0.1));
      nav_tx_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(nav_period),
          std::bind(&SikBridgeNode::send_nav_, this));
    }

    last_heartbeat_ = now();
    last_battery_tx_[0] = now() - rclcpp::Duration::from_seconds(10.0);
    last_battery_tx_[1] = now() - rclcpp::Duration::from_seconds(10.0);
  }

  ~SikBridgeNode() override {
    shutting_down_ = true;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

 private:
  void open_serial_() {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open serial device " + device_);
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      throw std::runtime_error("Failed to get serial attributes");
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    const speed_t speed = baud_to_speed_(baud_);
    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
      throw std::runtime_error("Failed to set serial baud rate");
    }

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      throw std::runtime_error("Failed to apply serial attributes");
    }
  }

  static speed_t baud_to_speed_(int baud) {
    switch (baud) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:
        return B57600;
    }
  }

  void start_reader_() {
    reader_thread_ = std::thread([this]() { read_loop_(); });
  }

  void read_loop_() {
    std::vector<uint8_t> buffer;
    buffer.reserve(512);

    while (rclcpp::ok() && !shutting_down_) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(fd_, &readfds);
      timeval timeout{};
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;

      const int ready = select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
      if (ready <= 0) {
        continue;
      }

      uint8_t temp[256];
      const ssize_t count = ::read(fd_, temp, sizeof(temp));
      if (count <= 0) {
        continue;
      }

      buffer.insert(buffer.end(), temp, temp + count);
      parse_frames_(buffer);
    }
  }

  void parse_frames_(std::vector<uint8_t> &buffer) {
    while (true) {
      if (buffer.size() < 4) {
        return;
      }

      if (buffer.front() != mr2_sik_bridge::kMagic) {
        buffer.erase(buffer.begin());
        continue;
      }

      const uint8_t length = buffer[2];
      const size_t frame_size = 4 + static_cast<size_t>(length) + 2;
      if (buffer.size() < frame_size) {
        return;
      }

      auto frame = mr2_sik_bridge::decode_frame(buffer.data(), frame_size);
      if (!frame) {
        if (log_frames_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Invalid SiK frame dropped");
        }
        buffer.erase(buffer.begin());
        continue;
      }

      handle_frame_(*frame);
      buffer.erase(buffer.begin(), buffer.begin() + frame_size);
    }
  }

  void handle_frame_(const Frame &frame) {
    switch (frame.header.msg_id) {
      case mr2_sik_bridge::MsgId::kCmdDrive: {
        auto cmd = mr2_sik_bridge::decode_cmd_drive(frame);
        if (cmd) {
          handle_cmd_drive_(*cmd);
        }
        break;
      }
      case mr2_sik_bridge::MsgId::kCmdArmTwist: {
        auto cmd = mr2_sik_bridge::decode_cmd_arm_twist(frame);
        if (cmd) {
          handle_cmd_arm_(*cmd);
        }
        break;
      }
      case mr2_sik_bridge::MsgId::kHeartbeat: {
        auto hb = mr2_sik_bridge::decode_heartbeat(frame);
        if (hb) {
          last_heartbeat_ = now();
        }
        break;
      }
      case mr2_sik_bridge::MsgId::kBaseSvin: {
        auto svin = mr2_sik_bridge::decode_base_svin(frame);
        if (svin && base_svin_pub_) {
          ublox_ubx_msgs::msg::UBXNavSvin msg;
          msg.header.stamp = now();
          msg.mean_x = svin->mean_x_cm;
          msg.mean_y = svin->mean_y_cm;
          msg.mean_z = svin->mean_z_cm;
          msg.mean_x_hp = svin->mean_x_hp;
          msg.mean_y_hp = svin->mean_y_hp;
          msg.mean_z_hp = svin->mean_z_hp;
          msg.valid = svin->valid;
          msg.active = svin->active;
          msg.mean_acc = svin->mean_acc_0p1mm;
          msg.obs = svin->obs;
          base_svin_pub_->publish(msg);
        }
        break;
      }
      case mr2_sik_bridge::MsgId::kBaseRtcm: {
        auto rtcm = mr2_sik_bridge::decode_base_rtcm(frame);
        if (rtcm && base_rtcm_pub_) {
          rtcm_msgs::msg::Message msg;
          msg.header.stamp = now();
          msg.header.frame_id = "base_rtcm";
          msg.message = rtcm->message;
          base_rtcm_pub_->publish(msg);
        }
        break;
      }
      case mr2_sik_bridge::MsgId::kBaseRtcmFrag: {
        auto frag = mr2_sik_bridge::decode_base_rtcm_frag(frame);
        if (frag && base_rtcm_pub_) {
          handle_base_rtcm_frag_(frame.header.seq, *frag);
        }
        break;
      }
      default:
        break;
    }
  }

  void handle_cmd_drive_(const CmdDrive &cmd) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = cmd.linear_x_m_s;
    msg.linear.y = cmd.linear_y_m_s;
    msg.angular.z = cmd.angular_z_rad_s;
    cmd_vel_pub_->publish(msg);
    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "CMD_DRIVE x=%.3f y=%.3f yaw=%.3f",
                           cmd.linear_x_m_s, cmd.linear_y_m_s,
                           cmd.angular_z_rad_s);
    }
  }

  void handle_cmd_arm_(const CmdArmTwist &cmd) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = arm_frame_id_;
    msg.twist.linear.x = cmd.lin_x_m_s;
    msg.twist.linear.y = cmd.lin_y_m_s;
    msg.twist.linear.z = cmd.lin_z_m_s;
    msg.twist.angular.x = cmd.ang_x_rad_s;
    msg.twist.angular.y = cmd.ang_y_rad_s;
    msg.twist.angular.z = cmd.ang_z_rad_s;
    arm_twist_pub_->publish(msg);
    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "CMD_ARM_TWIST lin=(%.3f, %.3f, %.3f) ang=(%.3f, %.3f, %.3f)",
                           cmd.lin_x_m_s, cmd.lin_y_m_s, cmd.lin_z_m_s,
                           cmd.ang_x_rad_s, cmd.ang_y_rad_s, cmd.ang_z_rad_s);
    }
  }

  void handle_base_rtcm_frag_(uint8_t seq, const BaseRtcmFrag &frag) {
    const auto now_time = now();
    prune_rtcm_frags_(now_time);

    auto &state = rtcm_frags_[seq];
    const bool needs_reset =
        state.parts.empty() || state.msg_len != frag.msg_len ||
        state.total_frags != frag.frag_count;
    if (needs_reset) {
      state.msg_len = frag.msg_len;
      state.total_frags = frag.frag_count;
      state.parts.clear();
      state.parts.resize(frag.frag_count);
      state.start = now_time;
    }

    if (frag.frag_index >= state.parts.size()) {
      return;
    }

    if (state.parts[frag.frag_index].empty()) {
      state.parts[frag.frag_index] = frag.data;
    }

    size_t bytes_accumulated = 0;
    for (const auto &p : state.parts) {
      if (p.empty()) {
        return;
      }
      bytes_accumulated += p.size();
    }

    if (bytes_accumulated != state.msg_len) {
      rtcm_frags_.erase(seq);
      return;
    }

    std::vector<uint8_t> merged;
    merged.reserve(state.msg_len);
    for (const auto &p : state.parts) {
      merged.insert(merged.end(), p.begin(), p.end());
    }

    rtcm_msgs::msg::Message msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_rtcm";
    msg.message = merged;
    base_rtcm_pub_->publish(msg);
    rtcm_frags_.erase(seq);
  }

  void prune_rtcm_frags_(const rclcpp::Time &now_time) {
    auto it = rtcm_frags_.begin();
    while (it != rtcm_frags_.end()) {
      const auto age = now_time - it->second.start;
      if (age.seconds() > 2.0) {
        it = rtcm_frags_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void zero_check() {
    const auto now_time = now();
    const auto elapsed = now_time - last_heartbeat_;
    if (elapsed.seconds() * 1000.0 <= heartbeat_timeout_ms_) {
      return;
    }

    geometry_msgs::msg::Twist zero_drive;
    cmd_vel_pub_->publish(zero_drive);

    geometry_msgs::msg::TwistStamped zero_arm;
    zero_arm.header.stamp = now_time;
    zero_arm.header.frame_id = arm_frame_id_;
    arm_twist_pub_->publish(zero_arm);
  }

  void send_heartbeat_() {
    Heartbeat hb;
    hb.timestamp_ms =
        static_cast<uint32_t>(now().nanoseconds() / 1000000);
    auto frame = mr2_sik_bridge::encode_heartbeat(next_seq_(), hb);
    write_frame_(frame);
  }

  void battery_cb(const mr2_battery_monitor::msg::PackTelemetry::SharedPtr msg,
                  uint8_t battery_id) {
    if (battery_id < 1 || battery_id > 2) {
      return;
    }
    if (battery_tx_rate_hz_ <= 0.0) {
      return;
    }

    const auto now_time = now();
    const double min_period = 1.0 / battery_tx_rate_hz_;
    auto &last_tx = last_battery_tx_[battery_id - 1];
    if ((now_time - last_tx).seconds() < min_period) {
      return;
    }

    const float total_capacity =
        static_cast<float>(msg->nominal_cell_capacity_mah) *
        static_cast<float>(msg->parallel_group_count);
    const float available_capacity =
        total_capacity * (msg->state_of_charge_pct / 100.0f);
    const float temperature_c = std::round(msg->temperature_c);
    const float pack_voltage_v = msg->pack_voltage_v;

    TelemBattery telem;
    telem.total_capacity_mah = total_capacity;
    telem.available_capacity_mah = available_capacity;
    telem.temperature_c = temperature_c;
    telem.pack_voltage_v = pack_voltage_v;

    auto frame =
        mr2_sik_bridge::encode_telem_battery(next_seq_(), telem, battery_id);
    write_frame_(frame);
    last_tx = now_time;
  }

  void nav_fix_cb_(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lock(nav_mutex_);
    nav_lat_deg_ = msg->latitude;
    nav_lon_deg_ = msg->longitude;
    nav_alt_m_ = msg->altitude;
    nav_has_fix_ = std::isfinite(nav_lat_deg_) && std::isfinite(nav_lon_deg_);
  }

  void odom_cb_(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!msg) {
      return;
    }
    const auto &q = msg->pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    const float heading_deg = static_cast<float>(
        std::fmod((yaw * 180.0 / M_PI) + 360.0, 360.0));

    float cov_x = std::numeric_limits<float>::quiet_NaN();
    float cov_y = std::numeric_limits<float>::quiet_NaN();
    float cov_yaw = std::numeric_limits<float>::quiet_NaN();
    const auto &cov = msg->pose.covariance;
    if (cov.size() >= 36) {
      cov_x = static_cast<float>(cov[0]);
      cov_y = static_cast<float>(cov[7]);
      cov_yaw = static_cast<float>(cov[35]);
    }

    std::lock_guard<std::mutex> lock(nav_mutex_);
    nav_heading_deg_ = heading_deg;
    nav_cov_x_var_ = cov_x;
    nav_cov_y_var_ = cov_y;
    nav_cov_yaw_var_ = cov_yaw;
    nav_has_heading_ = std::isfinite(nav_heading_deg_);
  }

  void send_nav_() {
    if (nav_tx_rate_hz_ <= 0.0) {
      return;
    }
    TelemNav nav;
    {
      std::lock_guard<std::mutex> lock(nav_mutex_);
      if (!nav_has_fix_) {
        return;
      }
      nav.timestamp_ms = static_cast<uint32_t>(now().nanoseconds() / 1000000);
      nav.latitude_deg = static_cast<float>(nav_lat_deg_);
      nav.longitude_deg = static_cast<float>(nav_lon_deg_);
      nav.altitude_m = static_cast<float>(nav_alt_m_);
      nav.heading_deg = nav_has_heading_ ? nav_heading_deg_
                                         : std::numeric_limits<float>::quiet_NaN();
      nav.cov_x_var = nav_cov_x_var_;
      nav.cov_y_var = nav_cov_y_var_;
      nav.cov_yaw_var = nav_cov_yaw_var_;
    }

    auto frame = mr2_sik_bridge::encode_telem_nav(next_seq_(), nav);
    write_frame_(frame);
  }

  void write_frame_(const std::vector<uint8_t> &frame) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ < 0) {
      return;
    }
    ssize_t total = 0;
    while (total < static_cast<ssize_t>(frame.size())) {
      const ssize_t written =
          ::write(fd_, frame.data() + total, frame.size() - total);
      if (written <= 0) {
        break;
      }
      total += written;
    }
  }

  uint8_t next_seq_() {
    const uint8_t seq = seq_;
    seq_ = static_cast<uint8_t>(seq_ + 1);
    return seq;
  }

  // Parameters
  std::string device_;
  int baud_;
  int heartbeat_timeout_ms_;
  double zero_publish_rate_hz_;
  double battery_tx_rate_hz_;
  double heartbeat_tx_rate_hz_;
  double nav_tx_rate_hz_;
  std::string cmd_vel_topic_;
  std::string arm_twist_topic_;
  std::string arm_frame_id_;
  std::string nav_fix_topic_;
  std::string odom_topic_;
  std::string battery_1_topic_;
  std::string battery_2_topic_;
  std::string base_svin_topic_;
  std::string base_rtcm_topic_;
  bool log_frames_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
  rclcpp::Publisher<ublox_ubx_msgs::msg::UBXNavSvin>::SharedPtr base_svin_pub_;
  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr base_rtcm_pub_;
  rclcpp::Subscription<mr2_battery_monitor::msg::PackTelemetry>::SharedPtr
      battery_sub_1_;
  rclcpp::Subscription<mr2_battery_monitor::msg::PackTelemetry>::SharedPtr
      battery_sub_2_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr zero_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_tx_timer_;
  rclcpp::TimerBase::SharedPtr nav_tx_timer_;

  // Serial
  int fd_{-1};
  std::thread reader_thread_;
  std::mutex write_mutex_;
  bool shutting_down_{false};
  uint8_t seq_{0};

  // State
  rclcpp::Time last_heartbeat_{};
  std::array<rclcpp::Time, 2> last_battery_tx_{
      {rclcpp::Time(0, 0, RCL_SYSTEM_TIME), rclcpp::Time(0, 0, RCL_SYSTEM_TIME)}};

  struct RtcmFragState {
    uint16_t msg_len{0};
    uint8_t total_frags{0};
    rclcpp::Time start{};
    std::vector<std::vector<uint8_t>> parts;
  };
  std::unordered_map<uint8_t, RtcmFragState> rtcm_frags_;

  std::mutex nav_mutex_;
  bool nav_has_fix_{false};
  bool nav_has_heading_{false};
  double nav_lat_deg_{0.0};
  double nav_lon_deg_{0.0};
  double nav_alt_m_{0.0};
  float nav_heading_deg_{std::numeric_limits<float>::quiet_NaN()};
  float nav_cov_x_var_{std::numeric_limits<float>::quiet_NaN()};
  float nav_cov_y_var_{std::numeric_limits<float>::quiet_NaN()};
  float nav_cov_yaw_var_{std::numeric_limits<float>::quiet_NaN()};
};

}  // namespace mr2_sik_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mr2_sik_bridge::SikBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("sik_bridge"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
