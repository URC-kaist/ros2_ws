#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mr2_battery_monitor/msg/pack_telemetry.hpp"
#include "mr2_sik_bridge/packets.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mr2_sik_bridge {

using mr2_sik_bridge::CmdArmTwist;
using mr2_sik_bridge::CmdDrive;
using mr2_sik_bridge::Frame;
using mr2_sik_bridge::Heartbeat;
using mr2_sik_bridge::TelemBattery;

class SikBridgeNode : public rclcpp::Node {
 public:
  SikBridgeNode()
      : rclcpp::Node("sik_bridge"),
        device_(declare_parameter<std::string>("device", "/dev/ttyUSB0")),
        baud_(declare_parameter<int>("baud", 57600)),
        heartbeat_timeout_ms_(
            declare_parameter<int>("heartbeat_timeout_ms", 500)),
        zero_publish_rate_hz_(
            declare_parameter<double>("zero_publish_rate_hz", 20.0)),
        battery_tx_rate_hz_(
            declare_parameter<double>("battery_tx_rate_hz", 1.0)),
        cmd_vel_topic_(
            declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
        arm_twist_topic_(declare_parameter<std::string>(
            "arm_twist_topic", "/moveit_servo/delta_twist_cmds")),
        arm_frame_id_(declare_parameter<std::string>("arm_frame_id", "base_link")) {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    arm_twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(arm_twist_topic_, 10);

    battery_sub_ = create_subscription<mr2_battery_monitor::msg::PackTelemetry>(
        "battery/telemetry", 10,
        std::bind(&SikBridgeNode::battery_cb, this, std::placeholders::_1));

    open_serial_();
    start_reader_();

    const auto zero_period =
        std::chrono::duration<double>(1.0 / std::max(1.0, zero_publish_rate_hz_));
    zero_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(zero_period),
        std::bind(&SikBridgeNode::zero_check, this));

    last_heartbeat_ = now();
    last_battery_tx_ = now() - rclcpp::Duration::from_seconds(10.0);
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
      default:
        break;
    }
  }

  void handle_cmd_drive_(const CmdDrive &cmd) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = cmd.linear_x_m_s;
    msg.angular.z = cmd.angular_z_rad_s;
    cmd_vel_pub_->publish(msg);
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

  void battery_cb(const mr2_battery_monitor::msg::PackTelemetry::SharedPtr msg) {
    if (battery_tx_rate_hz_ <= 0.0) {
      return;
    }

    const auto now_time = now();
    const double min_period = 1.0 / battery_tx_rate_hz_;
    if ((now_time - last_battery_tx_).seconds() < min_period) {
      return;
    }

    const float total_capacity =
        static_cast<float>(msg->nominal_cell_capacity_mah) *
        static_cast<float>(msg->parallel_group_count);
    const float available_capacity =
        total_capacity * (msg->state_of_charge_pct / 100.0f);
    const float temperature_c = std::round(msg->temperature_c);

    TelemBattery telem;
    telem.total_capacity_mah = total_capacity;
    telem.available_capacity_mah = available_capacity;
    telem.temperature_c = temperature_c;

    auto frame = mr2_sik_bridge::encode_telem_battery(next_seq_(), telem);
    write_frame_(frame);
    last_battery_tx_ = now_time;
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
  std::string cmd_vel_topic_;
  std::string arm_twist_topic_;
  std::string arm_frame_id_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
  rclcpp::Subscription<mr2_battery_monitor::msg::PackTelemetry>::SharedPtr
      battery_sub_;
  rclcpp::TimerBase::SharedPtr zero_timer_;

  // Serial
  int fd_{-1};
  std::thread reader_thread_;
  std::mutex write_mutex_;
  bool shutting_down_{false};
  uint8_t seq_{0};

  // State
  rclcpp::Time last_heartbeat_{};
  rclcpp::Time last_battery_tx_{};
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
