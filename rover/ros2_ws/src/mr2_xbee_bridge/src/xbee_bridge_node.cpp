#include <algorithm>
#include <array>
#include <atomic>
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

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mr2_action_interface/msg/mission_control.hpp"
#include "mr2_battery_monitor/msg/pack_telemetry.hpp"
#include "mr2_xbee_bridge/packets.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ublox_ubx_msgs/msg/ubx_nav_svin.hpp"
#include "rtcm_msgs/msg/message.hpp"

namespace mr2_xbee_bridge {

using mr2_xbee_bridge::CmdArmTwist;
using mr2_xbee_bridge::CmdDrive;
using mr2_xbee_bridge::Frame;
using mr2_xbee_bridge::Heartbeat;
using mr2_xbee_bridge::MissionControl;
using mr2_xbee_bridge::TelemBattery;
using mr2_xbee_bridge::TelemNav;
using mr2_xbee_bridge::CmdArmGripper;
using mr2_xbee_bridge::CmdArmJoint;
using mr2_xbee_bridge::CmdCameraTurret;

class XbeeBridgeNode : public rclcpp::Node {
 public:
  XbeeBridgeNode()
      : rclcpp::Node("xbee_bridge"),
        device_(declare_parameter<std::string>("device", "/dev/ttyXBEE")),
        heartbeat_timeout_ms_(
            declare_parameter<int>("heartbeat_timeout_ms", 500)),
        zero_publish_rate_hz_(
            declare_parameter<double>("zero_publish_rate_hz", 20.0)),
        battery_tx_rate_hz_(
            declare_parameter<double>("battery_tx_rate_hz", 1.0)),
        heartbeat_tx_rate_hz_(
            declare_parameter<double>("heartbeat_tx_rate_hz", 2.0)),
        nav_tx_rate_hz_(declare_parameter<double>("nav_tx_rate_hz", 2.0)),
        smoothing_initial_dt_s_(
            declare_parameter<double>("smoothing_initial_dt_s", 0.05)),
        smooth_drive_commands_(
            declare_parameter<bool>("smooth_drive_commands", true)),
        drive_linear_accel_limit_m_s2_(declare_parameter<double>(
            "drive_linear_accel_limit_m_s2", 0.5)),
        drive_angular_accel_limit_rad_s2_(declare_parameter<double>(
            "drive_angular_accel_limit_rad_s2", 1.0)),
        smooth_arm_twist_commands_(
            declare_parameter<bool>("smooth_arm_twist_commands", true)),
        arm_twist_linear_accel_limit_m_s2_(declare_parameter<double>(
            "arm_twist_linear_accel_limit_m_s2", 0.3)),
        arm_twist_angular_accel_limit_rad_s2_(declare_parameter<double>(
            "arm_twist_angular_accel_limit_rad_s2", 1.0)),
        smooth_arm_joint_commands_(
            declare_parameter<bool>("smooth_arm_joint_commands", true)),
        arm_joint_accel_limit_rad_s2_(declare_parameter<double>(
            "arm_joint_accel_limit_rad_s2", 0.8)),
        cmd_vel_topic_(
            declare_parameter<std::string>("cmd_vel_topic", "/base/cmd_vel")),
        mission_control_topic_(declare_parameter<std::string>(
            "mission_control_topic", "/mission_control")),
        arm_twist_topic_(declare_parameter<std::string>(
            "arm_twist_topic", "/moveit_servo/delta_twist_cmds")),
        arm_joint_topic_(declare_parameter<std::string>(
            "arm_joint_topic", "/moveit_servo/delta_joint_cmds")),
        arm_joint_names_(declare_parameter<std::vector<std::string>>(
            "arm_joint_names",
            {"arm_j1", "arm_j2", "arm_j3", "arm_j4", "arm_j5", "arm_j6"})),
        arm_joint_duration_s_(
            declare_parameter<double>("arm_joint_duration_s", 0.1)),
        gripper_cmd_topic_(declare_parameter<std::string>(
            "gripper_cmd_topic", "/gripper_controller/commands")),
        camera_turret_cmd_topic_(declare_parameter<std::string>(
            "camera_turret_cmd_topic", "/camera_turret/command")),
        gripper_min_position_rad_(
            declare_parameter<double>("gripper_min_position_rad", 0.0)),
        gripper_max_position_rad_(
            declare_parameter<double>("gripper_max_position_rad", 1.0)),
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
    mission_control_pub_ =
        create_publisher<mr2_action_interface::msg::MissionControl>(
            mission_control_topic_, 10);
    arm_twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(arm_twist_topic_, 10);
    arm_joint_pub_ =
        create_publisher<control_msgs::msg::JointJog>(arm_joint_topic_, 10);
    gripper_cmd_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(gripper_cmd_topic_, 10);
    camera_turret_cmd_pub_ =
        create_publisher<geometry_msgs::msg::Vector3>(camera_turret_cmd_topic_, 10);
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
        std::bind(&XbeeBridgeNode::zero_check, this));

    if (heartbeat_tx_rate_hz_ > 0.0) {
      const auto heartbeat_period = std::chrono::duration<double>(
          1.0 / std::max(heartbeat_tx_rate_hz_, 0.1));
      heartbeat_tx_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
          std::bind(&XbeeBridgeNode::send_heartbeat_, this));
    }

    if (nav_tx_rate_hz_ > 0.0) {
      const auto nav_period =
          std::chrono::duration<double>(1.0 / std::max(nav_tx_rate_hz_, 0.1));
      nav_tx_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(nav_period),
          std::bind(&XbeeBridgeNode::send_nav_, this));
    }

    last_heartbeat_ = now();
    last_battery_tx_[0] = now() - rclcpp::Duration::from_seconds(10.0);
    last_battery_tx_[1] = now() - rclcpp::Duration::from_seconds(10.0);
  }

  ~XbeeBridgeNode() override {
    request_shutdown_();
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

 private:
  void request_shutdown_() {
    bool expected = false;
    if (!shutting_down_.compare_exchange_strong(expected, true)) {
      return;
    }

    auto cancel_timer = [](const rclcpp::TimerBase::SharedPtr & timer) {
        if (timer) {
          timer->cancel();
        }
      };

    cancel_timer(zero_timer_);
    cancel_timer(heartbeat_tx_timer_);
    cancel_timer(nav_tx_timer_);

    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      fd_ = -1;
    }
  }

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

    if (cfsetispeed(&tio, B115200) != 0 || cfsetospeed(&tio, B115200) != 0) {
      throw std::runtime_error("Failed to set serial baud rate");
    }

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      throw std::runtime_error("Failed to apply serial attributes");
    }
  }

  void start_reader_() {
    reader_thread_ = std::thread([this]() { read_loop_(); });
  }

  void read_loop_() {
    std::vector<uint8_t> buffer;
    buffer.reserve(512);

    while (rclcpp::ok() && !shutting_down_.load()) {
      int local_fd = -1;
      {
        std::lock_guard<std::mutex> lock(write_mutex_);
        local_fd = fd_;
      }
      if (local_fd < 0) {
        return;
      }

      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(local_fd, &readfds);
      timeval timeout{};
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;

      const int ready = select(local_fd + 1, &readfds, nullptr, nullptr, &timeout);
      if (ready <= 0) {
        continue;
      }

      uint8_t temp[256];
      const ssize_t count = ::read(local_fd, temp, sizeof(temp));
      if (count <= 0) {
        continue;
      }

      buffer.insert(buffer.end(), temp, temp + count);
      parse_frames_(buffer);
    }
  }

  void parse_frames_(std::vector<uint8_t> &buffer) {
    size_t offset = 0;
    while (true) {
      if (shutting_down_.load()) {
        break;
      }

      if (buffer.size() - offset < 4) {
        break;
      }

      if (buffer[offset] != mr2_xbee_bridge::kMagic) {
        ++offset;
        continue;
      }

      const uint8_t length = buffer[offset + 2];
      const size_t frame_size = 4 + static_cast<size_t>(length) + 2;
      if (buffer.size() - offset < frame_size) {
        break;
      }

      auto frame = mr2_xbee_bridge::decode_frame(buffer.data() + offset, frame_size);
      if (!frame) {
        if (log_frames_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "Invalid XBEE frame dropped");
        }
        ++offset;
        continue;
      }

      handle_frame_(*frame);
      offset += frame_size;
    }

    if (offset == 0) {
      return;
    }
    if (offset >= buffer.size()) {
      buffer.clear();
      return;
    }
    buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(offset));
  }

  void handle_frame_(const Frame &frame) {
    switch (frame.header.msg_id) {
      case mr2_xbee_bridge::MsgId::kCmdDrive: {
        auto cmd = mr2_xbee_bridge::decode_cmd_drive(frame);
        if (cmd) {
          handle_cmd_drive_(*cmd);
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kCmdArmTwist: {
        auto cmd = mr2_xbee_bridge::decode_cmd_arm_twist(frame);
        if (cmd) {
          handle_cmd_arm_(*cmd);
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kHeartbeat: {
        auto hb = mr2_xbee_bridge::decode_heartbeat(frame);
        if (hb) {
          last_heartbeat_ = now();
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kMissionControl: {
        auto ctrl = mr2_xbee_bridge::decode_mission_control(frame);
        if (ctrl) {
          handle_mission_control_(*ctrl);
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kCmdArmGripper: {
        auto cmd = mr2_xbee_bridge::decode_cmd_arm_gripper(frame);
        if (cmd) {
          handle_cmd_arm_gripper_(*cmd);
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kCmdArmJoint: {
        auto cmd = mr2_xbee_bridge::decode_cmd_arm_joint(frame);
        if (cmd) {
          handle_cmd_arm_joint_(*cmd);
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kCmdCameraTurret: {
        auto cmd = mr2_xbee_bridge::decode_cmd_camera_turret(frame);
        if (cmd) {
          handle_cmd_camera_turret_(*cmd);
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kBaseSvin: {
        auto svin = mr2_xbee_bridge::decode_base_svin(frame);
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
      case mr2_xbee_bridge::MsgId::kBaseRtcm: {
        auto rtcm = mr2_xbee_bridge::decode_base_rtcm(frame);
        if (rtcm && base_rtcm_pub_) {
          rtcm_msgs::msg::Message msg;
          msg.header.stamp = now();
          msg.header.frame_id = "base_rtcm";
          msg.message = rtcm->message;
          base_rtcm_pub_->publish(msg);
        }
        break;
      }
      case mr2_xbee_bridge::MsgId::kBaseRtcmFrag: {
        auto frag = mr2_xbee_bridge::decode_base_rtcm_frag(frame);
        if (frag && base_rtcm_pub_) {
          handle_base_rtcm_frag_(frame.header.seq, *frag);
        }
        break;
      }
      default:
        break;
    }
  }

  void handle_mission_control_(const MissionControl &ctrl) {
    mr2_action_interface::msg::MissionControl msg;
    msg.command = ctrl.command;
    msg.clear_costmap = ctrl.clear_costmap;
    msg.mission_id = ctrl.mission_id;
    mission_control_pub_->publish(msg);
    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "MISSION_CONTROL cmd=%u clear=%s mission_id=%u",
                           ctrl.command, ctrl.clear_costmap ? "true" : "false",
                           ctrl.mission_id);
    }
  }

  void handle_cmd_drive_(const CmdDrive &cmd) {
    std::lock_guard<std::mutex> lock(command_smoothing_mutex_);
    const auto now_time = now();
    std::array<double, 3> target{
        static_cast<double>(cmd.linear_x_m_s),
        static_cast<double>(cmd.linear_y_m_s),
        static_cast<double>(cmd.angular_z_rad_s)};
    std::array<double, 3> output = target;
    if (smooth_drive_commands_) {
      const double dt = smoothing_dt_(last_drive_command_time_, now_time);
      output[0] = slew_(last_drive_command_[0], target[0],
                        drive_linear_accel_limit_m_s2_, dt);
      output[1] = slew_(last_drive_command_[1], target[1],
                        drive_linear_accel_limit_m_s2_, dt);
      output[2] = slew_(last_drive_command_[2], target[2],
                        drive_angular_accel_limit_rad_s2_, dt);
    }
    last_drive_command_ = output;
    last_drive_command_time_ = now_time;

    geometry_msgs::msg::Twist msg;
    msg.linear.x = output[0];
    msg.linear.y = output[1];
    msg.angular.z = output[2];
    cmd_vel_pub_->publish(msg);
    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "CMD_DRIVE target=(%.3f, %.3f, %.3f) out=(%.3f, %.3f, %.3f)",
                           cmd.linear_x_m_s, cmd.linear_y_m_s,
                           cmd.angular_z_rad_s, output[0], output[1],
                           output[2]);
    }
  }

  void handle_cmd_arm_(const CmdArmTwist &cmd) {
    std::lock_guard<std::mutex> lock(command_smoothing_mutex_);
    const auto now_time = now();
    std::array<double, 6> target{
        static_cast<double>(cmd.lin_x_m_s), static_cast<double>(cmd.lin_y_m_s),
        static_cast<double>(cmd.lin_z_m_s), static_cast<double>(cmd.ang_x_rad_s),
        static_cast<double>(cmd.ang_y_rad_s), static_cast<double>(cmd.ang_z_rad_s)};
    std::array<double, 6> output = target;
    if (smooth_arm_twist_commands_) {
      const double dt = smoothing_dt_(last_arm_twist_command_time_, now_time);
      for (size_t i = 0; i < 3; ++i) {
        output[i] = slew_(last_arm_twist_command_[i], target[i],
                          arm_twist_linear_accel_limit_m_s2_, dt);
      }
      for (size_t i = 3; i < output.size(); ++i) {
        output[i] = slew_(last_arm_twist_command_[i], target[i],
                          arm_twist_angular_accel_limit_rad_s2_, dt);
      }
    }
    last_arm_twist_command_ = output;
    last_arm_twist_command_time_ = now_time;

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now_time;
    msg.header.frame_id = arm_frame_id_;
    msg.twist.linear.x = output[0];
    msg.twist.linear.y = output[1];
    msg.twist.linear.z = output[2];
    msg.twist.angular.x = output[3];
    msg.twist.angular.y = output[4];
    msg.twist.angular.z = output[5];
    arm_twist_pub_->publish(msg);
    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "CMD_ARM_TWIST target lin=(%.3f, %.3f, %.3f) ang=(%.3f, %.3f, %.3f) out lin=(%.3f, %.3f, %.3f) ang=(%.3f, %.3f, %.3f)",
                           cmd.lin_x_m_s, cmd.lin_y_m_s, cmd.lin_z_m_s,
                           cmd.ang_x_rad_s, cmd.ang_y_rad_s, cmd.ang_z_rad_s,
                           output[0], output[1], output[2], output[3],
                           output[4], output[5]);
    }
  }

  void handle_cmd_arm_gripper_(const CmdArmGripper &cmd) {
    if (!gripper_cmd_pub_) {
      return;
    }
    const double clipped_norm =
        std::max(0.0, std::min(1.0, static_cast<double>(cmd.position_norm)));
    const double min_pos = std::min(gripper_min_position_rad_, gripper_max_position_rad_);
    const double max_pos = std::max(gripper_min_position_rad_, gripper_max_position_rad_);
    const double target = min_pos + clipped_norm * (max_pos - min_pos);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.push_back(target);
    gripper_cmd_pub_->publish(msg);
    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "CMD_ARM_GRIPPER norm=%.3f target=%.3f",
                           clipped_norm, target);
    }
  }

  void handle_cmd_arm_joint_(const CmdArmJoint &cmd) {
    if (!arm_joint_pub_) {
      return;
    }

    std::lock_guard<std::mutex> lock(command_smoothing_mutex_);
    const auto now_time = now();
    std::array<double, 6> target{};
    std::array<double, 6> output{};
    for (size_t i = 0; i < cmd.velocities_rad_s.size(); ++i) {
      target[i] = cmd.velocities_rad_s[i];
      output[i] = target[i];
    }
    if (smooth_arm_joint_commands_) {
      const double dt = smoothing_dt_(last_arm_joint_command_time_, now_time);
      for (size_t i = 0; i < output.size(); ++i) {
        output[i] = slew_(last_arm_joint_command_[i], target[i],
                          arm_joint_accel_limit_rad_s2_, dt);
      }
    }
    last_arm_joint_command_ = output;
    last_arm_joint_command_time_ = now_time;

    control_msgs::msg::JointJog msg;
    msg.header.stamp = now_time;
    msg.header.frame_id = arm_frame_id_;
    const size_t count =
        std::min(arm_joint_names_.size(), cmd.velocities_rad_s.size());
    msg.joint_names.reserve(count);
    msg.velocities.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      msg.joint_names.push_back(arm_joint_names_[i]);
      msg.velocities.push_back(output[i]);
    }
    msg.duration = arm_joint_duration_s_;
    arm_joint_pub_->publish(msg);

    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "CMD_ARM_JOINT target=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f) out=(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)",
          cmd.velocities_rad_s[0], cmd.velocities_rad_s[1],
          cmd.velocities_rad_s[2], cmd.velocities_rad_s[3],
          cmd.velocities_rad_s[4], cmd.velocities_rad_s[5], output[0],
          output[1], output[2], output[3], output[4], output[5]);
    }
  }

  void handle_cmd_camera_turret_(const CmdCameraTurret &cmd) {
    if (!camera_turret_cmd_pub_) {
      return;
    }

    geometry_msgs::msg::Vector3 msg;
    msg.x = std::clamp(static_cast<double>(cmd.x), -1.0, 1.0);
    msg.y = std::clamp(static_cast<double>(cmd.y), -1.0, 1.0);
    msg.z = std::clamp(static_cast<double>(cmd.z), -1.0, 1.0);
    camera_turret_cmd_pub_->publish(msg);

    if (log_frames_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "CMD_CAMERA_TURRET x=%.3f y=%.3f z=%.3f",
                           msg.x, msg.y, msg.z);
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

    CmdDrive zero_drive;
    handle_cmd_drive_(zero_drive);

    CmdArmTwist zero_arm;
    handle_cmd_arm_(zero_arm);

    CmdArmJoint zero_joint;
    handle_cmd_arm_joint_(zero_joint);

    CmdCameraTurret zero_turret;
    handle_cmd_camera_turret_(zero_turret);
  }

  void send_heartbeat_() {
    Heartbeat hb;
    hb.timestamp_ms =
        static_cast<uint32_t>(now().nanoseconds() / 1000000);
    auto frame = mr2_xbee_bridge::encode_heartbeat(next_seq_(), hb);
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
        mr2_xbee_bridge::encode_telem_battery(next_seq_(), telem, battery_id);
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

    auto frame = mr2_xbee_bridge::encode_telem_nav(next_seq_(), nav);
    write_frame_(frame);
  }

  double smoothing_dt_(const rclcpp::Time &last_time,
                       const rclcpp::Time &now_time) const {
    if (last_time.nanoseconds() <= 0) {
      return std::max(0.0, smoothing_initial_dt_s_);
    }
    const double dt = (now_time - last_time).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      return std::max(0.0, smoothing_initial_dt_s_);
    }
    return std::min(dt, 0.25);
  }

  static double slew_(double current, double target, double rate_limit,
                      double dt) {
    if (!std::isfinite(target)) {
      target = 0.0;
    }
    if (!std::isfinite(current)) {
      current = 0.0;
    }
    if (!std::isfinite(rate_limit) || rate_limit <= 0.0 || dt <= 0.0) {
      return target;
    }
    const double max_step = rate_limit * dt;
    return current + std::clamp(target - current, -max_step, max_step);
  }

  void write_frame_(const std::vector<uint8_t> &frame) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (fd_ < 0 || shutting_down_.load()) {
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
  int heartbeat_timeout_ms_;
  double zero_publish_rate_hz_;
  double battery_tx_rate_hz_;
  double heartbeat_tx_rate_hz_;
  double nav_tx_rate_hz_;
  double smoothing_initial_dt_s_;
  bool smooth_drive_commands_;
  double drive_linear_accel_limit_m_s2_;
  double drive_angular_accel_limit_rad_s2_;
  bool smooth_arm_twist_commands_;
  double arm_twist_linear_accel_limit_m_s2_;
  double arm_twist_angular_accel_limit_rad_s2_;
  bool smooth_arm_joint_commands_;
  double arm_joint_accel_limit_rad_s2_;
  std::string cmd_vel_topic_;
  std::string mission_control_topic_;
  std::string arm_twist_topic_;
  std::string arm_joint_topic_;
  std::vector<std::string> arm_joint_names_;
  double arm_joint_duration_s_;
  std::string gripper_cmd_topic_;
  std::string camera_turret_cmd_topic_;
  double gripper_min_position_rad_;
  double gripper_max_position_rad_;
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
  rclcpp::Publisher<mr2_action_interface::msg::MissionControl>::SharedPtr
      mission_control_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr arm_joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr camera_turret_cmd_pub_;
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
  std::atomic<bool> shutting_down_{false};
  uint8_t seq_{0};

  // State
  rclcpp::Time last_heartbeat_{};
  std::array<double, 3> last_drive_command_{};
  std::mutex command_smoothing_mutex_;
  rclcpp::Time last_drive_command_time_{0, 0, RCL_ROS_TIME};
  std::array<double, 6> last_arm_twist_command_{};
  rclcpp::Time last_arm_twist_command_time_{0, 0, RCL_ROS_TIME};
  std::array<double, 6> last_arm_joint_command_{};
  rclcpp::Time last_arm_joint_command_time_{0, 0, RCL_ROS_TIME};
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

}  // namespace mr2_xbee_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mr2_xbee_bridge::XbeeBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("xbee_bridge"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
