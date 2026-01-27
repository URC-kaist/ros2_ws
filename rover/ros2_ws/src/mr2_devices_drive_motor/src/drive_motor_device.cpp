#include "mr2_can_bus_core/can_device.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>

namespace mr2_devices_drive_motor {

class DriveMotorDevice : public CanDevice {
public:
  void configure(const hardware_interface::ComponentInfo &info,
                 rclcpp::Node *node) override {
    node_ = node;
    logger_ = node_ ? node_->get_logger()
                    : rclcpp::get_logger("drive_motor_device");

    const auto iface_it = info.parameters.find("can_iface");
    if (iface_it == info.parameters.end()) {
      throw std::runtime_error("Missing can_iface parameter");
    }
    iface_ = iface_it->second;

    const auto node_it = info.parameters.find("node_id");
    if (node_it == info.parameters.end()) {
      throw std::runtime_error("Missing node_id parameter");
    }
    node_id_ = static_cast<uint16_t>(std::stoi(node_it->second));

    int bitrate = 1'000'000;
    const auto bitrate_it = info.parameters.find("can_bitrate");
    if (bitrate_it != info.parameters.end()) {
      bitrate = std::stoi(bitrate_it->second);
    }

    bus_ = CanBusRegistry::get(iface_, bitrate);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus");
    }

    const uint32_t feedback_id = kTelemetryBase + node_id_;
    add_filter(bus_, feedback_id, 0x7FF,
               [this](const can_frame &f) { on_feedback(f); });

    last_integrate_time_ns_ = clock_.now().nanoseconds();
  }

  void process(const rclcpp::Time &) override {
    const int64_t now_ns = clock_.now().nanoseconds();
    integrate_position(now_ns);
    check_feedback_timeout(now_ns);

    double desired = desired_velocity_rad_s_;
    if (!std::isfinite(desired)) {
      desired = 0.0; // Fail safe to zero when controller hasn't set a target.
    }

    transmit_velocity(desired);
  }

  void export_state(double *&position, double *&velocity,
                    double *&effort) override {
    // Keep ROS state finite so TF/odometry remain well-defined even before
    // feedback arrives.
    if (!std::isfinite(position_rad_)) {
      position_rad_ = 0.0;
    }
    if (!std::isfinite(velocity_rad_)) {
      velocity_rad_ = 0.0;
    }

    position = &position_rad_;
    velocity = &velocity_rad_;
    effort_dummy_ = 0.0;
    effort = &effort_dummy_;
  }

  void export_command(double *&command) override {
    command = &desired_velocity_rad_s_;
  }

private:
  // Driveactuator protocol (SimpleFOC STM32F446): cmd = 0x200+node_id, tx = 0x400+node_id
  static constexpr uint16_t kCommandBase = 0x200;
  static constexpr uint16_t kTelemetryBase = 0x400;
  static constexpr int64_t kFeedbackTimeoutNs = 500'000'000; // 0.5 s

  void integrate_position(int64_t now_ns) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (last_integrate_time_ns_ > 0 && std::isfinite(velocity_rad_)) {
      const double dt = (now_ns - last_integrate_time_ns_) * 1e-9;
      if (dt > 0.0 && dt < 1.0) {
        position_rad_ += velocity_rad_ * dt;
      }
    }
    last_integrate_time_ns_ = now_ns;
  }

  void check_feedback_timeout(int64_t now_ns) {
    const int64_t last_fb = last_feedback_ns_.load(std::memory_order_acquire);
    if (last_fb == 0) {
      return;
    }
    if (now_ns - last_fb > kFeedbackTimeoutNs) {
      if (!timed_out_.exchange(true, std::memory_order_acq_rel)) {
        RCLCPP_WARN(logger_, "Drive motor %u feedback timed out", node_id_);
      }
    } else if (timed_out_.exchange(false, std::memory_order_acq_rel)) {
      RCLCPP_INFO(logger_, "Drive motor %u feedback restored", node_id_);
    }
  }

  void transmit_velocity(double vel_rad_s) {
    // Convert rad/s to milli-rad/s and clamp to int32 range.
    long long mrad_s_ll = std::llround(vel_rad_s * 1000.0);
    if (mrad_s_ll > std::numeric_limits<int32_t>::max()) {
      mrad_s_ll = std::numeric_limits<int32_t>::max();
    }
    if (mrad_s_ll < std::numeric_limits<int32_t>::min()) {
      mrad_s_ll = std::numeric_limits<int32_t>::min();
    }
    const int32_t mrad_s = static_cast<int32_t>(mrad_s_ll);

    struct can_frame fr {};
    fr.can_id = kCommandBase + node_id_;
    fr.can_dlc = 4;
    fr.data[0] = static_cast<uint8_t>(mrad_s & 0xFF);
    fr.data[1] = static_cast<uint8_t>((mrad_s >> 8) & 0xFF);
    fr.data[2] = static_cast<uint8_t>((mrad_s >> 16) & 0xFF);
    fr.data[3] = static_cast<uint8_t>((mrad_s >> 24) & 0xFF);

    send(fr, bus_);
  }

  void on_feedback(const can_frame &f) {
    if (f.can_dlc < 4) {
      return;
    }

    const int32_t raw = static_cast<int32_t>(
        (static_cast<uint32_t>(f.data[0])) |
        (static_cast<uint32_t>(f.data[1]) << 8) |
        (static_cast<uint32_t>(f.data[2]) << 16) |
        (static_cast<uint32_t>(f.data[3]) << 24));
    const double vel_rad_s = static_cast<double>(raw) / 1000.0;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      velocity_rad_ = vel_rad_s;
    }

    last_feedback_ns_.store(clock_.now().nanoseconds(),
                            std::memory_order_release);
  }

  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("drive_motor_device")};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  std::string iface_;
  uint16_t node_id_{0};

  std::mutex state_mutex_;
  int64_t last_integrate_time_ns_{0};
  std::atomic<int64_t> last_feedback_ns_{0};
  std::atomic<bool> timed_out_{false};

  double position_rad_{0.0};
  double velocity_rad_{0.0};
  double desired_velocity_rad_s_{std::numeric_limits<double>::quiet_NaN()};
  double effort_dummy_{0.0};
};

} // namespace mr2_devices_drive_motor

PLUGINLIB_EXPORT_CLASS(mr2_devices_drive_motor::DriveMotorDevice, CanDevice)
