#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mr2_can_bus_core/can_device.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>

namespace mr2_devices_ak_servo {

class AkServoDevice : public CanDevice {
public:
  static constexpr int64_t kStatusTimeoutNanosec = 500000000; // 0.5 s
  static constexpr double kCommandReleaseThreshold = 1e-3;

  void configure(const hardware_interface::ComponentInfo &ji,
                 rclcpp::Node *node) override {
    node_ = node;
    joint_name_ = ji.name;
    id_ = std::stoi(ji.parameters.at("motor_id"));
    iface_ = ji.parameters.at("can_iface");

    bus_ = CanBusRegistry::get(iface_, 1'000'000);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus");
    }

    logger_ = node->get_logger();
    ros_clock_ = node->get_clock();
    const std::string temperature_topic =
        "ak_servo/motor_" + std::to_string(id_) + "/temperature";
    temperature_frame_id_ = joint_name_.empty()
                                ? "ak_servo_motor_" + std::to_string(id_)
                                : joint_name_;
    temperature_pub_ = node->create_publisher<sensor_msgs::msg::Temperature>(
        temperature_topic, rclcpp::SystemDefaultsQoS());

    const uint32_t canonical_feedback_id =
        (0x00002900U | (static_cast<uint32_t>(id_) & 0xFFU));
    add_filter(bus_, canonical_feedback_id, 0x1FFFFFFF,
               [this](const can_frame &f) { on_status(f); });

    pos_.push_back(std::numeric_limits<double>::quiet_NaN());
    vel_.push_back(std::numeric_limits<double>::quiet_NaN());
    eff_.push_back(std::numeric_limits<double>::quiet_NaN());
    command_out_.push_back(std::numeric_limits<double>::quiet_NaN());
    desired_cmd_.push_back(std::numeric_limits<double>::quiet_NaN());
    hold_position_.store(std::numeric_limits<double>::quiet_NaN(),
                         std::memory_order_relaxed);

    timed_out_.store(false, std::memory_order_relaxed);
    last_status_nanosec_.store(0, std::memory_order_relaxed);
    configure_time_nanosec_.store(0, std::memory_order_relaxed);
    initial_command_synced_.store(false, std::memory_order_relaxed);
    controller_command_initialized_.store(false, std::memory_order_relaxed);
    last_controller_command_.store(std::numeric_limits<double>::quiet_NaN(),
                                   std::memory_order_relaxed);
    initial_controller_command_.store(std::numeric_limits<double>::quiet_NaN(),
                                      std::memory_order_relaxed);
    release_logged_.store(false, std::memory_order_relaxed);
  }

  void process(const rclcpp::Time &) override {
    if (timed_out_.load(std::memory_order_acquire)) {
      return;
    }

    const int64_t now_ns = clock_.now().nanoseconds();
    int64_t expected = 0;
    if (configure_time_nanosec_.compare_exchange_strong(expected, now_ns)) {
      // captured first call time
    }

    const int64_t last_status =
        last_status_nanosec_.load(std::memory_order_acquire);
    if (last_status <= 0) {
      const int64_t configure_time =
          configure_time_nanosec_.load(std::memory_order_acquire);
      if (configure_time > 0 &&
          now_ns - configure_time > kStatusTimeoutNanosec) {
        mark_timeout("no status frames received");
      }
      return;
    }

    if (last_status > 0 && now_ns - last_status > kStatusTimeoutNanosec) {
      mark_timeout("status stream stalled");
      return;
    }

    const double position_rad = pos_[0];
    if (!std::isfinite(position_rad)) {
      return;
    }

    if (!std::isfinite(command_out_[0])) {
      return;
    }

    double desired = desired_cmd_[0];
    if (std::isnan(desired)) {
      desired = position_rad;
      desired_cmd_[0] = desired;
    }

    if (!controller_command_initialized_.load(std::memory_order_acquire)) {
      controller_command_initialized_.store(true, std::memory_order_release);
      initial_controller_command_.store(desired, std::memory_order_release);
      last_controller_command_.store(desired, std::memory_order_release);
      hold_position_.store(position_rad, std::memory_order_release);
      release_logged_.store(false, std::memory_order_release);
    } else {
      last_controller_command_.store(desired, std::memory_order_release);
    }

    double hold = hold_position_.load(std::memory_order_acquire);
    if (std::isnan(hold)) {
      hold = position_rad;
      hold_position_.store(hold, std::memory_order_release);
    }

    if (!initial_command_synced_.load(std::memory_order_acquire)) {
      hold = position_rad;
      hold_position_.store(hold, std::memory_order_release);

      if (std::fabs(desired - hold) <= kCommandReleaseThreshold) {
        initial_command_synced_.store(true, std::memory_order_release);
        command_out_[0] = desired;
        hold_position_.store(desired, std::memory_order_release);
        if (!release_logged_.exchange(true, std::memory_order_acq_rel) &&
            node_) {
          RCLCPP_INFO(node_->get_logger(),
                      "AK servo %d released to controller command (%.4f rad).",
                      id_, desired);
        }
      } else {
        command_out_[0] = hold;
      }
    } else {
      command_out_[0] = desired;
      hold_position_.store(desired, std::memory_order_release);
    }

    struct can_frame fr {};
    fr.can_id = (0x00000400 | id_) | CAN_EFF_FLAG;
    fr.can_dlc = 4;
    const int32_t p = std::lround(command_out_[0] * 180.0 / M_PI * 1e4);
    fr.data[0] = (p >> 24) & 0xFF;
    fr.data[1] = (p >> 16) & 0xFF;
    fr.data[2] = (p >> 8) & 0xFF;
    fr.data[3] = (p)&0xFF;
    RCLCPP_INFO(logger_,
                "AK servo %d command: %.6f rad (%.3f deg)",
                id_, command_out_[0],
                command_out_[0] * 180.0 / M_PI);
    send(fr, bus_);
  }

  void export_state(std::vector<double *> &pos, std::vector<double *> &vel,
                    std::vector<double *> &eff) override {
    pos.push_back(&pos_[0]);
    vel.push_back(&vel_[0]);
    eff.push_back(&eff_[0]);
  }

  void export_command(std::vector<double *> &cmd) override {
    cmd.push_back(&desired_cmd_[0]);
  }

private:
  static const char *error_code_to_string(uint8_t code) {
    switch (code) {
    case 0:
      return "OK";
    case 1:
      return "Over temperature";
    case 2:
      return "Over current";
    case 3:
      return "Over voltage";
    case 4:
      return "Under voltage";
    case 5:
      return "Encoder fault";
    case 6:
      return "Phase current unbalance";
    default:
      return "Unknown";
    }
  }

  void on_status(const can_frame &f) {
    const int16_t p10 = static_cast<int16_t>(
        (static_cast<uint16_t>(f.data[0]) << 8) |
        static_cast<uint16_t>(f.data[1]));
    const int16_t v10 = static_cast<int16_t>(
        (static_cast<uint16_t>(f.data[2]) << 8) |
        static_cast<uint16_t>(f.data[3]));
    const int16_t c01 = static_cast<int16_t>(
        (static_cast<uint16_t>(f.data[4]) << 8) |
        static_cast<uint16_t>(f.data[5]));
    const int8_t temp_c = static_cast<int8_t>(f.data[6]);
    const uint8_t error_code = f.data[7];

    pos_[0] = (p10 / 10.0) * (M_PI / 180.0);
    vel_[0] = (v10 * 10.0) * (M_PI / 30.0);
    eff_[0] = c01 / 100.0;
    last_temperature_c_.store(temp_c, std::memory_order_release);

    const int64_t now_ns = clock_.now().nanoseconds();
    if (temperature_pub_ && can_publish()) {
      sensor_msgs::msg::Temperature msg;
      msg.header.stamp = rclcpp::Time(now_ns, RCL_STEADY_TIME);
      if (ros_clock_ && rclcpp::ok()) {
        try {
          msg.header.stamp = ros_clock_->now();
        } catch (const rclcpp::exceptions::RCLError &ex) {
          RCLCPP_WARN_ONCE(logger_,
                           "AK servo %d failed to query ROS clock during shutdown: %s",
                           id_, ex.what());
        }
      }
      msg.header.frame_id = temperature_frame_id_;
      msg.temperature = static_cast<double>(temp_c);
      msg.variance = -1.0;
      safe_publish(temperature_pub_, msg);
    }

    const uint8_t prev_error =
        last_error_code_.exchange(error_code, std::memory_order_acq_rel);
    if (error_code != prev_error) {
      if (error_code == 0) {
        if (prev_error != 0) {
          RCLCPP_INFO(logger_,
                      "AK servo %d cleared error code %u (%s).", id_,
                      static_cast<unsigned int>(prev_error),
                      error_code_to_string(prev_error));
        }
      } else {
        RCLCPP_ERROR(logger_,
                     "AK servo %d reported error code %u (%s).", id_,
                     static_cast<unsigned int>(error_code),
                     error_code_to_string(error_code));
      }
    }

    last_status_nanosec_.store(now_ns, std::memory_order_release);

    if (!std::isfinite(command_out_[0])) {
      const double current = pos_[0];
      command_out_[0] = current;
      desired_cmd_[0] = current;
      hold_position_.store(current, std::memory_order_release);
      initial_command_synced_.store(false, std::memory_order_release);
      controller_command_initialized_.store(false, std::memory_order_release);
      last_controller_command_.store(current, std::memory_order_release);
      initial_controller_command_.store(current, std::memory_order_release);
    }

    const bool was_timed_out =
        timed_out_.exchange(false, std::memory_order_acq_rel);
    if (was_timed_out &&
        !recovered_logged_.exchange(true, std::memory_order_acq_rel)) {
      RCLCPP_WARN(logger_,
                  "AK servo %d feedback recovered after timeout.", id_);
    }

    if (!initial_command_synced_.load(std::memory_order_acquire)) {
      const double current = pos_[0];
      hold_position_.store(current, std::memory_order_release);
      if (std::isnan(desired_cmd_[0])) {
        desired_cmd_[0] = current;
      }
    }
  }

  void mark_timeout(const char *reason) {
    bool expected = false;
    if (timed_out_.compare_exchange_strong(expected, true,
                                           std::memory_order_acq_rel)) {
      RCLCPP_ERROR(logger_,
                   "AK servo %d timed out waiting for feedback (%s)", id_,
                   reason);
      recovered_logged_.store(false, std::memory_order_release);
    }
  }

  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("ak_servo_device")};
  rclcpp::Clock::SharedPtr ros_clock_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
  std::string iface_;
  std::string joint_name_;
  std::string temperature_frame_id_;
  int id_{0};
  std::atomic<bool> timed_out_{false};
  std::atomic<int64_t> last_status_nanosec_{0};
  std::atomic<int64_t> configure_time_nanosec_{0};
  std::atomic<bool> initial_command_synced_{false};
  std::atomic<bool> controller_command_initialized_{false};
  std::atomic<double> last_controller_command_{0.0};
  std::atomic<double> initial_controller_command_{0.0};
  std::atomic<bool> recovered_logged_{true};
  std::atomic<int8_t> last_temperature_c_{0};
  std::atomic<uint8_t> last_error_code_{0};
  std::atomic<bool> release_logged_{false};

  std::vector<double> pos_, vel_, eff_;
  std::vector<double> command_out_;
  std::vector<double> desired_cmd_;
  std::atomic<double> hold_position_{std::numeric_limits<double>::quiet_NaN()};

  template<typename MsgT>
  void safe_publish(const typename rclcpp::Publisher<MsgT>::SharedPtr &pub,
                    const MsgT &msg) {
    if (!pub || !can_publish()) {
      return;
    }
    try {
      pub->publish(msg);
    } catch (const rclcpp::exceptions::RCLError &ex) {
      RCLCPP_WARN_ONCE(logger_,
                       "AK servo publisher inactive during shutdown: %s",
                       ex.what());
    }
  }

  bool can_publish() const {
    if (!node_) {
      return false;
    }
    auto base = node_->get_node_base_interface();
    if (!base) {
      return false;
    }
    auto context = base->get_context();
    return context && context->is_valid() && rclcpp::ok(context);
  }
};

} // namespace mr2_devices_ak_servo

PLUGINLIB_EXPORT_CLASS(mr2_devices_ak_servo::AkServoDevice, CanDevice)
