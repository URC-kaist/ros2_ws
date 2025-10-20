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
#include <mutex>

namespace mr2_devices_ak_servo {

class AkServoDevice : public CanDevice {
public:
  static constexpr int64_t kStatusTimeoutNanosec = 500000000; // 0.5 s
  static constexpr double kCommandReleaseThreshold = 1e-3;

  // Called once by the hardware interface to wire up CAN plumbing and ROS I/O.
  void configure(const hardware_interface::ComponentInfo &ji,
                 rclcpp::Node *node) override {
    node_ = node;

    // Parse motor configuration
    id_ = std::stoi(ji.parameters.at("motor_id"));
    iface_ = ji.parameters.at("can_iface");
    joint_name_ = ji.name;

    std::string temperature_topic =
        "ak_servo/motor_" + std::to_string(id_) + "/temperature";
    std::string temperature_frame_id =
        joint_name_.empty() ? "ak_servo_motor_" + std::to_string(id_)
                            : joint_name_;

    /// Retrieve can bus from the registry
    bus_ = CanBusRegistry::get(iface_, 1'000'000);
    if (!bus_) {
      throw std::runtime_error("Cannot open CAN bus");
    }

    // Set up temperature topic
    logger_ = node->get_logger();
    ros_clock_ = node->get_clock();
    temperature_frame_id_ = temperature_frame_id;
    temperature_pub_ = node_->create_publisher<sensor_msgs::msg::Temperature>(
        temperature_topic, rclcpp::SystemDefaultsQoS());

    // Add CAN bus feedback filter
    const uint32_t canonical_feedback_id =
        (0x00002900U | (static_cast<uint32_t>(id_) & 0xFFU));
    add_filter(bus_, canonical_feedback_id, 0x1FFFFFFF,
               [this](const can_frame &f) { on_status(f); });
  }

  // Runs every control cycle; validates feedback health and pushes the next
  // position command out on the bus once the controller has taken over.
  void process(const rclcpp::Time &) override {
    if (timed_out_.load(std::memory_order_acquire)) {
      return;
    }

    const int64_t now_ns = clock_.now().nanoseconds();

    // Record first configure time
    if (!configure_time_recorded_) {
      configure_time_nanosec_ = now_ns;
      configure_time_recorded_ = true;
    }

    if (!status_stream_active(now_ns)) {
      return;
    }

    const double position_rad = position_rad_;
    if (!std::isfinite(position_rad)) {
      return;
    }

    // Retrieve the latest desired command, else defaulting to "hold"
    double desired = desired_command_rad_;
    if (std::isnan(desired)) {
      if (hold_position_rad_ == std::numeric_limits<double>::quiet_NaN()) {
        hold_position_rad_ = position_rad_;
      }
      desired = hold_position_rad_;
    } else {
      hold_position_rad_ = std::numeric_limits<double>::quiet_NaN();
    }

    // Initialize controller state
    controller_initialized_.store(true, std::memory_order_relaxed);

    command_out_rad_ = compute_target_command(position_rad, desired);
    transmit_command();
  }

  void export_state(double *&position, double *&velocity,
                    double *&effort) override {
    position = &position_rad_;
    velocity = &velocity_rad_;
    effort = &effort_amp_;
  }

  void export_command(double *&command) override {
    command = &desired_command_rad_;
  }

private:
  struct StatusSample {
    double position_rad;
    double velocity_rad;
    double current_amp;
    int8_t temperature_c;
    uint8_t error_code;
    int64_t timestamp_ns;
  };

  // Ensure the CAN feedback stream is alive and mark a timeout if frames stop.
  bool status_stream_active(int64_t now_ns) {
    const int64_t last_status =
        last_status_nanosec_.load(std::memory_order_acquire);
    if (last_status <= 0) {
      if (configure_time_recorded_ &&
          now_ns - configure_time_nanosec_ > kStatusTimeoutNanosec) {
        mark_timeout("no status frames received");
      }
      return false;
    }

    if (now_ns - last_status > kStatusTimeoutNanosec) {
      mark_timeout("status stream stalled");
      return false;
    }
    return true;
  }

  // Decide which angle should be sent to the driver (initially the first
  // sampled position, then whatever the controller requests).
  double compute_target_command(double position_rad, double desired) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    (void)position_rad;
    return desired;
  }

  void transmit_command() {
    struct can_frame fr {};
    fr.can_id = (0x00000400 | id_) | CAN_EFF_FLAG;
    fr.can_dlc = 4;
    const int32_t p = std::lround(command_out_rad_ * 180.0 / M_PI * 1e4);
    fr.data[0] = (p >> 24) & 0xFF;
    fr.data[1] = (p >> 16) & 0xFF;
    fr.data[2] = (p >> 8) & 0xFF;
    fr.data[3] = (p)&0xFF;
    send(fr, bus_);
  }

  StatusSample decode_status_frame(const can_frame &f, int64_t now_ns) const {
    StatusSample sample{};
    const int16_t p10 =
        static_cast<int16_t>((static_cast<uint16_t>(f.data[0]) << 8) |
                             static_cast<uint16_t>(f.data[1]));
    const int16_t v10 =
        static_cast<int16_t>((static_cast<uint16_t>(f.data[2]) << 8) |
                             static_cast<uint16_t>(f.data[3]));
    const int16_t c01 =
        static_cast<int16_t>((static_cast<uint16_t>(f.data[4]) << 8) |
                             static_cast<uint16_t>(f.data[5]));

    sample.position_rad = (p10 / 10.0) * (M_PI / 180.0);
    sample.velocity_rad = (v10 * 10.0) * (M_PI / 30.0);
    sample.current_amp = c01 / 100.0;
    sample.temperature_c = static_cast<int8_t>(f.data[6]);
    sample.error_code = f.data[7];
    sample.timestamp_ns = now_ns;
    return sample;
  }

  void publish_temperature(const StatusSample &sample) {
    if (!temperature_pub_ || !can_publish()) {
      return;
    }

    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = rclcpp::Time(sample.timestamp_ns, RCL_STEADY_TIME);
    if (ros_clock_ && rclcpp::ok()) {
      try {
        msg.header.stamp = ros_clock_->now();
      } catch (const rclcpp::exceptions::RCLError &ex) {
        RCLCPP_WARN_ONCE(
            logger_,
            "AK servo %d failed to query ROS clock during shutdown: %s", id_,
            ex.what());
      }
    }
    msg.header.frame_id = temperature_frame_id_;
    msg.temperature = static_cast<double>(sample.temperature_c);
    msg.variance = -1.0;
    safe_publish(temperature_pub_, msg);
  }

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

  void handle_error_code(uint8_t error_code, uint8_t previous_error) {
    if (error_code == previous_error) {
      return;
    }

    if (error_code == 0) {
      if (previous_error != 0) {
        RCLCPP_INFO(logger_, "AK servo %d cleared error code %u (%s).", id_,
                    static_cast<unsigned int>(previous_error),
                    error_code_to_string(previous_error));
      }
    } else {
      RCLCPP_ERROR(logger_, "AK servo %d reported error code %u (%s).", id_,
                   static_cast<unsigned int>(error_code),
                   error_code_to_string(error_code));
    }
  }

  void note_feedback_recovery() {
    const bool was_timed_out =
        timed_out_.exchange(false, std::memory_order_acq_rel);
    bool should_log = false;
    if (was_timed_out) {
      if (!recovered_logged_.exchange(true, std::memory_order_acq_rel)) {
        should_log = true;
      }
    }
    if (should_log) {
      RCLCPP_WARN(logger_, "AK servo %d feedback recovered after timeout.",
                  id_);
    }
  }

  // Callback registered with the CAN manager; updates telemetry and command
  // state whenever the servo publishes a feedback frame.
  void on_status(const can_frame &f) {
    const int64_t now_ns = clock_.now().nanoseconds();
    const StatusSample sample = decode_status_frame(f, now_ns);

    position_rad_ = sample.position_rad;
    velocity_rad_ = sample.velocity_rad;
    effort_amp_ = sample.current_amp;
    last_temperature_c_ = sample.temperature_c;

    publish_temperature(sample);

    const uint8_t previous_error = last_error_code_;
    last_error_code_ = sample.error_code;
    handle_error_code(sample.error_code, previous_error);

    last_status_nanosec_.store(sample.timestamp_ns, std::memory_order_release);

    note_feedback_recovery();
  }

  void mark_timeout(const char *reason) {
    bool expected = false;
    if (timed_out_.compare_exchange_strong(expected, true,
                                           std::memory_order_acq_rel)) {
      recovered_logged_.store(false, std::memory_order_relaxed);
      RCLCPP_ERROR(logger_, "AK servo %d timed out waiting for feedback (%s)",
                   id_, reason);
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
  std::atomic<bool> controller_initialized_{false};
  std::atomic<bool> recovered_logged_{true};
  std::atomic<int64_t> last_status_nanosec_{0};
  bool configure_time_recorded_{false};
  int64_t configure_time_nanosec_ = 0;
  int8_t last_temperature_c_{0};
  uint8_t last_error_code_{0};

  double position_rad_{std::numeric_limits<double>::quiet_NaN()};
  double velocity_rad_{std::numeric_limits<double>::quiet_NaN()};
  double effort_amp_{std::numeric_limits<double>::quiet_NaN()};
  double command_out_rad_{std::numeric_limits<double>::quiet_NaN()};
  double desired_command_rad_{std::numeric_limits<double>::quiet_NaN()};
  double hold_position_rad_{std::numeric_limits<double>::quiet_NaN()};

  mutable std::mutex command_mutex_;

  template <typename MsgT>
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
