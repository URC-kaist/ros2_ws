#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <linux/can.h>

#include "rclcpp/rclcpp.hpp"

#include "mr2_can_bus_core/can_bus_manager.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"

using namespace std::chrono_literals;

class MockAkServoNode : public rclcpp::Node {
public:
  MockAkServoNode() : rclcpp::Node("mock_ak_servo") {
    can_iface_ = declare_parameter<std::string>("can_iface", "vcan0");
    servo_id_ = declare_parameter<int>("motor_id", 4);
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 200.0);
    max_velocity_rad_s_ = declare_parameter<double>("max_velocity", 4.0);
    effort_gain_ = declare_parameter<double>("effort_gain", 4.0);
    max_effort_amp_ = declare_parameter<double>("max_effort_amp", 6.0);

    if (update_rate_hz_ <= 0.0) {
      throw std::runtime_error("update_rate_hz must be positive");
    }

    bus_ = CanBusRegistry::get(can_iface_);
    if (!bus_) {
      throw std::runtime_error("Failed to acquire CAN bus on interface '" +
                               can_iface_ + "'");
    }

    const uint32_t command_filter_id =
        0x00000400u | static_cast<uint32_t>(servo_id_);
    bus_->register_listener(
        command_filter_id, 0x1FFFFFFFu,
        [this](const struct can_frame &frame) { this->handle_command(frame); });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_));
    timer_ =
        create_wall_timer(period, std::bind(&MockAkServoNode::update, this));

    last_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Mock AK servo started on %s id %d",
                can_iface_.c_str(), servo_id_);
  }

private:
  void handle_command(const struct can_frame &frame) {
    if ((frame.can_id & CAN_EFF_FLAG) == 0) {
      return;
    }

    if (frame.can_dlc < 4) {
      return;
    }

    const int32_t raw = (static_cast<int32_t>(frame.data[0]) << 24) |
                        (static_cast<int32_t>(frame.data[1]) << 16) |
                        (static_cast<int32_t>(frame.data[2]) << 8) |
                        (static_cast<int32_t>(frame.data[3]));

    const double target_deg = static_cast<double>(raw) / 1e4;
    const double target_rad = target_deg * M_PI / 180.0;

    std::lock_guard<std::mutex> lock(state_mtx_);
    target_position_rad_ = target_rad;
  }

  void update() {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    if (dt <= 0.0) {
      dt = 1.0 / update_rate_hz_;
    }
    last_time_ = now;

    double target;
    double position;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);

      target = target_position_rad_;
      position = position_rad_;

      const double error = target - position;
      const double max_step = max_velocity_rad_s_ * dt;
      const double step = std::clamp(error, -max_step, max_step);
      position_rad_ += step;

      velocity_rad_s_ = step / dt;
      effort_amp_ =
          std::clamp(error * effort_gain_, -max_effort_amp_, max_effort_amp_);

      position = position_rad_;
    }

    publish_status(position, velocity_rad_s_, effort_amp_);
  }

  void publish_status(double position, double velocity, double effort) {
    struct can_frame status {};
    status.can_id =
        (0x00002900u | static_cast<uint32_t>(servo_id_)) | CAN_EFF_FLAG;
    status.can_dlc = 8;

    const int16_t pos_10deg =
        static_cast<int16_t>(std::lround(position * 180.0 / M_PI * 10.0));
    const double rpm = velocity * 30.0 / M_PI;
    const int16_t vel_10rpm = static_cast<int16_t>(std::lround(rpm * 10.0));
    const int16_t cur_01amp = static_cast<int16_t>(std::lround(effort * 100.0));

    status.data[0] = static_cast<uint8_t>((pos_10deg >> 8) & 0xFF);
    status.data[1] = static_cast<uint8_t>(pos_10deg & 0xFF);
    status.data[2] = static_cast<uint8_t>((vel_10rpm >> 8) & 0xFF);
    status.data[3] = static_cast<uint8_t>(vel_10rpm & 0xFF);
    status.data[4] = static_cast<uint8_t>((cur_01amp >> 8) & 0xFF);
    status.data[5] = static_cast<uint8_t>(cur_01amp & 0xFF);
    status.data[6] = 0;
    status.data[7] = 0;

    bus_->enqueue_tx(status);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<CanBusManager> bus_;

  std::string can_iface_;
  int servo_id_{4};
  double update_rate_hz_{200.0};
  double max_velocity_rad_s_{4.0};
  double effort_gain_{4.0};
  double max_effort_amp_{6.0};

  std::mutex state_mtx_;
  double target_position_rad_{0.0};
  double position_rad_{0.0};
  double velocity_rad_s_{0.0};
  double effort_amp_{0.0};

  std::chrono::steady_clock::time_point last_time_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MockAkServoNode>());
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("mock_ak_servo"), "Unhandled exception: %s",
                 ex.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
