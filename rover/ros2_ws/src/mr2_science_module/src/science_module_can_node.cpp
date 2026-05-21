#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <linux/can.h>

#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "mr2_science_module/msg/carriage_motor_telemetry.hpp"
#include "mr2_science_module/srv/carriage_motor_command.hpp"
#include "mr2_science_module/srv/motor_position.hpp"
#include "mr2_science_module/srv/motor_velocity.hpp"
#include "mr2_science_module/srv/move_carriage.hpp"
#include "mr2_science_module/srv/pump.hpp"
#include "mr2_science_module/srv/select_centrifuge_position.hpp"
#include "mr2_science_module/srv/set_science_led.hpp"
#include "mr2_science_module/srv/trigger_centrifuge_ramp.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
constexpr uint32_t kStdIdMask = 0x7FF;
constexpr double kArcminPerRad = 10800.0 / M_PI;

bool valid_std_id(int can_id) {
  return can_id >= 0 && can_id <= static_cast<int>(kStdIdMask);
}

void encode_u16_le(can_frame &frame, size_t offset, uint16_t value) {
  frame.data[offset] = static_cast<uint8_t>(value & 0xFF);
  frame.data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void encode_i32_le(can_frame &frame, size_t offset, int32_t value) {
  const auto raw = static_cast<uint32_t>(value);
  frame.data[offset] = static_cast<uint8_t>(raw & 0xFF);
  frame.data[offset + 1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  frame.data[offset + 2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  frame.data[offset + 3] = static_cast<uint8_t>((raw >> 24) & 0xFF);
}

int32_t decode_i32_le(const can_frame &frame, size_t offset) {
  const uint32_t raw = static_cast<uint32_t>(frame.data[offset]) |
                       (static_cast<uint32_t>(frame.data[offset + 1]) << 8) |
                       (static_cast<uint32_t>(frame.data[offset + 2]) << 16) |
                       (static_cast<uint32_t>(frame.data[offset + 3]) << 24);
  return static_cast<int32_t>(raw);
}

bool valid_classic_frame(const can_frame &frame, uint32_t expected_id,
                         uint8_t expected_dlc) {
  return (frame.can_id & CAN_EFF_FLAG) == 0 &&
         (frame.can_id & CAN_RTR_FLAG) == 0 &&
         (frame.can_id & kStdIdMask) == expected_id &&
         frame.can_dlc == expected_dlc;
}

bool radians_to_arcmin(double radians, int32_t &out) {
  if (!std::isfinite(radians)) {
    return false;
  }
  const double converted = std::round(radians * kArcminPerRad);
  if (converted < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      converted > static_cast<double>(std::numeric_limits<int32_t>::max())) {
    return false;
  }
  out = static_cast<int32_t>(converted);
  return true;
}

} // namespace

class ScienceModuleCanNode : public rclcpp::Node {
public:
  ScienceModuleCanNode() : rclcpp::Node("science_module_can") {
    can_iface_ = declare_parameter<std::string>("can_iface", "can0");
    centrifuge_module_rx_id_ =
        read_can_id_parameter("centrifuge_module_rx_id", 0x400);
    carriage_module_rx_id_ =
        read_can_id_parameter("carriage_module_rx_id", 0x500);
    carriage_motor_rx_id_ =
        read_can_id_parameter("carriage_motor_rx_id", 0x650);
    carriage_motor_velocity_tx_id_ =
        read_can_id_parameter("carriage_motor_velocity_tx_id", 0x651);
    carriage_motor_position_tx_id_ =
        read_can_id_parameter("carriage_motor_position_tx_id", 0x652);
    centrifuge_motor_rx_id_ =
        read_can_id_parameter("centrifuge_motor_rx_id", 0x600);
    drill_motor_rx_id_ = read_can_id_parameter("drill_motor_rx_id", 0x700);

    bus_ = CanBusRegistry::get(can_iface_);
    if (!bus_) {
      throw std::runtime_error("Failed to acquire CAN bus on " + can_iface_);
    }

    telemetry_pub_ =
        create_publisher<mr2_science_module::msg::CarriageMotorTelemetry>(
            "science/carriage_motor/telemetry", rclcpp::SystemDefaultsQoS());

    bus_->register_listener(
        carriage_motor_velocity_tx_id_, kStdIdMask,
        [this](const can_frame &frame) { handle_carriage_motor_telemetry(frame); });
    bus_->register_listener(
        carriage_motor_position_tx_id_, kStdIdMask,
        [this](const can_frame &frame) { handle_carriage_motor_telemetry(frame); });

    pump_srv_ = create_service<mr2_science_module::srv::Pump>(
        "science/pump",
        [this](const std::shared_ptr<mr2_science_module::srv::Pump::Request> req,
               std::shared_ptr<mr2_science_module::srv::Pump::Response> res) {
          handle_pump(req, res);
        });
    led_srv_ = create_service<mr2_science_module::srv::SetScienceLed>(
        "science/led",
        [this](
            const std::shared_ptr<mr2_science_module::srv::SetScienceLed::Request>
                req,
            std::shared_ptr<mr2_science_module::srv::SetScienceLed::Response>
                res) { handle_led(req, res); });
    centrifuge_position_srv_ =
        create_service<mr2_science_module::srv::SelectCentrifugePosition>(
            "science/centrifuge_position",
            [this](const std::shared_ptr<
                       mr2_science_module::srv::SelectCentrifugePosition::Request>
                       req,
                   std::shared_ptr<
                       mr2_science_module::srv::SelectCentrifugePosition::Response>
                       res) { handle_centrifuge_position(req, res); });
    centrifuge_ramp_srv_ =
        create_service<mr2_science_module::srv::TriggerCentrifugeRamp>(
            "science/centrifuge_ramp",
            [this](const std::shared_ptr<
                       mr2_science_module::srv::TriggerCentrifugeRamp::Request>
                       req,
                   std::shared_ptr<
                       mr2_science_module::srv::TriggerCentrifugeRamp::Response>
                       res) { handle_centrifuge_ramp(req, res); });
    carriage_srv_ = create_service<mr2_science_module::srv::MoveCarriage>(
        "science/carriage",
        [this](
            const std::shared_ptr<mr2_science_module::srv::MoveCarriage::Request>
                req,
            std::shared_ptr<mr2_science_module::srv::MoveCarriage::Response>
                res) { handle_carriage(req, res); });
    carriage_motor_srv_ =
        create_service<mr2_science_module::srv::CarriageMotorCommand>(
            "science/carriage_motor",
            [this](const std::shared_ptr<
                       mr2_science_module::srv::CarriageMotorCommand::Request>
                       req,
                   std::shared_ptr<
                       mr2_science_module::srv::CarriageMotorCommand::Response>
                       res) { handle_carriage_motor(req, res); });
    drill_velocity_srv_ =
        create_service<mr2_science_module::srv::MotorVelocity>(
            "science/drill_velocity",
            [this](
                const std::shared_ptr<
                    mr2_science_module::srv::MotorVelocity::Request> req,
                std::shared_ptr<mr2_science_module::srv::MotorVelocity::Response>
                    res) { handle_motor_velocity(drill_motor_rx_id_, req, res); });
    drill_position_srv_ =
        create_service<mr2_science_module::srv::MotorPosition>(
            "science/drill_position",
            [this](
                const std::shared_ptr<
                    mr2_science_module::srv::MotorPosition::Request> req,
                std::shared_ptr<mr2_science_module::srv::MotorPosition::Response>
                    res) { handle_motor_position(drill_motor_rx_id_, req, res); });
    debug_centrifuge_motor_velocity_srv_ =
        create_service<mr2_science_module::srv::MotorVelocity>(
            "science/debug_centrifuge_motor_velocity",
            [this](
                const std::shared_ptr<
                    mr2_science_module::srv::MotorVelocity::Request> req,
                std::shared_ptr<mr2_science_module::srv::MotorVelocity::Response>
                    res) {
              handle_motor_velocity(centrifuge_motor_rx_id_, req, res);
            });
    debug_centrifuge_motor_position_srv_ =
        create_service<mr2_science_module::srv::MotorPosition>(
            "science/debug_centrifuge_motor_position",
            [this](
                const std::shared_ptr<
                    mr2_science_module::srv::MotorPosition::Request> req,
                std::shared_ptr<mr2_science_module::srv::MotorPosition::Response>
                    res) {
              handle_motor_position(centrifuge_motor_rx_id_, req, res);
            });

    RCLCPP_INFO(
        get_logger(),
        "science_module_can ready on %s: centrifuge module 0x%03X, carriage "
        "module 0x%03X, carriage motor 0x%03X, drill 0x%03X",
        can_iface_.c_str(), centrifuge_module_rx_id_, carriage_module_rx_id_,
        carriage_motor_rx_id_, drill_motor_rx_id_);
  }

private:
  uint32_t read_can_id_parameter(const std::string &name, int default_value) {
    const int value = declare_parameter<int>(name, default_value);
    if (!valid_std_id(value)) {
      throw std::runtime_error("Invalid CAN ID for " + name + ": " +
                               std::to_string(value));
    }
    return static_cast<uint32_t>(value);
  }

  void send_frame(uint32_t can_id, uint8_t dlc, const uint8_t *data) {
    can_frame frame {};
    frame.can_id = can_id & kStdIdMask;
    frame.can_dlc = dlc;
    for (uint8_t i = 0; i < dlc; ++i) {
      frame.data[i] = data[i];
    }
    bus_->enqueue_tx(frame);
  }

  void handle_pump(
      const std::shared_ptr<mr2_science_module::srv::Pump::Request> req,
      std::shared_ptr<mr2_science_module::srv::Pump::Response> res) {
    uint8_t command = 0;
    if (req->group == mr2_science_module::srv::Pump::Request::GROUP_PUMPS_2_4) {
      command = 0x07;
    } else if (req->group ==
               mr2_science_module::srv::Pump::Request::GROUP_PUMPS_1_3) {
      command = 0x08;
    } else {
      fail(res, "Invalid pump group");
      return;
    }

    can_frame frame {};
    frame.can_id = centrifuge_module_rx_id_;
    frame.data[0] = command;
    if (req->duration_ms == 0) {
      frame.can_dlc = 1;
    } else {
      frame.can_dlc = 3;
      encode_u16_le(frame, 1, req->duration_ms);
    }
    bus_->enqueue_tx(frame);
    ok(res, "Sent");
  }

  void handle_led(
      const std::shared_ptr<mr2_science_module::srv::SetScienceLed::Request> req,
      std::shared_ptr<mr2_science_module::srv::SetScienceLed::Response> res) {
    if (req->enabled) {
      const uint8_t data[2] = {0x09, req->brightness};
      send_frame(centrifuge_module_rx_id_, 2, data);
    } else {
      const uint8_t data[1] = {0x0C};
      send_frame(centrifuge_module_rx_id_, 1, data);
    }
    ok(res, "Sent");
  }

  void handle_centrifuge_position(
      const std::shared_ptr<
          mr2_science_module::srv::SelectCentrifugePosition::Request> req,
      std::shared_ptr<
          mr2_science_module::srv::SelectCentrifugePosition::Response> res) {
    if (req->index > 7) {
      fail(res, "Centrifuge position index must be 0..7");
      return;
    }
    const uint8_t data[2] = {0x0B, req->index};
    send_frame(centrifuge_module_rx_id_, 2, data);
    ok(res, "Sent");
  }

  void handle_centrifuge_ramp(
      const std::shared_ptr<
          mr2_science_module::srv::TriggerCentrifugeRamp::Request> req,
      std::shared_ptr<mr2_science_module::srv::TriggerCentrifugeRamp::Response>
          res) {
    if (!req->start) {
      fail(res, "Only start=true is supported by the firmware");
      return;
    }
    const uint8_t data[1] = {0x0A};
    send_frame(centrifuge_module_rx_id_, 1, data);
    ok(res, "Sent");
  }

  void handle_carriage(
      const std::shared_ptr<mr2_science_module::srv::MoveCarriage::Request> req,
      std::shared_ptr<mr2_science_module::srv::MoveCarriage::Response> res) {
    if (req->command > 6) {
      fail(res, "Carriage command must be 0..6");
      return;
    }

    can_frame frame {};
    frame.can_id = carriage_module_rx_id_;
    frame.data[0] = req->command;
    if (req->command == 0) {
      frame.can_dlc = 1;
    } else {
      frame.can_dlc = 3;
      encode_u16_le(frame, 1, req->vibration_duration_ms);
    }
    bus_->enqueue_tx(frame);
    ok(res, "Sent");
  }

  void handle_carriage_motor(
      const std::shared_ptr<
          mr2_science_module::srv::CarriageMotorCommand::Request> req,
      std::shared_ptr<mr2_science_module::srv::CarriageMotorCommand::Response>
          res) {
    if (req->mode < 1 || req->mode > 4) {
      fail(res, "Carriage motor mode must be 1..4");
      return;
    }

    can_frame frame {};
    frame.can_id = carriage_motor_rx_id_;
    frame.data[0] = req->mode;
    if (req->mode == 1 || req->mode == 2) {
      frame.can_dlc = 5;
      encode_i32_le(frame, 1, req->value);
    } else {
      frame.can_dlc = 1;
    }
    bus_->enqueue_tx(frame);
    ok(res, "Sent");
  }

  void handle_motor_velocity(
      uint32_t motor_can_id,
      const std::shared_ptr<mr2_science_module::srv::MotorVelocity::Request>
          req,
      std::shared_ptr<mr2_science_module::srv::MotorVelocity::Response> res) {
    int32_t arcmin_s = 0;
    if (!radians_to_arcmin(req->rad_s, arcmin_s)) {
      fail(res, "Velocity must be finite and fit in int32 arcmin/s");
      return;
    }
    send_motor_command(motor_can_id, 0x01, arcmin_s);
    ok(res, "Sent");
  }

  void handle_motor_position(
      uint32_t motor_can_id,
      const std::shared_ptr<mr2_science_module::srv::MotorPosition::Request> req,
      std::shared_ptr<mr2_science_module::srv::MotorPosition::Response> res) {
    int32_t arcmin = 0;
    if (!radians_to_arcmin(req->rad, arcmin)) {
      fail(res, "Position must be finite and fit in int32 arcmin");
      return;
    }
    send_motor_command(motor_can_id, 0x02, arcmin);
    ok(res, "Sent");
  }

  void send_motor_command(uint32_t motor_can_id, uint8_t mode, int32_t value) {
    can_frame frame {};
    frame.can_id = motor_can_id;
    frame.can_dlc = 5;
    frame.data[0] = mode;
    encode_i32_le(frame, 1, value);
    bus_->enqueue_tx(frame);
  }

  void handle_carriage_motor_telemetry(const can_frame &frame) {
    const uint32_t frame_id = frame.can_id & kStdIdMask;
    if (!valid_classic_frame(frame, frame_id, 8) ||
        (frame_id != carriage_motor_velocity_tx_id_ &&
         frame_id != carriage_motor_position_tx_id_)) {
      return;
    }

    mr2_science_module::msg::CarriageMotorTelemetry msg;
    msg.stamp = get_clock()->now();
    msg.position_um = decode_i32_le(frame, 0);
    msg.velocity_um_s = decode_i32_le(frame, 4);
    telemetry_pub_->publish(msg);
  }

  template <typename ResponseT>
  void ok(std::shared_ptr<ResponseT> response, const std::string &message) {
    response->success = true;
    response->message = message;
  }

  template <typename ResponseT>
  void fail(std::shared_ptr<ResponseT> response, const std::string &message) {
    response->success = false;
    response->message = message;
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }

  std::string can_iface_;
  uint32_t centrifuge_module_rx_id_{0};
  uint32_t carriage_module_rx_id_{0};
  uint32_t carriage_motor_rx_id_{0};
  uint32_t carriage_motor_velocity_tx_id_{0};
  uint32_t carriage_motor_position_tx_id_{0};
  uint32_t centrifuge_motor_rx_id_{0};
  uint32_t drill_motor_rx_id_{0};

  std::shared_ptr<CanBusManager> bus_;
  rclcpp::Publisher<mr2_science_module::msg::CarriageMotorTelemetry>::SharedPtr
      telemetry_pub_;

  rclcpp::Service<mr2_science_module::srv::Pump>::SharedPtr pump_srv_;
  rclcpp::Service<mr2_science_module::srv::SetScienceLed>::SharedPtr led_srv_;
  rclcpp::Service<mr2_science_module::srv::SelectCentrifugePosition>::SharedPtr
      centrifuge_position_srv_;
  rclcpp::Service<mr2_science_module::srv::TriggerCentrifugeRamp>::SharedPtr
      centrifuge_ramp_srv_;
  rclcpp::Service<mr2_science_module::srv::MoveCarriage>::SharedPtr
      carriage_srv_;
  rclcpp::Service<mr2_science_module::srv::CarriageMotorCommand>::SharedPtr
      carriage_motor_srv_;
  rclcpp::Service<mr2_science_module::srv::MotorVelocity>::SharedPtr
      drill_velocity_srv_;
  rclcpp::Service<mr2_science_module::srv::MotorPosition>::SharedPtr
      drill_position_srv_;
  rclcpp::Service<mr2_science_module::srv::MotorVelocity>::SharedPtr
      debug_centrifuge_motor_velocity_srv_;
  rclcpp::Service<mr2_science_module::srv::MotorPosition>::SharedPtr
      debug_centrifuge_motor_position_srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ScienceModuleCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(rclcpp::get_logger("science_module_can"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
