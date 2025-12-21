/*
 * Battery emulator node for simulation.
 *
 * Publishes synthetic PackTelemetry and BatteryState messages at a fixed
 * rate so that downstream nodes can run without real battery hardware.
 *
 * Parameters:
 *   publish_timer_ms (int, default 1000)
 *     Period in milliseconds between publishes.
 *   battery_frame_id (string, default "battery")
 *     Frame placed in the header of emitted messages.
 *   state_of_charge_pct (double, default 80.0)
 *   health_pct (double, default 90.0)
 *   temperature_c (double, default 25.0)
 *   pack_voltage_v (double, default 40.0)
 *   pack_life_cycles (int, default 40)
 *   firmware_cycle_start (int, default 0)
 *   nominal_cell_capacity_mah (int, default 4000)
 *   parallel_group_count (int, default 2)
 *   cell_count (int, default 10)
 *   cell_voltage_mv (double[], default empty)
 *     Optional per-cell voltages in millivolts. When empty, values are
 *     derived from pack_voltage_v / cell_count.
 *   publish_battery_state (bool, default true)
 *     Emit sensor_msgs/BatteryState alongside PackTelemetry.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "mr2_battery_monitor/msg/pack_telemetry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;

namespace {
float nanf_() { return std::numeric_limits<float>::quiet_NaN(); }
} // namespace

class BatteryEmulatorNode : public rclcpp::Node {
public:
  BatteryEmulatorNode() : rclcpp::Node("battery_emulator") {
    frame_id_ = declare_parameter<std::string>("battery_frame_id", "battery");
    publish_timer_ms_ = declare_parameter<int>("publish_timer_ms", 1000);
    if (publish_timer_ms_ <= 0) {
      publish_timer_ms_ = 1000;
    }

    soc_pct_ = declare_parameter<double>("state_of_charge_pct", 80.0);
    health_pct_ = declare_parameter<double>("health_pct", 90.0);
    temperature_c_ = declare_parameter<double>("temperature_c", 25.0);
    pack_voltage_v_ = declare_parameter<double>("pack_voltage_v", 40.0);
    pack_life_cycles_ = declare_parameter<int>("pack_life_cycles", 40);
    firmware_cycle_start_ = declare_parameter<int>("firmware_cycle_start", 0);
    nominal_cell_capacity_mah_ =
        declare_parameter<int>("nominal_cell_capacity_mah", 4000);
    parallel_group_count_ =
        declare_parameter<int>("parallel_group_count", 2);
    cell_count_ = declare_parameter<int>("cell_count", 10);
    if (cell_count_ <= 0) {
      RCLCPP_WARN(get_logger(),
                  "cell_count must be positive; defaulting to 10");
      cell_count_ = 10;
    }

    auto cell_voltage_param =
        declare_parameter<std::vector<double>>("cell_voltage_mv", {});
    build_cell_voltage_(cell_voltage_param);

    publish_battery_state_ =
        declare_parameter<bool>("publish_battery_state", true);

    telemetry_pub_ = create_publisher<mr2_battery_monitor::msg::PackTelemetry>(
        "battery/telemetry", 10);
    if (publish_battery_state_) {
      battery_pub_ =
          create_publisher<sensor_msgs::msg::BatteryState>("battery/state", 10);
    }

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_timer_ms_),
        std::bind(&BatteryEmulatorNode::publish_once_, this));

    RCLCPP_INFO(
        get_logger(),
        "Battery emulator publishing every %d ms (%d cells, %.1f V, %.1f %%)",
        publish_timer_ms_, cell_count_, pack_voltage_v_, soc_pct_);
  }

private:
  void build_cell_voltage_(const std::vector<double> &param) {
    cell_voltage_mv_.assign(
        static_cast<size_t>(cell_count_),
        static_cast<float>((pack_voltage_v_ / static_cast<double>(cell_count_)) *
                           1000.0));

    if (!param.empty()) {
      if (static_cast<int>(param.size()) != cell_count_) {
        RCLCPP_WARN(
            get_logger(),
            "cell_voltage_mv size (%zu) does not match cell_count (%d); "
            "resizing to fit",
            param.size(), cell_count_);
      }
      cell_voltage_mv_.resize(static_cast<size_t>(cell_count_), nanf_());
      for (size_t i = 0; i < std::min(param.size(), cell_voltage_mv_.size());
           ++i) {
        cell_voltage_mv_[i] = static_cast<float>(param[i]);
      }
    }
  }

  void publish_once_() {
    mr2_battery_monitor::msg::PackTelemetry telem;
    telem.header.stamp = now();
    telem.header.frame_id = frame_id_;
    telem.state_of_charge_pct = static_cast<float>(soc_pct_);
    telem.health_pct = static_cast<float>(health_pct_);
    telem.temperature_c = static_cast<float>(temperature_c_);
    telem.pack_voltage_v = static_cast<float>(pack_voltage_v_);
    telem.pack_life_cycles = static_cast<uint16_t>(
        std::max(0, pack_life_cycles_));
    telem.firmware_cycle_count =
        static_cast<uint32_t>(std::max(0, firmware_cycle_start_) +
                              publish_count_);
    telem.nominal_cell_capacity_mah =
        static_cast<uint16_t>(std::max(0, nominal_cell_capacity_mah_));
    telem.parallel_group_count =
        static_cast<uint8_t>(std::max(1, parallel_group_count_));
    telem.cell_count = static_cast<uint8_t>(cell_count_);
    telem.cell_voltage_mv = cell_voltage_mv_;
    telem.cell_voltage_valid.assign(cell_voltage_mv_.size(), true);

    telemetry_pub_->publish(telem);

    if (battery_pub_) {
      sensor_msgs::msg::BatteryState bs;
      bs.header = telem.header;
      bs.voltage = telem.pack_voltage_v;
      bs.temperature = telem.temperature_c;
      bs.percentage = telem.state_of_charge_pct / 100.0f;
      bs.charge = nanf_();
      bs.current = nanf_();
      bs.design_capacity =
          static_cast<float>(telem.nominal_cell_capacity_mah) *
          static_cast<float>(telem.parallel_group_count) / 1000.0f;
      bs.capacity = nanf_();
      bs.power_supply_status = bs.POWER_SUPPLY_STATUS_UNKNOWN;
      bs.power_supply_health = bs.POWER_SUPPLY_HEALTH_UNKNOWN;
      bs.power_supply_technology = bs.POWER_SUPPLY_TECHNOLOGY_LION;
      bs.present = true;
      bs.cell_voltage.resize(telem.cell_voltage_mv.size(), nanf_());
      for (size_t i = 0; i < telem.cell_voltage_mv.size(); ++i) {
        bs.cell_voltage[i] = telem.cell_voltage_mv[i] / 1000.0f;
      }
      battery_pub_->publish(bs);
    }

    ++publish_count_;
  }

  std::string frame_id_;
  int publish_timer_ms_{1000};
  double soc_pct_{80.0};
  double health_pct_{90.0};
  double temperature_c_{25.0};
  double pack_voltage_v_{40.0};
  int pack_life_cycles_{40};
  int firmware_cycle_start_{0};
  int nominal_cell_capacity_mah_{4000};
  int parallel_group_count_{2};
  int cell_count_{10};
  bool publish_battery_state_{true};

  std::vector<float> cell_voltage_mv_;
  uint32_t publish_count_{0};

  rclcpp::Publisher<mr2_battery_monitor::msg::PackTelemetry>::SharedPtr
      telemetry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryEmulatorNode>());
  rclcpp::shutdown();
  return 0;
}
