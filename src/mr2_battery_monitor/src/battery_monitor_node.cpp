/*
 * Battery monitor node for Makita XGT packs on the MR2 platform.
 *
 * Listens to CAN frames emitted by the STM32H523 firmware (IDs 0x300–0x314)
 * and republishes them as ROS 2 topics:
 *   - battery/telemetry  (mr2_battery_monitor::msg::PackTelemetry)
 *   - battery/state      (sensor_msgs::msg::BatteryState, optional)
 *
 * Parameters:
 *   can_iface (string, default: "can0")
 *     SocketCAN interface to bind to.
 *   publish_battery_state (bool, default: true)
 *     Whether to emit the standard BatteryState message alongside telemetry.
 *   publish_timer_ms (int, default: 100)
 *     Timer period that flushes newly received data to the ROS graph.
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <linux/can.h>

#include "mr2_battery_monitor/msg/pack_telemetry.hpp"
#include "mr2_can_bus_core/can_bus_registry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace std::chrono_literals;

namespace {
constexpr uint32_t kStdIdMask = 0x7FF;

float nanf_() { return std::numeric_limits<float>::quiet_NaN(); }
} // namespace

class BatteryMonitorNode : public rclcpp::Node {
public:
  BatteryMonitorNode() : rclcpp::Node("battery_monitor") {
    can_iface_ = declare_parameter<std::string>("can_iface", "can0");
    publish_battery_state_ =
        declare_parameter<bool>("publish_battery_state", true);
    publish_timer_ms_ = declare_parameter<int>("publish_timer_ms", 100);
    frame_id_ = declare_parameter<std::string>("battery_frame_id", "battery");
    summary_id_ =
        static_cast<uint32_t>(declare_parameter<int>("summary_can_id", 0x300));
    metadata_id_ =
        static_cast<uint32_t>(declare_parameter<int>("metadata_can_id", 0x301));
    cell_base_id_ = static_cast<uint32_t>(
        declare_parameter<int>("cell_base_can_id", 0x310));
    cell_last_id_ = cell_base_id_ + 4;
    if (publish_timer_ms_ <= 0) {
      publish_timer_ms_ = 100;
    }

    data_.cell_count = 10;
    reset_cells_locked(data_.cell_count);

    init_bus_();

    telemetry_pub_ = create_publisher<mr2_battery_monitor::msg::PackTelemetry>(
        "battery/telemetry", rclcpp::SensorDataQoS());
    if (publish_battery_state_) {
      battery_pub_ =
          create_publisher<sensor_msgs::msg::BatteryState>("battery/state", 10);
    }

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_timer_ms_),
        std::bind(&BatteryMonitorNode::publish_if_ready_, this));

    RCLCPP_INFO(
        get_logger(),
        "Battery monitor ready on %s frame_id='%s' "
        "(summary 0x%03X metadata 0x%03X cells 0x%03X-0x%03X) "
        "publishing every %d ms when new data arrives",
        can_iface_.c_str(), frame_id_.c_str(), summary_id_, metadata_id_,
        cell_base_id_, cell_last_id_, publish_timer_ms_);
  }

private:
  struct Telemetry {
    float soc_pct{nanf_()};
    float health_pct{nanf_()};
    float temperature_c{nanf_()};
    float voltage_v{nanf_()};
    uint16_t pack_life_cycles{0};
    uint32_t firmware_cycle{0};
    uint16_t nominal_cell_capacity_mah{0};
    uint8_t parallel_groups{1};
    uint8_t cell_count{10};
    std::vector<float> cell_mv;
    std::vector<bool> cell_valid;
    rclcpp::Time stamp{};
    bool has_summary{false};
    bool has_metadata{false};
  };

  void init_bus_() {
    bus_ = CanBusRegistry::get(can_iface_);
    if (!bus_) {
      throw std::runtime_error("Failed to acquire CAN bus on " + can_iface_);
    }

    bus_->register_listener(
        summary_id_, kStdIdMask,
        [this](const struct can_frame &fr) { handle_summary_(fr); });
    bus_->register_listener(
        metadata_id_, kStdIdMask,
        [this](const struct can_frame &fr) { handle_metadata_(fr); });
    for (uint32_t id = cell_base_id_; id <= cell_last_id_; ++id) {
      bus_->register_listener(
          id, kStdIdMask,
          [this](const struct can_frame &fr) { handle_cells_(fr); });
    }
  }

  void handle_summary_(const struct can_frame &fr) {
    if (fr.can_dlc < 8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Summary frame DLC %d < 8", fr.can_dlc);
      return;
    }

    const uint8_t *d = fr.data;
    const uint8_t soc = d[0];
    const uint8_t health = d[1];
    const int16_t temp_raw = static_cast<int16_t>(d[2] | (d[3] << 8));
    const uint16_t volt_raw = static_cast<uint16_t>(d[4] | (d[5] << 8));
    const uint16_t cycle_low = static_cast<uint16_t>(d[6] | (d[7] << 8));

    std::lock_guard<std::mutex> lk(mtx_);
    const uint32_t combined_cycle =
        (static_cast<uint32_t>(cycle_high_) << 16) | cycle_low;
    if (combined_cycle != last_cycle_) {
      last_cycle_ = combined_cycle;
      reset_cells_locked(data_.cell_count);
    }
    cycle_low_ = cycle_low;

    data_.soc_pct = static_cast<float>(soc);
    data_.health_pct = static_cast<float>(health);
    data_.temperature_c = static_cast<float>(temp_raw) / 10.0f;
    data_.voltage_v = static_cast<float>(volt_raw) / 100.0f;
    data_.firmware_cycle = combined_cycle;
    data_.stamp = now();
    data_.has_summary = true;
    publish_pending_ = true;
  }

  void handle_metadata_(const struct can_frame &fr) {
    if (fr.can_dlc < 8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Metadata frame DLC %d < 8", fr.can_dlc);
      return;
    }

    const uint8_t *d = fr.data;
    const uint16_t cap_mAh = static_cast<uint16_t>(d[0] | (d[1] << 8));
    const uint8_t parallel = d[2];
    uint8_t cell_count = d[3];
    const uint16_t pack_cycles = static_cast<uint16_t>(d[4] | (d[5] << 8));
    const uint16_t cycle_high = static_cast<uint16_t>(d[6] | (d[7] << 8));

    if (cell_count == 0) {
      cell_count = data_.cell_count;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    const bool cycle_changed =
        ((static_cast<uint32_t>(cycle_high) << 16) | cycle_low_) != last_cycle_;
    cycle_high_ = cycle_high;
    if (cycle_changed) {
      last_cycle_ = (static_cast<uint32_t>(cycle_high_) << 16) | cycle_low_;
      reset_cells_locked(cell_count);
    } else if (cell_count != data_.cell_count) {
      reset_cells_locked(cell_count);
    }

    data_.nominal_cell_capacity_mah = cap_mAh;
    data_.parallel_groups = (parallel == 0) ? 1 : parallel;
    data_.cell_count = cell_count;
    data_.pack_life_cycles = pack_cycles;
    data_.firmware_cycle = last_cycle_;
    data_.has_metadata = true;
    publish_pending_ = true;
  }

  void handle_cells_(const struct can_frame &fr) {
    if (fr.can_dlc < 8) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cell frame DLC %d < 8", fr.can_dlc);
      return;
    }

    auto update_slot = [&](uint8_t idx, uint16_t mv, uint8_t valid) {
      if (idx == 0 || idx > data_.cell_count) {
        return;
      }
      const size_t pos = static_cast<size_t>(idx - 1);
      if (pos >= data_.cell_mv.size()) {
        return;
      }
      if (valid && mv > 0) {
        data_.cell_mv[pos] = static_cast<float>(mv);
        data_.cell_valid[pos] = true;
      } else {
        data_.cell_mv[pos] = nanf_();
        data_.cell_valid[pos] = false;
      }
    };

    std::lock_guard<std::mutex> lk(mtx_);
    update_slot(fr.data[0],
                static_cast<uint16_t>(fr.data[1] | (fr.data[2] << 8)),
                fr.data[3]);
    update_slot(fr.data[4],
                static_cast<uint16_t>(fr.data[5] | (fr.data[6] << 8)),
                fr.data[7]);
    publish_pending_ = true;
  }

  void reset_cells_locked(uint8_t cell_count) {
    data_.cell_count = cell_count == 0 ? 10 : cell_count;
    data_.cell_mv.assign(data_.cell_count, nanf_());
    data_.cell_valid.assign(data_.cell_count, false);
  }

  void publish_if_ready_() {
    Telemetry snapshot;
    bool should_publish = false;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (publish_pending_ && data_.has_summary) {
        snapshot = data_;
        publish_pending_ = false;
        should_publish = true;
      }
    }
    if (!should_publish) {
      return;
    }

    mr2_battery_monitor::msg::PackTelemetry telem_msg;
    telem_msg.header.stamp = snapshot.stamp;
    telem_msg.header.frame_id = frame_id_;
    telem_msg.state_of_charge_pct = snapshot.soc_pct;
    telem_msg.health_pct = snapshot.health_pct;
    telem_msg.temperature_c = snapshot.temperature_c;
    telem_msg.pack_voltage_v = snapshot.voltage_v;
    telem_msg.pack_life_cycles = snapshot.pack_life_cycles;
    telem_msg.firmware_cycle_count = snapshot.firmware_cycle;
    telem_msg.nominal_cell_capacity_mah = snapshot.nominal_cell_capacity_mah;
    telem_msg.parallel_group_count = snapshot.parallel_groups;
    telem_msg.cell_count = snapshot.cell_count;
    telem_msg.cell_voltage_mv = snapshot.cell_mv;
    telem_msg.cell_voltage_valid = snapshot.cell_valid;

    telemetry_pub_->publish(telem_msg);

    if (battery_pub_) {
      sensor_msgs::msg::BatteryState bs;
      bs.header = telem_msg.header;
      bs.voltage = snapshot.voltage_v;
      bs.temperature = snapshot.temperature_c;
      bs.percentage = snapshot.soc_pct / 100.0f;
      bs.charge = nanf_();
      bs.current = nanf_();
      bs.design_capacity =
          static_cast<float>(snapshot.nominal_cell_capacity_mah) *
          static_cast<float>(snapshot.parallel_groups) / 1000.0f;
      bs.capacity = nanf_();
      bs.power_supply_status = bs.POWER_SUPPLY_STATUS_UNKNOWN;
      bs.power_supply_health = bs.POWER_SUPPLY_HEALTH_UNKNOWN;
      bs.power_supply_technology = bs.POWER_SUPPLY_TECHNOLOGY_LION;
      bs.present = true;
      bs.cell_voltage.resize(snapshot.cell_mv.size(), nanf_());
      for (size_t i = 0; i < snapshot.cell_mv.size(); ++i) {
        if (snapshot.cell_valid[i]) {
          bs.cell_voltage[i] = snapshot.cell_mv[i] / 1000.0f;
        }
      }
      battery_pub_->publish(bs);
    }
  }

  // Members
  std::shared_ptr<CanBusManager> bus_;
  Telemetry data_;
  std::mutex mtx_;
  bool publish_pending_{false};
  uint16_t cycle_low_{0};
  uint16_t cycle_high_{0};
  uint32_t last_cycle_{0};

  std::string can_iface_;
  std::string frame_id_;
  uint32_t summary_id_{0x300};
  uint32_t metadata_id_{0x301};
  uint32_t cell_base_id_{0x310};
  uint32_t cell_last_id_{0x314};
  bool publish_battery_state_{true};
  int publish_timer_ms_{100};

  rclcpp::Publisher<mr2_battery_monitor::msg::PackTelemetry>::SharedPtr
      telemetry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<BatteryMonitorNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("battery_monitor"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
