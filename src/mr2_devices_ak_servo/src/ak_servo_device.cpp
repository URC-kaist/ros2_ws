#include <cmath>
#include "mr2_can_bus_core/can_device.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mr2_devices_ak_servo
{

class AkServoDevice : public CanDevice
{
public:
  void configure(const hardware_interface::ComponentInfo & ji,
                 rclcpp::Node * node) override
  {
    node_   = node;
    id_     = std::stoi(ji.parameters.at("motor_id"));
    gear_   = std::stod(ji.parameters.at("gear_ratio"));
    iface_  = ji.parameters.at("can_iface");        // e.g. "can0"

    // obtain bus & store shared_ptr
    bus_ = CanBusRegistry::get(iface_, 1'000'000);
    if (!bus_) throw std::runtime_error("Cannot open CAN bus");

    // allocate state/command scalars before registering CAN callbacks to
    // avoid race conditions if a status frame arrives immediately.
    pos_.push_back(0.0); vel_.push_back(0.0);
    eff_.push_back(0.0);
    cmd_pos_.push_back(0.0); cmd_vel_.push_back(0.0);

    // register listener for periodic 0x29<ID> status frames
    add_filter(bus_, 0x00002900 | id_, 0x1FFFFFFF,
               [this](const can_frame & f){ on_status(f); });
  }

  void process(const rclcpp::Time &) override
  {
    struct can_frame fr{};
    fr.can_id  = (0x00000600 | id_) | CAN_EFF_FLAG;   // Control-ID 6
    fr.can_dlc = 8;
    int32_t p  = std::lround(cmd_pos_[0] * 180.0 / M_PI * gear_ * 1e4);
    int16_t v10 = std::lround(cmd_vel_[0] * gear_ * 3.0 / M_PI);   // rad/s→0.1rpm
    fr.data[0] = (p >> 24) & 0xFF;
    fr.data[1] = (p >> 16) & 0xFF;
    fr.data[2] = (p >>  8) & 0xFF;
    fr.data[3] = (p      ) & 0xFF;
    fr.data[4] = (v10 >> 8) & 0xFF;
    fr.data[5] = (v10     ) & 0xFF;
    fr.data[6] = fr.data[7] = 0;   // acceleration = 0
    send(fr, bus_);
  }

  void export_state(std::vector<double*> & pos,
                    std::vector<double*> & vel,
                    std::vector<double*> & eff) override
  {
    pos.push_back(&pos_[0]);
    vel.push_back(&vel_[0]);
    eff.push_back(&eff_[0]);
  }

  void export_command(std::vector<double*> & cmd_pos,
                      std::vector<double*> & cmd_vel) override
  {
    cmd_pos.push_back(&cmd_pos_[0]);
    cmd_vel.push_back(&cmd_vel_[0]);
  }

private:
  /* -------- CAN status-frame callback -------- */
  void on_status(const can_frame & f)
  {
    int16_t p10 = (f.data[0] << 8) | f.data[1];
    int16_t v10 = (f.data[2] << 8) | f.data[3];
    int16_t c01 = (f.data[4] << 8) | f.data[5];

    pos_[0] = (p10 / 10.0) / gear_ * (M_PI / 180.0);
    vel_[0] = (v10 * 10.0) / gear_ * (M_PI / 30.0);   // rpm→rad/s
    eff_[0] = c01 / 100.0;                            // amps
  }

  /* members */
  rclcpp::Node * node_;
  std::shared_ptr<CanBusManager> bus_;
  std::string iface_;
  int    id_{0};
  double gear_{1.0};

  std::vector<double> pos_, vel_, eff_, cmd_pos_, cmd_vel_;
};

}  // namespace mr2_devices_ak_servo

PLUGINLIB_EXPORT_CLASS(mr2_devices_ak_servo::AkServoDevice, CanDevice)
