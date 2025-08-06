#include "mr2_rover_control/rover_system.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mr2_rover_control
{
hardware_interface::CallbackReturn RoverSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cmd_.assign(info_.joints.size(), 0.0);
  pos_.assign(info_.joints.size(), 0.0);
  vel_.assign(info_.joints.size(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & ji = info_.joints[i];
    if (!ji.command_interfaces.empty()) {
      command_interfaces.emplace_back(ji.name, ji.command_interfaces[0].name, &cmd_[i]);
    }
  }
  return command_interfaces;
}

hardware_interface::return_type RoverSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Simple simulation: copy command to state
  pos_ = cmd_;
  vel_.assign(cmd_.size(), 0.0);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // No hardware to write to in this example
  return hardware_interface::return_type::OK;
}

}  // namespace mr2_rover_control

PLUGINLIB_EXPORT_CLASS(mr2_rover_control::RoverSystem, hardware_interface::SystemInterface)

