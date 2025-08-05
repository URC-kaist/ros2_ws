#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"

#include "mr2_can_bus_core/can_device.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace mr2_can_hardware_interface
{

class CanHW : public hardware_interface::SystemInterface
{
public:
  CanHW() = default;
  ~CanHW() override = default;

  // ---------------- Lifecycle hooks ----------------------------------
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    // Local node for parameters / logging
    node_   = rclcpp::Node::make_shared("can_hw");

    // Device plugin loader (package "can_bus_core", base class CanDevice)
    try {
      loader_ = std::make_shared<pluginlib::ClassLoader<CanDevice>>("can_bus_core", "CanDevice");
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Pluginlib load error: %s", ex.what());
      return CallbackReturn::ERROR;
    }

    // --- create device instances, one per <joint> block ----------------
    for (const auto & joint : info_.joints) {
      const auto & params = joint.parameters;
      auto it = params.find("device_plugin");
      if (it == params.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s missing <device_plugin> param", joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      const std::string & plugin_name = it->second;

      std::shared_ptr<CanDevice> dev;
      try {
        dev = loader_->createSharedInstance(plugin_name);
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load device plugin %s: %s",
                     plugin_name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }

      try {
        dev->configure(joint, node_.get());
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Device %s configure() threw: %s", joint.name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }

      devs_.push_back(std::move(dev));
    }

    // collect state / command pointers --------------------------------
    for (auto & dev : devs_) {
      dev->export_state(pos_ptrs_, vel_ptrs_, eff_ptrs_);
      dev->export_command(cmd_ptrs_);
    }

    if (pos_ptrs_.size() != info_.joints.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Mismatch: %zu joints vs %zu pos-ptrs", info_.joints.size(), pos_ptrs_.size());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node_->get_logger(), "CanHW initialised: %zu joints, %zu device plugins",
                info_.joints.size(), devs_.size());
    return CallbackReturn::SUCCESS;
  }

  // ---------------- Interface export ----------------------------------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> sis;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      sis.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, pos_ptrs_[i]);
      sis.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, vel_ptrs_[i]);
      sis.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   eff_ptrs_[i]);
    }
    return sis;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> cis;
    for (size_t i = 0; i < info_.joints.size(); ++i)
      cis.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, cmd_ptrs_[i]);
    return cis;
  }

  // ---------------- Read / Write --------------------------------------
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // state already populated asynchronously by device callbacks
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & now, const rclcpp::Duration &) override
  {
    for (auto & dev : devs_) dev->process(now);
    return return_type::OK;
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<CanDevice>> loader_;
  std::vector<std::shared_ptr<CanDevice>> devs_;

  std::vector<double*> pos_ptrs_, vel_ptrs_, eff_ptrs_, cmd_ptrs_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace mr2_can_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::CanHW, hardware_interface::SystemInterface)

