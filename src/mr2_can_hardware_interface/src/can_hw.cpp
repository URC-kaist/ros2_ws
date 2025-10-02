#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"

#include "transmission_interface/four_bar_linkage_transmission_loader.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

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

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("can_hw");

    try
    {
      loader_ = std::make_shared<pluginlib::ClassLoader<CanDevice>>("mr2_can_bus_core", "CanDevice");
    }
    catch (const pluginlib::PluginlibException & ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Pluginlib load error: %s", ex.what());
      return CallbackReturn::ERROR;
    }

    joint_index_.clear();
    actuator_index_.clear();
    joints_.clear();
    actuators_.clear();
    transmissions_.clear();
    pos_ptrs_.clear();
    vel_ptrs_.clear();
    eff_ptrs_.clear();
    cmd_ptrs_.clear();
    devs_.clear();

    transmission_interface::SimpleTransmissionLoader simple_loader;
    transmission_interface::FourBarLinkageTransmissionLoader four_bar_loader;

    for (const auto & joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s must expose a single position command interface",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1 ||
          joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s must expose a single position state interface",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }

      auto & joint_data = get_joint(joint.name);

      const auto plugin_it = joint.parameters.find("device_plugin");
      if (plugin_it == joint.parameters.end())
      {
        RCLCPP_ERROR(node_->get_logger(),
          "Joint %s missing <param name=\"device_plugin\">", joint.name.c_str());
        return CallbackReturn::ERROR;
      }

      const auto actuator_it = joint.parameters.find("actuator");
      joint_data.actuator_name =
        actuator_it != joint.parameters.end() ? actuator_it->second : joint.name;

      auto & actuator = get_actuator(joint_data.actuator_name);

      if (!actuator.configured)
      {
        std::shared_ptr<CanDevice> dev;
        try
        {
          dev = loader_->createSharedInstance(plugin_it->second);
        }
        catch (const pluginlib::PluginlibException & ex)
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to load device plugin %s: %s",
            plugin_it->second.c_str(), ex.what());
          return CallbackReturn::ERROR;
        }

        try
        {
          dev->configure(joint, node_.get());
        }
        catch (const std::exception & ex)
        {
          RCLCPP_ERROR(node_->get_logger(), "Device %s configure() threw: %s",
            joint.name.c_str(), ex.what());
          return CallbackReturn::ERROR;
        }

        const auto pos_idx = pos_ptrs_.size();
        const auto vel_idx = vel_ptrs_.size();
        const auto eff_idx = eff_ptrs_.size();
        const auto cmd_idx = cmd_ptrs_.size();

        dev->export_state(pos_ptrs_, vel_ptrs_, eff_ptrs_);
        dev->export_command(cmd_ptrs_);

        actuator.state_ptr = pos_ptrs_.size() > pos_idx ? pos_ptrs_[pos_idx] : nullptr;
        actuator.velocity_ptr = vel_ptrs_.size() > vel_idx ? vel_ptrs_[vel_idx] : nullptr;
        actuator.effort_ptr = eff_ptrs_.size() > eff_idx ? eff_ptrs_[eff_idx] : nullptr;
        actuator.command_ptr = cmd_ptrs_.size() > cmd_idx ? cmd_ptrs_[cmd_idx] : nullptr;

        if (!actuator.state_ptr || !actuator.command_ptr)
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Device for actuator %s failed to export state/command interfaces",
            actuator.name.c_str());
          return CallbackReturn::ERROR;
        }

        actuator.configured = true;
        devs_.push_back(std::move(dev));
      }
    }

    for (const auto & transmission_info : info_.transmissions)
    {
      std::shared_ptr<transmission_interface::Transmission> transmission;
      try
      {
        if (transmission_info.type == "transmission_interface/SimpleTransmission")
        {
          transmission = simple_loader.load(transmission_info);
        }
        else if (transmission_info.type == "transmission_interface/FourBarLinkageTransmission")
        {
          transmission = four_bar_loader.load(transmission_info);
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "Unsupported transmission type '%s'",
            transmission_info.type.c_str());
          return CallbackReturn::ERROR;
        }
      }
      catch (const transmission_interface::TransmissionInterfaceException & ex)
      {
        RCLCPP_ERROR(node_->get_logger(), "Error while loading transmission '%s': %s",
          transmission_info.name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }

      if (!transmission)
      {
        RCLCPP_ERROR(node_->get_logger(),
          "Loader returned null transmission for '%s'", transmission_info.name.c_str());
        return CallbackReturn::ERROR;
      }

      std::vector<transmission_interface::JointHandle> joint_handles;
      joint_handles.reserve(transmission_info.joints.size());

      for (const auto & joint_info : transmission_info.joints)
      {
        auto & joint = get_joint(joint_info.name);
        joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_POSITION,
          &joint.transmission_passthrough);
      }

      std::vector<transmission_interface::ActuatorHandle> actuator_handles;
      actuator_handles.reserve(transmission_info.actuators.size());

      for (const auto & actuator_info : transmission_info.actuators)
      {
        auto & actuator = get_actuator(actuator_info.name);

        if (!actuator.configured)
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Transmission '%s' references actuator '%s' with no configured device",
            transmission_info.name.c_str(), actuator_info.name.c_str());
          return CallbackReturn::ERROR;
        }

        actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_POSITION,
          &actuator.transmission_passthrough);
      }

      try
      {
        transmission->configure(joint_handles, actuator_handles);
      }
      catch (const transmission_interface::TransmissionInterfaceException & ex)
      {
        RCLCPP_ERROR(node_->get_logger(), "Error while configuring transmission '%s': %s",
          transmission_info.name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }

      transmissions_.push_back(std::move(transmission));
    }

    if (transmissions_.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "No transmissions defined for CAN hardware interface");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node_->get_logger(), "CanHW initialised: %zu joints, %zu actuators, %zu transmissions",
      joints_.size(), actuators_.size(), transmissions_.size());

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(info_.joints.size());

    for (const auto & joint : info_.joints)
    {
      auto it = joint_index_.find(joint.name);
      if (it == joint_index_.end())
      {
        continue;
      }

      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
        &joints_[it->second].state);
    }

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(info_.joints.size());

    for (const auto & joint : info_.joints)
    {
      auto it = joint_index_.find(joint.name);
      if (it == joint_index_.end())
      {
        continue;
      }

      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
        &joints_[it->second].command);
    }

    return interfaces;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (auto & actuator : actuators_)
    {
      if (actuator.state_ptr)
      {
        actuator.state = *actuator.state_ptr;
      }

      actuator.transmission_passthrough = actuator.state;
    }

    for (auto & transmission : transmissions_)
    {
      transmission->actuator_to_joint();
    }

    for (auto & joint : joints_)
    {
      joint.state = joint.transmission_passthrough;
    }

    return return_type::OK;
  }

  return_type write(const rclcpp::Time & now, const rclcpp::Duration &) override
  {
    for (auto & joint : joints_)
    {
      joint.transmission_passthrough = joint.command;
    }

    for (auto & transmission : transmissions_)
    {
      transmission->joint_to_actuator();
    }

    for (auto & actuator : actuators_)
    {
      actuator.command = actuator.transmission_passthrough;

      if (actuator.command_ptr)
      {
        *actuator.command_ptr = actuator.command;
      }
    }

    for (auto & dev : devs_)
    {
      dev->process(now);
    }

    return return_type::OK;
  }

private:
  struct JointData
  {
    explicit JointData(std::string name_in)
    : name(std::move(name_in))
    {
    }

    std::string name;
    double command{0.0};
    double state{0.0};
    double transmission_passthrough{0.0};
    std::string actuator_name;
  };

  struct ActuatorData
  {
    explicit ActuatorData(std::string name_in)
    : name(std::move(name_in))
    {
    }

    std::string name;
    double command{0.0};
    double state{0.0};
    double transmission_passthrough{0.0};
    double * state_ptr{nullptr};
    double * velocity_ptr{nullptr};
    double * effort_ptr{nullptr};
    double * command_ptr{nullptr};
    bool configured{false};
  };

  JointData & get_joint(const std::string & name)
  {
    auto it = joint_index_.find(name);
    if (it == joint_index_.end())
    {
      joints_.emplace_back(name);
      joint_index_[name] = joints_.size() - 1;
      return joints_.back();
    }

    return joints_[it->second];
  }

  ActuatorData & get_actuator(const std::string & name)
  {
    auto it = actuator_index_.find(name);
    if (it == actuator_index_.end())
    {
      actuators_.emplace_back(name);
      actuator_index_[name] = actuators_.size() - 1;
      return actuators_.back();
    }

    return actuators_[it->second];
  }

  std::shared_ptr<pluginlib::ClassLoader<CanDevice>> loader_;
  std::vector<std::shared_ptr<CanDevice>> devs_;

  std::unordered_map<std::string, size_t> joint_index_;
  std::unordered_map<std::string, size_t> actuator_index_;
  std::vector<JointData> joints_;
  std::vector<ActuatorData> actuators_;
  std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;

  std::vector<double *> pos_ptrs_;
  std::vector<double *> vel_ptrs_;
  std::vector<double *> eff_ptrs_;
  std::vector<double *> cmd_ptrs_;

  rclcpp::Node::SharedPtr node_;
};

}  // namespace mr2_can_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::CanHW, hardware_interface::SystemInterface)

