#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

      auto & joint_data = get_joint(joint.name);

      bool position_found = false;
      joint_data.has_velocity = false;
      joint_data.has_effort = false;

      for (const auto & state_interface : joint.state_interfaces)
      {
        if (state_interface.name == hardware_interface::HW_IF_POSITION)
        {
          position_found = true;
        }
        else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
        {
          joint_data.has_velocity = true;
        }
        else if (state_interface.name == hardware_interface::HW_IF_EFFORT)
        {
          joint_data.has_effort = true;
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Joint %s has unsupported state interface '%s'",
            joint.name.c_str(), state_interface.name.c_str());
          return CallbackReturn::ERROR;
        }
      }

      if (!position_found)
      {
        RCLCPP_ERROR(node_->get_logger(),
          "Joint %s must expose at least a position state interface", joint.name.c_str());
        return CallbackReturn::ERROR;
      }

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
      joint_handles.reserve(transmission_info.joints.size() * 3);

      auto make_joint_handle =
        [&](const std::string & joint_name, const std::string & interface) -> bool
      {
        auto & joint = get_joint(joint_name);
        double * passthrough = nullptr;

        if (interface == hardware_interface::HW_IF_POSITION)
        {
          passthrough = &joint.position_transmission;
        }
        else if (interface == hardware_interface::HW_IF_VELOCITY)
        {
          if (!joint.has_velocity)
          {
            RCLCPP_ERROR(node_->get_logger(),
              "Transmission requests velocity interface for joint %s, but it is not available",
              joint_name.c_str());
            return false;
          }
          passthrough = &joint.velocity_transmission;
        }
        else if (interface == hardware_interface::HW_IF_EFFORT)
        {
          if (!joint.has_effort)
          {
            RCLCPP_ERROR(node_->get_logger(),
              "Transmission requests effort interface for joint %s, but it is not available",
              joint_name.c_str());
            return false;
          }
          passthrough = &joint.effort_transmission;
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Transmission requests unsupported joint interface '%s' for %s",
            interface.c_str(), joint_name.c_str());
          return false;
        }

        joint_handles.emplace_back(joint_name, interface, passthrough);
        return true;
      };

      for (const auto & joint_info : transmission_info.joints)
      {
        if (joint_info.state_interfaces.empty())
        {
          if (!make_joint_handle(joint_info.name, hardware_interface::HW_IF_POSITION))
          {
            return CallbackReturn::ERROR;
          }
          auto & joint = get_joint(joint_info.name);
          if (joint.has_velocity &&
            !make_joint_handle(joint_info.name, hardware_interface::HW_IF_VELOCITY))
          {
            return CallbackReturn::ERROR;
          }
          if (joint.has_effort &&
            !make_joint_handle(joint_info.name, hardware_interface::HW_IF_EFFORT))
          {
            return CallbackReturn::ERROR;
          }
        }
        else
        {
          for (const auto & interface : joint_info.state_interfaces)
          {
            if (!make_joint_handle(joint_info.name, interface))
            {
              return CallbackReturn::ERROR;
            }
          }
        }
      }

      std::vector<transmission_interface::ActuatorHandle> actuator_handles;
      actuator_handles.reserve(transmission_info.actuators.size() * 3);

      auto make_actuator_handle =
        [&](const std::string & actuator_name, const std::string & interface,
          bool is_command) -> bool
      {
        auto & actuator = get_actuator(actuator_name);
        double * passthrough = nullptr;

        if (interface == hardware_interface::HW_IF_POSITION)
        {
          passthrough = &actuator.position_transmission;
        }
        else if (interface == hardware_interface::HW_IF_VELOCITY)
        {
          if (!actuator.velocity_ptr)
          {
            RCLCPP_ERROR(node_->get_logger(),
              "Transmission requests velocity %s interface for actuator %s, but device did not export it",
              is_command ? "command" : "state", actuator_name.c_str());
            return false;
          }
          passthrough = &actuator.velocity_transmission;
        }
        else if (interface == hardware_interface::HW_IF_EFFORT)
        {
          if (!actuator.effort_ptr)
          {
            RCLCPP_ERROR(node_->get_logger(),
              "Transmission requests effort %s interface for actuator %s, but device did not export it",
              is_command ? "command" : "state", actuator_name.c_str());
            return false;
          }
          passthrough = &actuator.effort_transmission;
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Transmission requests unsupported actuator interface '%s' for %s",
            interface.c_str(), actuator_name.c_str());
          return false;
        }

        actuator_handles.emplace_back(actuator_name, interface, passthrough);
        return true;
      };

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

        std::unordered_set<std::string> added_interfaces;

        auto ensure_handle =
          [&](const std::string & interface, bool is_command) -> bool
        {
          if (!added_interfaces.insert(interface).second)
          {
            return true;
          }
          return make_actuator_handle(actuator_info.name, interface, is_command);
        };

        if (actuator_info.state_interfaces.empty())
        {
          if (!ensure_handle(hardware_interface::HW_IF_POSITION, false))
          {
            return CallbackReturn::ERROR;
          }
          if (actuator.velocity_ptr &&
            !ensure_handle(hardware_interface::HW_IF_VELOCITY, false))
          {
            return CallbackReturn::ERROR;
          }
          if (actuator.effort_ptr &&
            !ensure_handle(hardware_interface::HW_IF_EFFORT, false))
          {
            return CallbackReturn::ERROR;
          }
        }
        else
        {
          for (const auto & interface : actuator_info.state_interfaces)
          {
            if (!ensure_handle(interface, false))
            {
              return CallbackReturn::ERROR;
            }
          }
        }

        for (const auto & interface : actuator_info.command_interfaces)
        {
          if (!ensure_handle(interface, true))
          {
            return CallbackReturn::ERROR;
          }
        }
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
    interfaces.reserve(info_.joints.size() * 3);

    for (const auto & joint : info_.joints)
    {
      auto it = joint_index_.find(joint.name);
      if (it == joint_index_.end())
      {
        continue;
      }

      auto & data = joints_[it->second];
      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &data.position_state);

      if (data.has_velocity)
      {
        interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &data.velocity_state);
      }

      if (data.has_effort)
      {
        interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &data.effort_state);
      }
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
        &joints_[it->second].position_command);
    }

    return interfaces;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    for (auto & actuator : actuators_)
    {
      if (actuator.state_ptr)
      {
        actuator.position_state = *actuator.state_ptr;
      }

      if (actuator.velocity_ptr)
      {
        actuator.velocity_state = *actuator.velocity_ptr;
      }

      if (actuator.effort_ptr)
      {
        actuator.effort_state = *actuator.effort_ptr;
      }

      actuator.position_transmission = actuator.position_state;
      actuator.velocity_transmission = actuator.velocity_ptr ? actuator.velocity_state : 0.0;
      actuator.effort_transmission = actuator.effort_ptr ? actuator.effort_state : 0.0;
    }

    for (auto & transmission : transmissions_)
    {
      transmission->actuator_to_joint();
    }

    for (auto & joint : joints_)
    {
      joint.position_state = joint.position_transmission;
      if (joint.has_velocity)
      {
        joint.velocity_state = joint.velocity_transmission;
      }
      if (joint.has_effort)
      {
        joint.effort_state = joint.effort_transmission;
      }
    }

    return return_type::OK;
  }

  return_type write(const rclcpp::Time & now, const rclcpp::Duration &) override
  {
    for (auto & joint : joints_)
    {
      joint.position_transmission = joint.position_command;
      joint.velocity_transmission = 0.0;
      joint.effort_transmission = 0.0;
    }

    for (auto & transmission : transmissions_)
    {
      transmission->joint_to_actuator();
    }

    for (auto & actuator : actuators_)
    {
      actuator.position_command = actuator.position_transmission;
      actuator.velocity_command = actuator.velocity_transmission;
      actuator.effort_command = actuator.effort_transmission;

      if (actuator.command_ptr)
      {
        *actuator.command_ptr = actuator.position_command;
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
    double position_command{0.0};
    double position_state{0.0};
    double velocity_state{0.0};
    double effort_state{0.0};
    double position_transmission{0.0};
    double velocity_transmission{0.0};
    double effort_transmission{0.0};
    std::string actuator_name;
    bool has_velocity{false};
    bool has_effort{false};
  };

  struct ActuatorData
  {
    explicit ActuatorData(std::string name_in)
    : name(std::move(name_in))
    {
    }

    std::string name;
    double position_command{0.0};
    double velocity_command{0.0};
    double effort_command{0.0};
    double position_state{0.0};
    double velocity_state{0.0};
    double effort_state{0.0};
    double position_transmission{0.0};
    double velocity_transmission{0.0};
    double effort_transmission{0.0};
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

