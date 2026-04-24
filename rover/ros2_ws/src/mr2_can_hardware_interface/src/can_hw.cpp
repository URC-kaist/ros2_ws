#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "transmission_interface/four_bar_linkage_transmission_loader.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

#include "mr2_can_bus_core/can_device.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace mr2_can_hardware_interface {

class CanHW : public hardware_interface::SystemInterface {
public:
  CanHW() = default;
  ~CanHW() override = default;

  CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("can_hw");

    try {
      loader_ = std::make_shared<pluginlib::ClassLoader<CanDevice>>(
          "mr2_can_bus_core", "CanDevice");
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "Pluginlib load error: %s", ex.what());
      return CallbackReturn::ERROR;
    }

    joint_index_.clear();
    actuator_index_.clear();
    joints_.clear();
    actuators_.clear();
    transmissions_.clear();
    device_states_.clear();
    devs_.clear();

    transmission_interface::SimpleTransmissionLoader simple_loader;
    transmission_interface::FourBarLinkageTransmissionLoader four_bar_loader;

    for (const auto &joint : info_.joints) {
      auto &joint_data = get_joint(joint.name);
      joint_data.has_velocity_state = false;
      joint_data.has_effort_state = false;

      if (joint.command_interfaces.size() != 1 ||
          (joint.command_interfaces[0].name !=
               hardware_interface::HW_IF_POSITION &&
           joint.command_interfaces[0].name !=
               hardware_interface::HW_IF_VELOCITY)) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Joint %s must expose a single position or velocity "
                     "command interface",
                     joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      joint_data.command_interface = joint.command_interfaces[0].name;

      bool has_position_state = false;
      for (const auto &state_iface : joint.state_interfaces) {
        if (state_iface.name == hardware_interface::HW_IF_POSITION) {
          has_position_state = true;
        } else if (state_iface.name == hardware_interface::HW_IF_VELOCITY) {
          joint_data.has_velocity_state = true;
        } else if (state_iface.name == hardware_interface::HW_IF_EFFORT) {
          joint_data.has_effort_state = true;
        } else {
          RCLCPP_ERROR(node_->get_logger(),
                       "Joint %s exposes unsupported state interface '%s'",
                       joint.name.c_str(), state_iface.name.c_str());
          return CallbackReturn::ERROR;
        }
      }

      if (!has_position_state) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Joint %s must expose a position state interface",
                     joint.name.c_str());
        return CallbackReturn::ERROR;
      }

      const auto plugin_it = joint.parameters.find("device_plugin");
      if (plugin_it == joint.parameters.end()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Joint %s missing <param name=\"device_plugin\">",
                     joint.name.c_str());
        return CallbackReturn::ERROR;
      }

      const auto actuator_it = joint.parameters.find("actuator");
      joint_data.actuator_name = actuator_it != joint.parameters.end()
                                     ? actuator_it->second
                                     : joint.name;
      const auto lower_limit_it = joint.parameters.find("joint_lower_limit");
      if (lower_limit_it != joint.parameters.end()) {
        joint_data.lower_limit = std::stod(lower_limit_it->second);
      }
      const auto upper_limit_it = joint.parameters.find("joint_upper_limit");
      if (upper_limit_it != joint.parameters.end()) {
        joint_data.upper_limit = std::stod(upper_limit_it->second);
      }
      if (std::isfinite(joint_data.lower_limit) &&
          std::isfinite(joint_data.upper_limit) &&
          joint_data.lower_limit > joint_data.upper_limit) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Joint %s lower limit (%.6f) exceeds upper limit (%.6f)",
            joint.name.c_str(), joint_data.lower_limit, joint_data.upper_limit);
        return CallbackReturn::ERROR;
      }

      const auto origin_offset_it = joint.parameters.find("origin_offset");
      if (origin_offset_it != joint.parameters.end()) {
        joint_data.uses_boot_origin = true;
        joint_data.origin_offset = std::stod(origin_offset_it->second);
      }

      auto &actuator = get_actuator(joint_data.actuator_name);
      if (joint_data.command_interface == hardware_interface::HW_IF_VELOCITY) {
        actuator.uses_velocity_command = true;
      }

      if (!actuator.configured) {
        std::shared_ptr<CanDevice> dev;
        try {
          dev = loader_->createSharedInstance(plugin_it->second);
        } catch (const pluginlib::PluginlibException &ex) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Failed to load device plugin %s: %s",
                       plugin_it->second.c_str(), ex.what());
          return CallbackReturn::ERROR;
        }

        try {
          dev->configure(joint, node_.get());
        } catch (const std::exception &ex) {
          RCLCPP_ERROR(node_->get_logger(), "Device %s configure() threw: %s",
                       joint.name.c_str(), ex.what());
          return CallbackReturn::ERROR;
        }

        DevicePointers &device_state = device_states_[actuator.name];
        device_state = {};

        dev->export_state(device_state.state_ptr, device_state.velocity_ptr,
                          device_state.effort_ptr);
        dev->export_command(device_state.command_ptr);

        if (!device_state.state_ptr || !device_state.command_ptr) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Device for actuator %s failed to export state/command "
                       "interfaces",
                       actuator.name.c_str());
          return CallbackReturn::ERROR;
        }

        std::vector<std::pair<std::string, double *>> named_entries;

        actuator.configured = true;
        actuator.device = &device_state;
        devs_.push_back(std::move(dev));
      }
    }

    for (const auto &transmission_info : info_.transmissions) {
      std::shared_ptr<transmission_interface::Transmission> transmission;
      try {
        if (transmission_info.type ==
            "transmission_interface/SimpleTransmission") {
          transmission = simple_loader.load(transmission_info);
        } else if (transmission_info.type ==
                   "transmission_interface/FourBarLinkageTransmission") {
          transmission = four_bar_loader.load(transmission_info);
        } else {
          RCLCPP_ERROR(node_->get_logger(),
                       "Unsupported transmission type '%s'",
                       transmission_info.type.c_str());
          return CallbackReturn::ERROR;
        }
      } catch (
          const transmission_interface::TransmissionInterfaceException &ex) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Error while loading transmission '%s': %s",
                     transmission_info.name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }

      if (!transmission) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Loader returned null transmission for '%s'",
                     transmission_info.name.c_str());
        return CallbackReturn::ERROR;
      }

      std::vector<transmission_interface::JointHandle> joint_handles;
      joint_handles.reserve(transmission_info.joints.size() * 3);

      for (const auto &joint_info : transmission_info.joints) {
        auto &joint = get_joint(joint_info.name);

        bool position_added = false;
        bool velocity_added = false;
        bool effort_added = false;

        auto add_joint_handle = [&](const std::string &interface) {
          if (interface == hardware_interface::HW_IF_POSITION &&
              !position_added) {
            joint_handles.emplace_back(joint_info.name,
                                       hardware_interface::HW_IF_POSITION,
                                       &joint.transmission_passthrough);
            position_added = true;
          } else if (interface == hardware_interface::HW_IF_VELOCITY &&
                     !velocity_added) {
            joint_handles.emplace_back(joint_info.name,
                                       hardware_interface::HW_IF_VELOCITY,
                                       &joint.transmission_velocity);
            velocity_added = true;
          } else if (interface == hardware_interface::HW_IF_EFFORT &&
                     !effort_added) {
            joint_handles.emplace_back(joint_info.name,
                                       hardware_interface::HW_IF_EFFORT,
                                       &joint.transmission_effort);
            effort_added = true;
          }
        };

        add_joint_handle(hardware_interface::HW_IF_POSITION);

        if (joint.has_velocity_state) {
          add_joint_handle(hardware_interface::HW_IF_VELOCITY);
        }

        if (joint.has_effort_state) {
          add_joint_handle(hardware_interface::HW_IF_EFFORT);
        }

        for (const auto &cmd_iface : joint_info.command_interfaces) {
          add_joint_handle(cmd_iface);
        }

        for (const auto &state_iface : joint_info.state_interfaces) {
          add_joint_handle(state_iface);
        }
      }

      std::vector<transmission_interface::ActuatorHandle> actuator_handles;
      actuator_handles.reserve(transmission_info.actuators.size() * 3);

      for (const auto &actuator_info : transmission_info.actuators) {
        auto &actuator = get_actuator(actuator_info.name);

        if (!actuator.configured) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Transmission '%s' references actuator '%s' with no "
                       "configured device",
                       transmission_info.name.c_str(),
                       actuator_info.name.c_str());
          return CallbackReturn::ERROR;
        }

        bool position_added = false;
        bool velocity_added = false;
        bool effort_added = false;

        auto add_actuator_handle = [&](const std::string &interface) {
          if (interface == hardware_interface::HW_IF_POSITION &&
              !position_added) {
            actuator_handles.emplace_back(actuator_info.name,
                                          hardware_interface::HW_IF_POSITION,
                                          &actuator.transmission_passthrough);
            position_added = true;
          } else if (interface == hardware_interface::HW_IF_VELOCITY &&
                     !velocity_added) {
            actuator_handles.emplace_back(actuator_info.name,
                                          hardware_interface::HW_IF_VELOCITY,
                                          &actuator.transmission_velocity);
            velocity_added = true;
          } else if (interface == hardware_interface::HW_IF_EFFORT &&
                     !effort_added) {
            actuator_handles.emplace_back(actuator_info.name,
                                          hardware_interface::HW_IF_EFFORT,
                                          &actuator.transmission_effort);
            effort_added = true;
          }
        };

        add_actuator_handle(hardware_interface::HW_IF_POSITION);

        if (actuator.device && actuator.device->velocity_ptr) {
          add_actuator_handle(hardware_interface::HW_IF_VELOCITY);
        }

        if (actuator.device && actuator.device->effort_ptr) {
          add_actuator_handle(hardware_interface::HW_IF_EFFORT);
        }

        for (const auto &cmd_iface : actuator_info.command_interfaces) {
          add_actuator_handle(cmd_iface);
        }

        for (const auto &state_iface : actuator_info.state_interfaces) {
          add_actuator_handle(state_iface);
        }
      }

      try {
        transmission->configure(joint_handles, actuator_handles);
      } catch (
          const transmission_interface::TransmissionInterfaceException &ex) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Error while configuring transmission '%s': %s",
                     transmission_info.name.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }

      transmissions_.push_back(std::move(transmission));
    }
    if (transmissions_.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "No transmissions defined for CAN hardware interface");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "CanHW initialised: %zu joints, %zu actuators, %zu transmissions",
        joints_.size(), actuators_.size(), transmissions_.size());

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(info_.joints.size() * 3);

    for (const auto &joint : info_.joints) {
      auto it = joint_index_.find(joint.name);
      if (it == joint_index_.end()) {
        continue;
      }

      auto &joint_data = joints_[it->second];

      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                              &joint_data.state);

      if (joint_data.has_velocity_state) {
        interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                                &joint_data.velocity);
      }

      if (joint_data.has_effort_state) {
        interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT,
                                &joint_data.effort);
      }
    }

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(info_.joints.size());

    for (const auto &joint : info_.joints) {
      auto it = joint_index_.find(joint.name);
      if (it == joint_index_.end()) {
        continue;
      }

      interfaces.emplace_back(joint.name, joints_[it->second].command_interface,
                              &joints_[it->second].command);
    }

    return interfaces;
  }

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override {
    (void)previous_state;
    for (auto &joint : joints_) {
      joint.boot_reference_valid = false;
      joint.boot_reference_position =
          std::numeric_limits<double>::quiet_NaN();

      if (joint.command_interface == hardware_interface::HW_IF_POSITION) {
        joint.command = std::numeric_limits<double>::quiet_NaN();
        joint.transmission_passthrough =
            std::numeric_limits<double>::quiet_NaN();
        joint.transmission_velocity = 0.0;
      } else {
        joint.command = 0.0;
      }
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    (void)previous_state;
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    for (auto &actuator : actuators_) {
      const auto *device = actuator.device;
      actuator.state = (device && device->state_ptr)
                           ? *device->state_ptr
                           : std::numeric_limits<double>::quiet_NaN();
      actuator.velocity = (device && device->velocity_ptr)
                              ? *device->velocity_ptr
                              : std::numeric_limits<double>::quiet_NaN();
      actuator.effort = (device && device->effort_ptr)
                            ? *device->effort_ptr
                            : std::numeric_limits<double>::quiet_NaN();

      actuator.transmission_passthrough = actuator.state;
      actuator.transmission_velocity = actuator.velocity;
      actuator.transmission_effort = actuator.effort;
    }

    for (auto &transmission : transmissions_) {
      transmission->actuator_to_joint();
    }

    for (auto &joint : joints_) {
      const double actuator_position = joint.transmission_passthrough;
      if (joint.uses_boot_origin && std::isfinite(actuator_position) &&
          !joint.boot_reference_valid) {
        joint.boot_reference_position = actuator_position;
        joint.boot_reference_valid = true;
      }

      if (joint.uses_boot_origin && joint.boot_reference_valid &&
          std::isfinite(actuator_position)) {
        joint.state = actuator_position - joint.boot_reference_position +
                      joint.origin_offset;
      } else if (!std::isfinite(joint.state)) {
        joint.state = joint.uses_boot_origin ? joint.origin_offset : 0.0;
      }

      if (std::isfinite(joint.transmission_velocity)) {
        joint.velocity = joint.transmission_velocity;
      } else if (!std::isfinite(joint.velocity)) {
        joint.velocity = 0.0;
      }

      if (std::isfinite(joint.transmission_effort)) {
        joint.effort = joint.transmission_effort;
      } else if (!std::isfinite(joint.effort)) {
        joint.effort = 0.0;
      }
    }

    return return_type::OK;
  }

  return_type write(const rclcpp::Time &now,
                    const rclcpp::Duration &period) override {
    for (auto &joint : joints_) {
      if (joint.command_interface == hardware_interface::HW_IF_POSITION) {
        double limited_command = joint.command;
        if (std::isfinite(limited_command)) {
          if (std::isfinite(joint.lower_limit)) {
            limited_command = std::max(limited_command, joint.lower_limit);
          }
          if (std::isfinite(joint.upper_limit)) {
            limited_command = std::min(limited_command, joint.upper_limit);
          }
          joint.command = limited_command;
        }
        if (!std::isfinite(joint.command)) {
          joint.transmission_passthrough =
              std::numeric_limits<double>::quiet_NaN();
        } else if (joint.uses_boot_origin) {
          if (joint.boot_reference_valid) {
            joint.transmission_passthrough =
                joint.command - joint.origin_offset +
                joint.boot_reference_position;
          } else {
            joint.transmission_passthrough =
                std::numeric_limits<double>::quiet_NaN();
          }
        } else {
          joint.transmission_passthrough = joint.command;
        }
        joint.transmission_velocity = 0.0;
      } else {
        if (joint.uses_boot_origin && joint.boot_reference_valid &&
            std::isfinite(joint.state)) {
          joint.transmission_passthrough =
              joint.state - joint.origin_offset + joint.boot_reference_position;
        } else {
          joint.transmission_passthrough = joint.state;
        }
        joint.transmission_velocity = joint.command;
      }
      joint.transmission_effort = 0.0;
    }

    for (auto &transmission : transmissions_) {
      transmission->joint_to_actuator();
    }

    for (auto &actuator : actuators_) {
      if (actuator.uses_velocity_command) {
        // Velocity-mode actuators should never be driven by position fallback.
        // If no finite velocity command is available yet, hold them at zero.
        if (std::isfinite(actuator.transmission_velocity)) {
          actuator.command = actuator.transmission_velocity;
        } else {
          actuator.command = 0.0;
        }
      } else {
        actuator.command = actuator.transmission_passthrough;
      }
      if (actuator.device && actuator.device->command_ptr) {
        *actuator.device->command_ptr = actuator.command;
      }
    }

    for (auto &dev : devs_) {
      dev->process(now);
    }

    return return_type::OK;
  }

private:
  struct JointData {
    explicit JointData(std::string name_in) : name(std::move(name_in)) {}

    std::string name;
    double command{std::numeric_limits<double>::quiet_NaN()};
    double state{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double effort{std::numeric_limits<double>::quiet_NaN()};
    double lower_limit{-std::numeric_limits<double>::infinity()};
    double upper_limit{std::numeric_limits<double>::infinity()};
    double origin_offset{0.0};
    double boot_reference_position{std::numeric_limits<double>::quiet_NaN()};
    double transmission_passthrough{std::numeric_limits<double>::quiet_NaN()};
    double transmission_velocity{std::numeric_limits<double>::quiet_NaN()};
    double transmission_effort{std::numeric_limits<double>::quiet_NaN()};
    std::string command_interface{hardware_interface::HW_IF_POSITION};
    std::string actuator_name;
    bool has_velocity_state{false};
    bool has_effort_state{false};
    bool uses_boot_origin{false};
    bool boot_reference_valid{false};
  };

  struct DevicePointers {
    double *state_ptr{nullptr};
    double *velocity_ptr{nullptr};
    double *effort_ptr{nullptr};
    double *command_ptr{nullptr};
  };

  struct ActuatorData {
    explicit ActuatorData(std::string name_in) : name(std::move(name_in)) {}

    std::string name;
    double command{std::numeric_limits<double>::quiet_NaN()};
    double state{std::numeric_limits<double>::quiet_NaN()};
    double velocity{std::numeric_limits<double>::quiet_NaN()};
    double effort{std::numeric_limits<double>::quiet_NaN()};
    double transmission_passthrough{std::numeric_limits<double>::quiet_NaN()};
    double transmission_velocity{std::numeric_limits<double>::quiet_NaN()};
    double transmission_effort{std::numeric_limits<double>::quiet_NaN()};
    DevicePointers *device{nullptr};
    bool configured{false};
    bool uses_velocity_command{false};
  };

  JointData &get_joint(const std::string &name) {
    auto it = joint_index_.find(name);
    if (it == joint_index_.end()) {
      joints_.emplace_back(name);
      joint_index_[name] = joints_.size() - 1;
      return joints_.back();
    }

    return joints_[it->second];
  }

  ActuatorData &get_actuator(const std::string &name) {
    auto it = actuator_index_.find(name);
    if (it == actuator_index_.end()) {
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
  std::vector<std::shared_ptr<transmission_interface::Transmission>>
      transmissions_;
  std::unordered_map<std::string, DevicePointers> device_states_;

  rclcpp::Node::SharedPtr node_;
};

} // namespace mr2_can_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::CanHW,
                       hardware_interface::SystemInterface)
