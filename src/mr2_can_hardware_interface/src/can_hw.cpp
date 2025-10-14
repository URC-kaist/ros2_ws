#include <algorithm>
#include <cmath>
#include <memory>
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
#include "mr2_can_hardware_interface/homing_policy.hpp"

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

    try {
      homing_loader_ =
          std::make_shared<pluginlib::ClassLoader<HomingPolicy>>(
              "mr2_can_hardware_interface",
              "mr2_can_hardware_interface::HomingPolicy");
    } catch (const pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to load homing policy plugins: %s", ex.what());
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
    named_states_.clear();
    homing_instances_.clear();
    homing_active_ = false;
    homed_ = false;
    homing_failed_ = false;
    homing_error_message_.clear();

    transmission_interface::SimpleTransmissionLoader simple_loader;
    transmission_interface::FourBarLinkageTransmissionLoader four_bar_loader;

    struct HomingConfig {
      std::string plugin;
      HomingPolicy::ParamMap params;
      std::vector<std::string> joints;
    };
    std::unordered_map<std::string, HomingConfig> homing_configs;

    for (const auto &joint : info_.joints) {
      auto &joint_data = get_joint(joint.name);
      joint_data.has_velocity_state = false;
      joint_data.has_effort_state = false;

      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name !=
              hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Joint %s must expose a single position command interface",
                     joint.name.c_str());
        return CallbackReturn::ERROR;
      }

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

      auto &actuator = get_actuator(joint_data.actuator_name);

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

        const auto pos_idx = pos_ptrs_.size();
        const auto vel_idx = vel_ptrs_.size();
        const auto eff_idx = eff_ptrs_.size();
        const auto cmd_idx = cmd_ptrs_.size();

        dev->export_state(pos_ptrs_, vel_ptrs_, eff_ptrs_);
        dev->export_command(cmd_ptrs_);

        actuator.state_ptr =
            pos_ptrs_.size() > pos_idx ? pos_ptrs_[pos_idx] : nullptr;
        actuator.velocity_ptr =
            vel_ptrs_.size() > vel_idx ? vel_ptrs_[vel_idx] : nullptr;
        actuator.effort_ptr =
            eff_ptrs_.size() > eff_idx ? eff_ptrs_[eff_idx] : nullptr;
        actuator.command_ptr =
            cmd_ptrs_.size() > cmd_idx ? cmd_ptrs_[cmd_idx] : nullptr;

        if (!actuator.state_ptr || !actuator.command_ptr) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Device for actuator %s failed to export state/command "
                       "interfaces",
                       actuator.name.c_str());
          return CallbackReturn::ERROR;
        }

        std::vector<std::pair<std::string, double *>> named_entries;
        dev->export_named_states(named_entries);
        if (register_named_states(named_entries) != CallbackReturn::SUCCESS) {
          return CallbackReturn::ERROR;
        }

        actuator.configured = true;
        devs_.push_back(std::move(dev));
      }

      const auto homing_plugin_it =
          joint.parameters.find("homing_plugin");
      if (homing_plugin_it != joint.parameters.end()) {
        const std::string group =
            joint.parameters.count("homing_group")
                ? joint.parameters.at("homing_group")
                : joint.name;

        auto &cfg = homing_configs[group];
        if (!cfg.plugin.empty() && cfg.plugin != homing_plugin_it->second) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Conflicting homing plugins declared for group '%s'",
                       group.c_str());
          return CallbackReturn::ERROR;
        }

        cfg.plugin = homing_plugin_it->second;
        cfg.joints.push_back(joint.name);

        for (const auto &param : joint.parameters) {
          if (param.first.rfind("homing_", 0) == 0 &&
              param.first != "homing_plugin" &&
              param.first != "homing_group") {
            const std::string key = param.first.substr(std::string("homing_").size());
            cfg.params[key] = param.second;
          }
        }
      }
    }

    for (const auto &gpio : info_.gpios) {
      const auto plugin_it = gpio.parameters.find("device_plugin");
      if (plugin_it == gpio.parameters.end()) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GPIO component %s missing <param name=\"device_plugin\">",
            gpio.name.c_str());
        return CallbackReturn::ERROR;
      }

      std::shared_ptr<CanDevice> dev;
      try {
        dev = loader_->createSharedInstance(plugin_it->second);
      } catch (const pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to load GPIO device plugin %s: %s",
                     plugin_it->second.c_str(), ex.what());
        return CallbackReturn::ERROR;
      }

      try {
        dev->configure(gpio, node_.get());
      } catch (const std::exception &ex) {
        RCLCPP_ERROR(node_->get_logger(),
                     "GPIO device %s configure() threw: %s", gpio.name.c_str(),
                     ex.what());
        return CallbackReturn::ERROR;
      }

      std::vector<std::pair<std::string, double *>> named_entries;
      dev->export_named_states(named_entries);
      if (register_named_states(named_entries) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
      }

      // Allow devices to export state/command buffers if they choose to.
      dev->export_state(pos_ptrs_, vel_ptrs_, eff_ptrs_);
      dev->export_command(cmd_ptrs_);

      devs_.push_back(std::move(dev));
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

        if (actuator.velocity_ptr) {
          add_actuator_handle(hardware_interface::HW_IF_VELOCITY);
        }

        if (actuator.effort_ptr) {
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

    if (!homing_configs.empty()) {
      HomingPolicy::NamedStateMap state_map;
      state_map.reserve(named_states_.size());
      for (const auto &entry : named_states_) {
        state_map.emplace(entry.first,
                          static_cast<const double *>(entry.second));
      }

      for (auto &[group, cfg] : homing_configs) {
        if (cfg.plugin.empty()) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Homing group '%s' missing plugin type", group.c_str());
          return CallbackReturn::ERROR;
        }

        std::shared_ptr<HomingPolicy> policy;
        try {
          policy = homing_loader_->createSharedInstance(cfg.plugin);
        } catch (const pluginlib::PluginlibException &ex) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Failed to load homing policy '%s': %s",
                       cfg.plugin.c_str(), ex.what());
          return CallbackReturn::ERROR;
        }

        std::vector<HomingPolicy::JointHandle> handles;
        handles.reserve(cfg.joints.size());
        for (const auto &joint_name : cfg.joints) {
          auto it = joint_index_.find(joint_name);
          if (it == joint_index_.end()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Homing group '%s' references unknown joint '%s'",
                         group.c_str(), joint_name.c_str());
            return CallbackReturn::ERROR;
          }

          auto &joint = joints_[it->second];
          handles.push_back({joint.name,
                             &joint.command,
                             &joint.state,
                             joint.has_velocity_state ? &joint.velocity : nullptr,
                             &joint.offset});
        }

        policy->configure(node_, handles, state_map, cfg.params);
        if (policy->has_error()) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Homing policy for group '%s' failed to configure: %s",
                       group.c_str(), policy->error_message().c_str());
          return CallbackReturn::ERROR;
        }

        homing_instances_.push_back({group, policy, cfg.joints});
      }

      RCLCPP_INFO(node_->get_logger(),
                  "Configured %zu homing policy plugins.",
                  homing_instances_.size());
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

      interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                              &joints_[it->second].command);
    }

    return interfaces;
  }

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override {
    (void)previous_state;
    const auto now = node_->get_clock()->now();
    homing_failed_ = false;
    homing_error_message_.clear();

    for (auto &joint : joints_) {
      joint.command_seeded = false;
    }

    if (homing_instances_.empty()) {
      homed_ = true;
      homing_active_ = false;
      return CallbackReturn::SUCCESS;
    }

    homed_ = false;
    homing_active_ = true;

    for (auto &instance : homing_instances_) {
      if (!instance.policy) {
        continue;
      }
      instance.policy->reset();
      instance.policy->begin(now);
    }

    RCLCPP_INFO(node_->get_logger(),
                "Starting homing sequence (%zu policies)", homing_instances_.size());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    (void)previous_state;
    homing_active_ = false;
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    for (auto &actuator : actuators_) {
      if (actuator.state_ptr) {
        actuator.state = *actuator.state_ptr;
      } else {
        actuator.state = 0.0;
      }

      if (actuator.velocity_ptr) {
        actuator.velocity = *actuator.velocity_ptr;
      } else {
        actuator.velocity = 0.0;
      }

      if (actuator.effort_ptr) {
        actuator.effort = *actuator.effort_ptr;
      } else {
        actuator.effort = 0.0;
      }

      actuator.transmission_passthrough = actuator.state;
      actuator.transmission_velocity = actuator.velocity;
      actuator.transmission_effort = actuator.effort;
    }

    for (auto &transmission : transmissions_) {
      transmission->actuator_to_joint();
    }

    for (auto &joint : joints_) {
      joint.state = joint.transmission_passthrough - joint.offset;
      joint.velocity = joint.transmission_velocity;
      joint.effort = joint.transmission_effort;

      if (!joint.command_seeded && std::isfinite(joint.state)) {
        joint.command = joint.state;
        joint.command_seeded = true;
      }
    }

    return return_type::OK;
  }

  return_type
  write(const rclcpp::Time &now, const rclcpp::Duration &period) override {
    if (homing_failed_) {
      return return_type::ERROR;
    }

    if (homing_active_) {
      for (auto &joint : joints_) {
        joint.command = joint.state;
      }

      for (auto &instance : homing_instances_) {
        if (instance.policy) {
          instance.policy->update(now, period);
        }
      }

      bool all_finished = true;
      for (auto &instance : homing_instances_) {
        if (!instance.policy) {
          continue;
        }

        if (instance.policy->has_error()) {
          homing_failed_ = true;
          homing_error_message_ = instance.policy->error_message();
          RCLCPP_ERROR(
              node_->get_logger(),
              "Homing policy '%s' reported error: %s", instance.group.c_str(),
              homing_error_message_.c_str());
          homing_active_ = false;
          return return_type::ERROR;
        }

        if (!instance.policy->is_finished()) {
          all_finished = false;
        }
      }

      if (all_finished) {
        for (auto &instance : homing_instances_) {
          if (instance.policy) {
            instance.policy->finalize(now);
          }
        }
        homing_active_ = false;
        homed_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Homing sequence completed successfully.");
      }
    } else if (!homed_) {
      for (auto &joint : joints_) {
        joint.command = joint.state;
      }
    }

    for (auto &joint : joints_) {
      joint.transmission_passthrough = joint.command + joint.offset;
      joint.transmission_velocity = 0.0;
      joint.transmission_effort = 0.0;
    }

    for (auto &transmission : transmissions_) {
      transmission->joint_to_actuator();
    }

    for (auto &actuator : actuators_) {
      actuator.command = actuator.transmission_passthrough;

      if (actuator.command_ptr) {
        *actuator.command_ptr = actuator.command;
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
    double command{0.0};
    double state{0.0};
    double velocity{0.0};
    double effort{0.0};
    double offset{0.0};
    double transmission_passthrough{0.0};
    double transmission_velocity{0.0};
    double transmission_effort{0.0};
    std::string actuator_name;
    bool has_velocity_state{false};
    bool has_effort_state{false};
    bool command_seeded{false};
  };

  struct ActuatorData {
    explicit ActuatorData(std::string name_in) : name(std::move(name_in)) {}

    std::string name;
    double command{0.0};
    double state{0.0};
    double velocity{0.0};
    double effort{0.0};
    double transmission_passthrough{0.0};
    double transmission_velocity{0.0};
    double transmission_effort{0.0};
    double *state_ptr{nullptr};
    double *velocity_ptr{nullptr};
    double *effort_ptr{nullptr};
    double *command_ptr{nullptr};
    bool configured{false};
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

  CallbackReturn register_named_states(
      const std::vector<std::pair<std::string, double *>> &states) {
    for (const auto &entry : states) {
      if (!entry.second) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Named state '%s' has null pointer.",
                     entry.first.c_str());
        return CallbackReturn::ERROR;
      }

      auto [it, inserted] =
          named_states_.emplace(entry.first, entry.second);

      if (!inserted) {
        RCLCPP_WARN(node_->get_logger(),
                    "Named state '%s' already registered. Overwriting pointer.",
                    entry.first.c_str());
        it->second = entry.second;
      }
    }
    return CallbackReturn::SUCCESS;
  }

  struct HomingInstance {
    std::string group;
    std::shared_ptr<HomingPolicy> policy;
    std::vector<std::string> joint_names;
  };

  std::shared_ptr<pluginlib::ClassLoader<CanDevice>> loader_;
  std::shared_ptr<pluginlib::ClassLoader<HomingPolicy>> homing_loader_;
  std::vector<std::shared_ptr<CanDevice>> devs_;

  std::unordered_map<std::string, size_t> joint_index_;
  std::unordered_map<std::string, size_t> actuator_index_;
  std::vector<JointData> joints_;
  std::vector<ActuatorData> actuators_;
  std::vector<std::shared_ptr<transmission_interface::Transmission>>
      transmissions_;

  std::vector<double *> pos_ptrs_;
  std::vector<double *> vel_ptrs_;
  std::vector<double *> eff_ptrs_;
  std::vector<double *> cmd_ptrs_;

  std::unordered_map<std::string, double *> named_states_;
  std::vector<HomingInstance> homing_instances_;
  bool homing_active_{false};
  bool homed_{false};
  bool homing_failed_{false};
  std::string homing_error_message_;

  rclcpp::Node::SharedPtr node_;
};

} // namespace mr2_can_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mr2_can_hardware_interface::CanHW,
                       hardware_interface::SystemInterface)
