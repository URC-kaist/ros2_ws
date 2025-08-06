#pragma once
#include <linux/can.h>
#include <vector>
#include <memory>
#include <rclcpp/time.hpp>  
#include <rclcpp/node.hpp>  
#include <hardware_interface/types/hardware_interface_type_values.hpp>  
#include <hardware_interface/component_parser.hpp>
#include "can_bus_registry.hpp"

class CanDevice
{
public:
  virtual ~CanDevice() = default;

  /**
   * Configure the device with parameters taken from the joint / component
   * description and the ROS node that owns it.
   */
  virtual void configure(const hardware_interface::ComponentInfo & info,
                         rclcpp::Node * node) = 0;

  /**
   * Called once per controller update cycle – typically send a CAN command.
   */
  virtual void process(const rclcpp::Time & now) = 0;

  /**
   * Export state arrays (pointers) so the ros2_control SystemInterface can
   * wire them into the joint_state interfaces.
   */
  virtual void export_state(std::vector<double*> & pos,
                            std::vector<double*> & vel,
                            std::vector<double*> & eff) = 0;

  /**
   * Export command array pointers (usually position commands).
   */
  virtual void export_command(std::vector<double*> & cmd) = 0;

protected:
  /** Shorthand for sending on the appropriate bus */
  void send(const struct can_frame & f,
            const std::shared_ptr<CanBusManager> & bus)
  { bus->enqueue_tx(f); }

  /** Register a listener for this device on its bus */
  void add_filter(const std::shared_ptr<CanBusManager> & bus,
                  uint32_t id, uint32_t mask,
                  CanBusManager::RxCallback cb)
  { bus->register_listener(id, mask, std::move(cb)); }
};
