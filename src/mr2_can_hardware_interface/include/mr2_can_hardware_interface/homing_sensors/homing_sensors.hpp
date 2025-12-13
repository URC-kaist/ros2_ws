#pragma once

#include "mr2_can_bus_core/can_device.hpp"

namespace mr2_can_hardware_interface::sensors {

class LimitSwitchDriver : public CanDevice {
public:
  virtual void get_state(const double *&state, const double *&watchdog) = 0;
  virtual const double *state_ptr() const = 0;
  virtual const double *watchdog_ptr() const = 0;
};

class AbsoluteEncoderDriver : public CanDevice {
public:
  virtual void get_state(const double *&angle, const double *&watchdog) = 0;
  virtual const double *angle_ptr() const = 0;
  virtual const double *watchdog_ptr() const = 0;
};

} // namespace mr2_can_hardware_interface::sensors
