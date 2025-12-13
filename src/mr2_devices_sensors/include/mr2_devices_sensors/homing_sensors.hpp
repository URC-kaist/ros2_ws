#pragma once

#include "mr2_can_bus_core/can_device.hpp"

namespace mr2_devices_sensors {

class LimitSwitchDriver : public CanDevice {
public:
  virtual void get_state(const double *&state, const double *&watchdog) = 0;
};

class AbsoluteEncoderDriver : public CanDevice {
public:
  virtual void get_state(const double *&angle, const double *&watchdog) = 0;
};

} // namespace mr2_devices_sensors
