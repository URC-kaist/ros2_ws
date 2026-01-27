std_msgs/Header header

# State of charge in percent (0–100)
float32 state_of_charge_pct

# Pack "health" percentage derived by firmware (0–100)
float32 health_pct

# Pack temperature in degrees Celsius
float32 temperature_c

# Pack voltage in volts
float32 pack_voltage_v

# Lifetime charge cycle counter reported by the pack
uint16 pack_life_cycles

# Firmware cycle counter that increments once per CAN burst
uint32 firmware_cycle_count

# Nominal single-cell capacity in milliamp-hours
uint16 nominal_cell_capacity_mah

# Reported number of cells in parallel per series string
uint8 parallel_group_count

# Reported series cell count
uint8 cell_count

# Per-cell voltages in millivolts, sized to cell_count
float32[] cell_voltage_mv

# Per-cell validity flags, true when the voltage reading is valid
bool[] cell_voltage_valid
