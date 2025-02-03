# BMSData.msg
# Standard header (you can use it for timestamping and frame IDs)
std_msgs/Header header

# Array of cell voltages (in volts)
float32[] cell_voltages

# Temperatures (in °C)
float32 temperature_fet
float32 temperature_probe_1
float32 temperature_probe_2

# Global battery voltage (in volts)
float32 global_voltage

# Battery current (in Amperes)
float32 current

# Battery capacity (in %)
uint8 capacity
