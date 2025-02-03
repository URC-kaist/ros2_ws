import time
import struct
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from mr2_jk_bms_interfaces.msg import BMSData  # Import your custom message

class BMSReader(Node):
    def __init__(self):
        super().__init__('bms_reader')

        # Declare parameters for polling interval and serial port.
        self.declare_parameter('sleep_time', 10.0)
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.sleep_time = self.get_parameter('sleep_time').value
        self.serial_port = self.get_parameter('serial_port').value

        # Attempt to open the serial port.
        try:
            self.bms = serial.Serial(self.serial_port, baudrate=115200, timeout=0.2)
            self.get_logger().info(f"BMS connected on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"BMS not found on {self.serial_port}: {e}")
            self.bms = None

        # Create a publisher for the custom BMS metrics.
        self.publisher = self.create_publisher(BMSData, 'bms_metrics', 10)

        # Create a timer to poll the BMS every sleep_time seconds.
        self.timer = self.create_timer(self.sleep_time, self.timer_callback)
        self.get_logger().info(f"BMSReader node started, polling every {self.sleep_time} seconds.")

    def send_bms_command(self, cmd_string: str):
        """
        Sends a command (provided as a hex string) to the BMS.
        """
        cmd_bytes = bytearray.fromhex(cmd_string)
        self.bms.write(cmd_bytes)

    def read_bms(self) -> BMSData:
        """
        Reads and parses data from the BMS and returns a BMSData message.
        """
        bms_data = BMSData()
        # Populate the header with the current time.
        bms_data.header = Header()
        bms_data.header.stamp = self.get_clock().now().to_msg()
        bms_data.header.frame_id = "bms_frame"
        
        try:
            # Send the command to request data from the BMS.
            self.send_bms_command('4E 57 00 13 00 00 00 00 06 03 00 00 00 00 00 00 68 00 00 01 29')
            time.sleep(0.1)  # Small delay for the BMS to respond

            if self.bms.inWaiting() >= 4:
                # Check for the header bytes (0x4E, 0x57).
                if self.bms.read(1).hex() == '4e' and self.bms.read(1).hex() == '57':
                    # Next two bytes indicate the packet length (including these two bytes).
                    length = int.from_bytes(self.bms.read(2), byteorder='big')
                    length -= 2  # Remaining bytes after the length field.

                    # Ensure the full packet is available.
                    available = self.bms.inWaiting()
                    if available != length:
                        time.sleep(0.1)
                        available = self.bms.inWaiting()
                        if available != length:
                            self.bms.reset_input_buffer()
                            raise Exception("Something went wrong reading the data...")

                    # Reconstruct the header and length field (needed for CRC calculation).
                    header = bytearray.fromhex("4e57")
                    header += (length + 2).to_bytes(2, byteorder='big')
                    data = bytearray(self.bms.read(available))
                    data = header + data

                    # Calculate the CRC sum over all bytes except the last four.
                    crc_calc = sum(data[0:-4])
                    # Extract the CRC value from the data.
                    crc_lo = struct.unpack_from('>H', data[-2:])[0]
                    if crc_calc != crc_lo:
                        self.bms.reset_input_buffer()
                        raise Exception("CRC Wrong")

                    # Extract the actual payload data.
                    data = data[11:length - 19]
                    # The second byte is the count of cell data bytes.
                    bytecount = data[1]
                    # Each cell voltage is encoded in 3 bytes.
                    cellcount = int(bytecount / 3)

                    # Process cell voltages.
                    cell_voltages = []
                    for i in range(cellcount):
                        voltage = struct.unpack_from('>xH', data, i * 3 + 2)[0] / 1000
                        cell_voltages.append(voltage)
                    bms_data.cell_voltages = cell_voltages

                    # Temperatures are encoded in the next nine bytes.
                    temp_fet = struct.unpack_from('>H', data, bytecount + 3)[0]
                    if temp_fet > 100:
                        temp_fet = -(temp_fet - 100)
                    temp_1 = struct.unpack_from('>H', data, bytecount + 6)[0]
                    if temp_1 > 100:
                        temp_1 = -(temp_1 - 100)
                    temp_2 = struct.unpack_from('>H', data, bytecount + 9)[0]
                    if temp_2 > 100:
                        temp_2 = -(temp_2 - 100)
                    bms_data.temperature_fet = float(temp_fet)
                    bms_data.temperature_probe_1 = float(temp_1)
                    bms_data.temperature_probe_2 = float(temp_2)

                    # Global battery voltage.
                    voltage_global = struct.unpack_from('>H', data, bytecount + 12)[0] / 100
                    bms_data.global_voltage = float(voltage_global)

                    # Current measurement.
                    raw_current = struct.unpack_from('>H', data, bytecount + 15)[0]
                    if raw_current & 0x8000:
                        current = -((raw_current - 0x8000) * 0.01)
                    else:
                        current = raw_current * 0.01
                    bms_data.current = float(current)
                    self.get_logger().info(f"Current: {current} A")

                    # Remaining capacity (%).
                    capacity = struct.unpack_from('>B', data, bytecount + 18)[0]
                    bms_data.capacity = capacity

            # Clear any remaining bytes in the buffer.
            self.bms.reset_input_buffer()

        except Exception as e:
            self.get_logger().error(f"BMS error: {e}")
            return None

        return bms_data

    def timer_callback(self):
        """
        Timer callback to poll the BMS, collect metrics, and publish them.
        """
        if self.bms is None:
            self.get_logger().warn("No BMS connected.")
            return

        bms_data = self.read_bms()
        if bms_data is not None:
            self.publisher.publish(bms_data)
            self.get_logger().info("Published BMS metrics: " +
                                   f"Cells: {bms_data.cell_voltages}, Temp FET: {bms_data.temperature_fet}, " +
                                   f"Temp Probe 1: {bms_data.temperature_probe_1}, Temp Probe 2: {bms_data.temperature_probe_2}, " +
                                   f"Global Voltage: {bms_data.global_voltage}, Current: {bms_data.current}, " +
                                   f"Capacity: {bms_data.capacity}")
        else:
            self.get_logger().info("No metrics available.")

def main(args=None):
    rclpy.init(args=args)
    node = BMSReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down node.")
    finally:
        if node.bms:
            node.bms.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
