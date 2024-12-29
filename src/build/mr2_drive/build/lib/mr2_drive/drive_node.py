#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import serial
import serial.tools.list_ports


class DriveControllerNode(Node):
    def __init__(self):
        super().__init__('drive_controller_node')
        self.get_logger().info('Initializing DriveControllerNode...')

        # Discover the serial port for the drive controller
        port_name = self.find_drive_controller_port(
            target_vid=1155,  # Replace with your actual VID (decimal)
            target_pid=14158    # Replace with your actual PID (decimal)
        )

        if port_name:
            self.get_logger().info(f'Found drive controller on port: {port_name}')
        else:
            self.get_logger().error('Could not find drive controller port. Exiting...')
            raise RuntimeError('Drive controller not found.')

        # Initialize the serial port
        self.baud_rate = 115200
        self.timeout_s = 1.0
        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=self.baud_rate,
                timeout=self.timeout_s
            )
            self.get_logger().info(f'Successfully opened {port_name} at {self.baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port_name}: {e}')
            raise e

        # Subscribe to the /drive_cmd topic
        self.subscription = self.create_subscription(
            Twist,
            '/drive_cmd',
            self.drive_cmd_callback,
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning

    def find_drive_controller_port(self, target_vid=None, target_pid=None):
        """Scan available serial ports and return the matching device."""
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if target_vid is not None and target_pid is not None:
                if p.vid == target_vid and p.pid == target_pid:
                    return p.device
        return None

    def drive_cmd_callback(self, msg):
        """
        Callback for /drive_cmd topic.
        Maps Twist.linear.x to throttle and Twist.angular.z to steering.
        """
        throttle = msg.linear.x  # Assume -1.0 to 1.0 range for throttle
        steering = msg.angular.z  # Assume -1.0 to 1.0 range for steering

        self.get_logger().info(f'Received drive command: Throttle={throttle}, Steering={steering}')

        # Map commands to your controller's protocol
        # Example: `THROTTLE:x\nSTEER:y\n` where x and y are normalized
        throttle_command = f"THROTTLE:{throttle}\n"
        steering_command = f"STEER:{steering}\n"

        # Send commands over serial
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            try:
                self.serial_port.write(throttle_command.encode('utf-8'))
                self.serial_port.write(steering_command.encode('utf-8'))
                self.get_logger().info(f'Sent: {throttle_command.strip()} and {steering_command.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to write to serial: {e}')
        else:
            self.get_logger().error('Serial port is not open. Cannot send commands.')

    def destroy_node(self):
        # Close serial port on shutdown
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

