#!/usr/bin/env python3

import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SerialPWMController(Node):
    def __init__(self):
        super().__init__('serial_pwm_controller')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to {port} at {baud}")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port: {e}")

        self.sub = self.create_subscription(
            String,
            '/pwm_command',
            self.command_callback,
            10
        )

        self.get_logger().info("Listening on /pwm_command")

    def send_pwm(self, left, right):
        if self.ser is None:
            self.get_logger().error("Serial not connected")
            return

        cmd = f"PWM,{left},{right}\n"
        try:
            self.ser.write(cmd.encode())
            self.get_logger().info(f"Sent: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def command_callback(self, msg):
        key = msg.data.strip().lower()

        if key == 'w':
            self.send_pwm(-150, 150)
        elif key == 's':
            self.send_pwm(150, -150)
        elif key == 'a':
            self.send_pwm(120, 120)
        elif key == 'd':
            self.send_pwm(-120, -120)
        elif key == 'x':
            self.send_pwm(0, 0)
        else:
            self.get_logger().warn(f"Unknown command: {key}")

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialPWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()