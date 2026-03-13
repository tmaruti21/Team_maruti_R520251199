#!/usr/bin/env python3
"""
esp32_bridge.py  –  /cmd_vel to ESP32 serial motor bridge
=========================================================
Subscribes to geometry_msgs/Twist on /cmd_vel, converts the command into
left/right differential wheel PWM values, and sends them to an ESP32 over a
serial link.

Serial protocol
---------------
Each line written to the ESP32 is ASCII:

  CMD,<left_pwm>,<left_dir>,<right_pwm>,<right_dir>\n

where:
  left_pwm / right_pwm ∈ [0, 255]
  left_dir / right_dir ∈ {0, 1}   (1 = forward, 0 = reverse)

Usage
-----
  ros2 run custom_follow esp32_bridge --ros-args \
      -p serial_port:=/dev/ttyUSB0 \
      -p wheel_base_m:=0.24 \
      -p max_linear_speed:=0.35
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

try:
    import serial
    from serial import SerialException
    _SERIAL_IMPORT_ERROR = None
except ImportError as exc:
    serial = None
    SerialException = Exception
    _SERIAL_IMPORT_ERROR = exc


class Esp32BridgeNode(Node):
    """Bridge /cmd_vel to the ESP32 motor controller over serial."""

    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM2')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base_m', 0.24)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('min_pwm', 0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('command_timeout_s', 0.6)
        self.declare_parameter('tx_rate_hz', 20.0)
        self.declare_parameter('startup_delay_s', 2.0)

        p = self.get_parameter
        self._serial_port = p('serial_port').value
        self._baud_rate = int(p('baud_rate').value)
        self._wheel_base = float(p('wheel_base_m').value)
        self._max_speed = float(p('max_linear_speed').value)
        self._max_pwm = int(p('max_pwm').value)
        self._min_pwm = int(p('min_pwm').value)
        self._invert_left = bool(p('invert_left').value)
        self._invert_right = bool(p('invert_right').value)
        self._command_timeout = float(p('command_timeout_s').value)
        self._tx_rate_hz = float(p('tx_rate_hz').value)
        self._startup_delay = float(p('startup_delay_s').value)

        if self._max_speed <= 0.0:
            raise RuntimeError('max_linear_speed must be greater than zero')
        if self._max_pwm <= 0:
            raise RuntimeError('max_pwm must be greater than zero')
        if self._tx_rate_hz <= 0.0:
            raise RuntimeError('tx_rate_hz must be greater than zero')

        self._serial = None
        self._serial_lock = threading.Lock()
        self._desired_linear = 0.0
        self._desired_angular = 0.0
        self._last_cmd_time = None
        self._last_sent_tuple = None

        self._open_serial()

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self._timer = self.create_timer(1.0 / self._tx_rate_hz, self._flush_motor_command)

        self.get_logger().info('ESP32 bridge started')
        self.get_logger().info(f'  Serial port : {self._serial_port} @ {self._baud_rate} baud')
        self.get_logger().info(f'  Wheel base  : {self._wheel_base:.3f} m')
        self.get_logger().info(f'  Max speed   : {self._max_speed:.3f} m/s -> {self._max_pwm} PWM')
        self.get_logger().info('Subscribing to /cmd_vel')

    def _open_serial(self):
        if serial is None:
            raise RuntimeError(f'pyserial is not installed: {_SERIAL_IMPORT_ERROR}')

        try:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=0.1,
                write_timeout=0.1,
            )
        except SerialException as exc:
            raise RuntimeError(f'Unable to open serial port {self._serial_port}: {exc}') from exc

        if self._startup_delay > 0.0:
            time.sleep(self._startup_delay)

        try:
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except SerialException:
            pass

        self._write_line('CMD,0,1,0,1\n')

    def _cmd_vel_callback(self, msg: Twist):
        self._desired_linear = float(msg.linear.x)
        self._desired_angular = float(msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    def _flush_motor_command(self):
        if self._last_cmd_time is None:
            linear = 0.0
            angular = 0.0
        else:
            age_s = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
            if age_s > self._command_timeout:
                linear = 0.0
                angular = 0.0
            else:
                linear = self._desired_linear
                angular = self._desired_angular

        line, motor_tuple = self._format_command(linear, angular)
        self._write_line(line)

        if motor_tuple != self._last_sent_tuple:
            left_pwm, left_dir, right_pwm, right_dir = motor_tuple
            self.get_logger().info(
                f'TX L={left_pwm:03d} {"F" if left_dir else "R"}  '
                f'R={right_pwm:03d} {"F" if right_dir else "R"}')
            self._last_sent_tuple = motor_tuple

    def _format_command(self, linear, angular):
        if not math.isfinite(linear):
            linear = 0.0
        if not math.isfinite(angular):
            angular = 0.0

        left_speed = linear - angular * (self._wheel_base / 2.0)
        right_speed = linear + angular * (self._wheel_base / 2.0)

        left_speed = max(-self._max_speed, min(self._max_speed, left_speed))
        right_speed = max(-self._max_speed, min(self._max_speed, right_speed))

        left_pwm, left_dir = self._speed_to_pwm(left_speed, self._invert_left)
        right_pwm, right_dir = self._speed_to_pwm(right_speed, self._invert_right)

        motor_tuple = (left_pwm, left_dir, right_pwm, right_dir)
        return f'CMD,{left_pwm},{left_dir},{right_pwm},{right_dir}\n', motor_tuple

    def _speed_to_pwm(self, wheel_speed, invert_direction):
        pwm = int(round(abs(wheel_speed) / self._max_speed * self._max_pwm))
        pwm = max(0, min(self._max_pwm, pwm))

        if 0 < pwm < self._min_pwm:
            pwm = self._min_pwm

        forward = wheel_speed >= 0.0
        if invert_direction and pwm > 0:
            forward = not forward

        return pwm, 1 if forward else 0

    def _write_line(self, line):
        if not self._serial:
            return

        try:
            with self._serial_lock:
                self._serial.write(line.encode('ascii'))
                self._serial.flush()
        except SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            try:
                self._serial.close()
            except SerialException:
                pass
            self._serial = None

    def cleanup(self):
        self._write_line('CMD,0,1,0,1\n')
        if self._serial:
            try:
                self._serial.close()
            except SerialException:
                pass
            self._serial = None


def main(args=None):
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()