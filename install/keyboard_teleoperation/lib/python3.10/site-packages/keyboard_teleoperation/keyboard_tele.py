#!/usr/bin/env python3
"""Keyboard teleoperation node for differential-drive rover control."""

from dataclasses import dataclass
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass(frozen=True)
class MotionCommand:
    linear: float
    angular: float


class KeyboardTeleopNode(Node):
    """Publish /cmd_vel from keyboard keys: w, a, s, d."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 1.20)
        self.declare_parameter('key_timeout_s', 0.35)
        self.declare_parameter('publish_rate_hz', 20.0)

        p = self.get_parameter
        self._cmd_topic = p('cmd_topic').value
        self._linear_speed = float(p('linear_speed').value)
        self._angular_speed = float(p('angular_speed').value)
        self._key_timeout_s = float(p('key_timeout_s').value)
        self._publish_rate_hz = float(p('publish_rate_hz').value)

        if self._publish_rate_hz <= 0.0:
            raise RuntimeError('publish_rate_hz must be > 0')
        if self._key_timeout_s < 0.0:
            raise RuntimeError('key_timeout_s must be >= 0')

        self._publisher = self.create_publisher(Twist, self._cmd_topic, 10)
        self._stop_command = MotionCommand(0.0, 0.0)
        self._active_command = self._stop_command
        self._last_key_stamp = self.get_clock().now()

        self._stdin_fd = None
        self._saved_termios = None
        self._keyboard_enabled = self._setup_keyboard_input()

        self._key_to_command = {
            'w': MotionCommand(self._linear_speed, 0.0),
            's': MotionCommand(-self._linear_speed, 0.0),
            'a': MotionCommand(0.0, self._angular_speed),
            'd': MotionCommand(0.0, -self._angular_speed),
            'x': self._stop_command,
            ' ': self._stop_command,
        }

        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)

        self.get_logger().info('Keyboard teleop started')
        self.get_logger().info(f'Publishing {self._cmd_topic} at {self._publish_rate_hz:.1f} Hz')
        self.get_logger().info('Controls: w=forward, s=backward, a=left, d=right, x/space=stop, Ctrl+C=exit')

        if not self._keyboard_enabled:
            self.get_logger().warn('Keyboard disabled because stdin is not a TTY. Run with "ros2 run" in a terminal.')

    def _setup_keyboard_input(self) -> bool:
        if not hasattr(sys.stdin, 'isatty') or not sys.stdin.isatty():
            return False

        self._stdin_fd = sys.stdin.fileno()
        self._saved_termios = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        return True

    def _read_key(self):
        if not self._keyboard_enabled:
            return None

        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None

        key = sys.stdin.read(1)
        if not key:
            return None

        return key.lower()

    def _on_timer(self) -> None:
        key = self._read_key()
        if key is not None:
            if key == '\x03':
                rclpy.shutdown()
                return

            command = self._key_to_command.get(key)
            if command is not None:
                self._active_command = command
                self._last_key_stamp = self.get_clock().now()

        age_s = (self.get_clock().now() - self._last_key_stamp).nanoseconds / 1e9
        command = self._active_command if age_s <= self._key_timeout_s else self._stop_command
        self._publish_command(command)

    def _publish_command(self, command: MotionCommand) -> None:
        msg = Twist()
        msg.linear.x = float(command.linear)
        msg.angular.z = float(command.angular)
        self._publisher.publish(msg)

    def cleanup(self) -> None:
        self._publish_command(self._stop_command)
        if self._saved_termios is not None and self._stdin_fd is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._saved_termios)
            self._saved_termios = None
            self._stdin_fd = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
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
