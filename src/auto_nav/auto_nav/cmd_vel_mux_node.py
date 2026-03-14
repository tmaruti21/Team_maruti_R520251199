#!/usr/bin/env python3
"""Select between autonomous and manual cmd_vel sources."""

from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelMuxNode(Node):
    """Publish one selected command from auto/manual cmd_vel streams."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_mux')

        self.declare_parameter('auto_topic', '/cmd_vel_auto')
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('output_topic', '/cmd_vel_out')
        self.declare_parameter('manual_priority', True)
        self.declare_parameter('manual_timeout_s', 0.50)
        self.declare_parameter('auto_timeout_s', 0.70)
        self.declare_parameter('publish_rate_hz', 30.0)

        p = self.get_parameter
        self._auto_topic = str(p('auto_topic').value)
        self._manual_topic = str(p('manual_topic').value)
        self._output_topic = str(p('output_topic').value)
        self._manual_priority = bool(p('manual_priority').value)
        self._manual_timeout_s = float(p('manual_timeout_s').value)
        self._auto_timeout_s = float(p('auto_timeout_s').value)
        self._publish_rate_hz = float(p('publish_rate_hz').value)

        if self._manual_timeout_s < 0.0 or self._auto_timeout_s < 0.0:
            raise RuntimeError('manual_timeout_s and auto_timeout_s must be >= 0')
        if self._publish_rate_hz <= 0.0:
            raise RuntimeError('publish_rate_hz must be > 0')

        self._manual_cmd: Tuple[float, float] = (0.0, 0.0)
        self._auto_cmd: Tuple[float, float] = (0.0, 0.0)
        self._manual_stamp = None
        self._auto_stamp = None

        self._last_source = None
        self._last_move_state = None

        self.create_subscription(Twist, self._auto_topic, self._auto_callback, 10)
        self.create_subscription(Twist, self._manual_topic, self._manual_callback, 10)
        self._publisher = self.create_publisher(Twist, self._output_topic, 10)
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)

        self.get_logger().info('cmd_vel mux started')
        self.get_logger().info(f'  auto topic   : {self._auto_topic}')
        self.get_logger().info(f'  manual topic : {self._manual_topic}')
        self.get_logger().info(f'  output topic : {self._output_topic}')
        self.get_logger().info(
            f'  manual priority={self._manual_priority} '
            f'manual_timeout={self._manual_timeout_s:.2f}s '
            f'auto_timeout={self._auto_timeout_s:.2f}s'
        )

    def _manual_callback(self, msg: Twist) -> None:
        self._manual_cmd = (float(msg.linear.x), float(msg.angular.z))
        self._manual_stamp = self.get_clock().now()

    def _auto_callback(self, msg: Twist) -> None:
        self._auto_cmd = (float(msg.linear.x), float(msg.angular.z))
        self._auto_stamp = self.get_clock().now()

    def _is_fresh(self, stamp, timeout_s: float) -> bool:
        if stamp is None:
            return False
        age_s = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age_s <= timeout_s

    def _select_command(self) -> Tuple[str, Tuple[float, float]]:
        manual_fresh = self._is_fresh(self._manual_stamp, self._manual_timeout_s)
        auto_fresh = self._is_fresh(self._auto_stamp, self._auto_timeout_s)

        if self._manual_priority and manual_fresh:
            return 'MANUAL', self._manual_cmd
        if auto_fresh:
            return 'AUTO', self._auto_cmd
        if manual_fresh:
            return 'MANUAL', self._manual_cmd
        return 'STOP', (0.0, 0.0)

    def _on_timer(self) -> None:
        source, (linear_x, angular_z) = self._select_command()

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._publisher.publish(msg)

        moving = abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4
        if source != self._last_source or moving != self._last_move_state:
            state = 'MOVE' if moving else 'STOP'
            self.get_logger().info(
                f'MUX source={source} state={state} cmd=(v={linear_x:.3f}, w={angular_z:.3f})'
            )
            self._last_source = source
            self._last_move_state = moving


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
