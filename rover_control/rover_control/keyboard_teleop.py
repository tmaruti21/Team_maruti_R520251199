#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MSG = """
Keyboard Teleop
---------------
w : forward
s : backward
a : left
d : right
x : stop
q : quit
"""


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(String, '/pwm_command', 10)
        self.get_logger().info('Keyboard teleop started')
        print(MSG)

    def publish_key(self, key):
        msg = String()
        msg.data = key
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {key}')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        while rclpy.ok():
            key = get_key()

            if key in ['w', 'a', 's', 'd', 'x']:
                node.publish_key(key)
            elif key == 'q':
                print("Quitting keyboard teleop")
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()