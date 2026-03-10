#!/usr/bin/env python3
"""
PWM Motor Driver Node
=====================
Bridges ROS2 /cmd_vel (geometry_msgs/Twist) to real DC motors via a
dual H-bridge (e.g. L298N / L293D) connected to a Raspberry Pi.

Wiring diagram (default BCM pin numbering, change via ROS2 parameters):
┌─────────────────────────────────────────────────────────────────┐
│  Raspberry Pi  ↔  L298N / L293D                                 │
│                                                                  │
│  LEFT  motor:   ENA → GPIO 12 (hardware PWM)                    │
│                 IN1 → GPIO 16  (direction)                      │
│                 IN2 → GPIO 18  (direction)                      │
│                                                                  │
│  RIGHT motor:   ENB → GPIO 13 (hardware PWM)                    │
│                 IN3 → GPIO 20  (direction)                      │
│                 IN4 → GPIO 21  (direction)                      │
└─────────────────────────────────────────────────────────────────┘

Differential-drive kinematics:
  v_left  = linear_x  −  angular_z × (wheel_base / 2)
  v_right = linear_x  +  angular_z × (wheel_base / 2)

Both values are clamped to [−max_speed, +max_speed], then mapped to:
  PWM duty cycle  = |v| / max_speed × 100  (0–100 %)
  direction pins  = HIGH/LOW based on sign(v)

Usage
-----
  # Run as a standalone ROS2 node:
  ros2 run lane_detection motor_driver_node

  # Override pin assignments via parameters:
  ros2 run lane_detection motor_driver_node \
      --ros-args \
      -p pin_left_en:=12  -p pin_left_in1:=16 -p pin_left_in2:=18 \
      -p pin_right_en:=13 -p pin_right_in3:=20 -p pin_right_in4:=21 \
      -p wheel_base_m:=0.18 -p max_linear_speed:=0.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── GPIO import with graceful fallback ───────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class MotorDriverNode(Node):
    """Converts /cmd_vel Twist messages into left/right motor PWM signals."""

    def __init__(self):
        super().__init__('motor_driver')

        # ── ROS2 parameters (override via --ros-args -p <name>:=<value>) ─────
        self.declare_parameter('pin_left_en',        12)   # PWM pin  – left  motor enable
        self.declare_parameter('pin_left_in1',       16)   # DIR pin  – left  motor IN1
        self.declare_parameter('pin_left_in2',       18)   # DIR pin  – left  motor IN2
        self.declare_parameter('pin_right_en',       13)   # PWM pin  – right motor enable
        self.declare_parameter('pin_right_in3',      20)   # DIR pin  – right motor IN3
        self.declare_parameter('pin_right_in4',      21)   # DIR pin  – right motor IN4
        self.declare_parameter('pwm_frequency_hz',  100)   # PWM carrier frequency
        self.declare_parameter('wheel_base_m',      0.18)  # track width in metres
        self.declare_parameter('max_linear_speed',  0.5)   # m/s → 100 % duty cycle

        p = self.get_parameter
        self._pin_l_en   = p('pin_left_en').value
        self._pin_l_in1  = p('pin_left_in1').value
        self._pin_l_in2  = p('pin_left_in2').value
        self._pin_r_en   = p('pin_right_en').value
        self._pin_r_in3  = p('pin_right_in3').value
        self._pin_r_in4  = p('pin_right_in4').value
        self._pwm_freq   = p('pwm_frequency_hz').value
        self._wheel_base = p('wheel_base_m').value
        self._max_speed  = p('max_linear_speed').value

        # ── GPIO / PWM setup ─────────────────────────────────────────────────
        self._pwm_left  = None
        self._pwm_right = None

        if _GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            all_pins = [self._pin_l_en, self._pin_l_in1, self._pin_l_in2,
                        self._pin_r_en, self._pin_r_in3, self._pin_r_in4]
            GPIO.setup(all_pins, GPIO.OUT, initial=GPIO.LOW)

            self._pwm_left  = GPIO.PWM(self._pin_l_en, self._pwm_freq)
            self._pwm_right = GPIO.PWM(self._pin_r_en, self._pwm_freq)
            self._pwm_left.start(0)
            self._pwm_right.start(0)

            self.get_logger().info('GPIO initialised – running in hardware mode')
        else:
            self.get_logger().warn(
                'RPi.GPIO not found – running in SIMULATION mode '
                '(motor commands will be logged but no GPIO output)')

        # ── /cmd_vel subscriber ───────────────────────────────────────────────
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.get_logger().info('=' * 55)
        self.get_logger().info('Motor Driver Node started')
        self.get_logger().info(f'  Left  motor : EN={self._pin_l_en}  IN1={self._pin_l_in1}  IN2={self._pin_l_in2}')
        self.get_logger().info(f'  Right motor : EN={self._pin_r_en}  IN3={self._pin_r_in3}  IN4={self._pin_r_in4}')
        self.get_logger().info(f'  PWM freq    : {self._pwm_freq} Hz')
        self.get_logger().info(f'  Wheel base  : {self._wheel_base} m')
        self.get_logger().info(f'  Max speed   : {self._max_speed} m/s')
        self.get_logger().info('Subscribing to /cmd_vel')
        self.get_logger().info('=' * 55)

    # ── Callback ─────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        """Convert Twist → differential wheel speeds → PWM."""
        v   = msg.linear.x    # m/s  (forward positive)
        omega = msg.angular.z # rad/s (CCW positive → turns left)

        # Differential-drive inverse kinematics
        v_left  = v - omega * (self._wheel_base / 2.0)
        v_right = v + omega * (self._wheel_base / 2.0)

        # Clamp to [-max_speed, +max_speed]
        v_left  = max(-self._max_speed, min(self._max_speed, v_left))
        v_right = max(-self._max_speed, min(self._max_speed, v_right))

        # Convert to duty cycle [0, 100] and direction booleans
        duty_left  = abs(v_left)  / self._max_speed * 100.0
        duty_right = abs(v_right) / self._max_speed * 100.0
        fwd_left   = v_left  >= 0
        fwd_right  = v_right >= 0

        self._drive(duty_left, fwd_left, duty_right, fwd_right)

        self.get_logger().debug(
            f'v={v:+.3f}  ω={omega:+.3f}  '
            f'L={v_left:+.3f}({duty_left:.1f}%{"F" if fwd_left else "R"})  '
            f'R={v_right:+.3f}({duty_right:.1f}%{"F" if fwd_right else "R"})')

    # ── Motor control helpers ─────────────────────────────────────────────────

    def _drive(self, duty_left: float, fwd_left: bool,
               duty_right: float, fwd_right: bool):
        """Set motor directions and PWM duty cycles."""
        if _GPIO_AVAILABLE and self._pwm_left and self._pwm_right:
            # Left motor direction
            GPIO.output(self._pin_l_in1, GPIO.HIGH if fwd_left  else GPIO.LOW)
            GPIO.output(self._pin_l_in2, GPIO.LOW  if fwd_left  else GPIO.HIGH)
            # Right motor direction
            GPIO.output(self._pin_r_in3, GPIO.HIGH if fwd_right else GPIO.LOW)
            GPIO.output(self._pin_r_in4, GPIO.LOW  if fwd_right else GPIO.HIGH)
            # Speed
            self._pwm_left.ChangeDutyCycle(duty_left)
            self._pwm_right.ChangeDutyCycle(duty_right)
        else:
            # Simulation: just log
            l_dir = 'FWD' if fwd_left  else 'REV'
            r_dir = 'FWD' if fwd_right else 'REV'
            self.get_logger().info(
                f'[SIM] LEFT {l_dir} {duty_left:5.1f}%  |  RIGHT {r_dir} {duty_right:5.1f}%')

    def stop_motors(self):
        """Immediately halt both motors (zero duty cycle, neutral direction)."""
        if _GPIO_AVAILABLE and self._pwm_left and self._pwm_right:
            self._pwm_left.ChangeDutyCycle(0)
            self._pwm_right.ChangeDutyCycle(0)
            GPIO.output(self._pin_l_in1, GPIO.LOW)
            GPIO.output(self._pin_l_in2, GPIO.LOW)
            GPIO.output(self._pin_r_in3, GPIO.LOW)
            GPIO.output(self._pin_r_in4, GPIO.LOW)
        else:
            self.get_logger().info('[SIM] Motors stopped')

    def cleanup(self):
        """Stop motors and release GPIO resources."""
        self.stop_motors()
        if _GPIO_AVAILABLE:
            if self._pwm_left:
                self._pwm_left.stop()
            if self._pwm_right:
                self._pwm_right.stop()
            GPIO.cleanup()
        self.get_logger().info('Motor driver cleanup complete')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
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
