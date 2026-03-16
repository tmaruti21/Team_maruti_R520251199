import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, Temperature
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import serial


@dataclass
class ImuSample:
    stamp_ms: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: floatls 
    gz: float
    temp_c: float


@dataclass
class EncSample:
    stamp_ms: int
    left_ticks: int
    right_ticks: int


def _parse_csv_floats(parts) -> Optional[Tuple[float, ...]]:
    try:
        return tuple(float(p.strip()) for p in parts)
    except Exception:
        return None


def _parse_csv_ints(parts) -> Optional[Tuple[int, ...]]:
    try:
        return tuple(int(p.strip()) for p in parts)
    except Exception:
        return None


class RoverSerialBridge(Node):
    """
    Reads serial CSV from ESP32 and publishes:
      - sensor_msgs/Imu on /imu/data_raw
      - sensor_msgs/Temperature on /imu/temperature
      - nav_msgs/Odometry on /wheel/odom
      - optional TF odom->base_link

    Expected incoming (choose one):
      A) 10 fields:
         ts_ms,ax,ay,az,gx,gy,gz,temp,left_ticks,right_ticks

      B) 8-field IMU line:
         ts_ms,ax,ay,az,gx,gy,gz,temp
         and 3-field encoder line:
         ts_ms,left_ticks,right_ticks

    Units expected by ROS:
      accel m/s^2, gyro rad/s, temp Celsius.
    """

    def __init__(self):
        super().__init__("rover_serial_bridge")

        # Serial parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("serial_timeout_s", 0.1)

        # Frames & topics
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")

        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("temp_topic", "/imu/temperature")
        self.declare_parameter("odom_topic", "/wheel/odom")

        # Odometry params
        self.declare_parameter("wheel_radius_m", 0.05)     # 5 cm default
        self.declare_parameter("wheel_separation_m", 0.30) # 30 cm default
        self.declare_parameter("ticks_per_rev", 28)        # e.g. 7PPR x4 = 28 (update!)
        self.declare_parameter("publish_tf", True)

        # Incoming data behavior
        self.declare_parameter("enc_is_cumulative", True)  # recommended True

        # If your ESP32 sends g and deg/s instead of SI, set these:
        self.declare_parameter("incoming_accel_in_g", False)
        self.declare_parameter("incoming_gyro_in_dps", False)

        # Covariances (simple defaults; tune later)
        self.declare_parameter("gyro_cov", 0.02)     # (rad/s)^2
        self.declare_parameter("accel_cov", 0.2)     # (m/s^2)^2
        self.declare_parameter("odom_xy_cov", 0.05)  # m^2
        self.declare_parameter("odom_yaw_cov", 0.2)  # rad^2

        # Publishers
        imu_topic = self.get_parameter("imu_topic").value
        temp_topic = self.get_parameter("temp_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.temp_pub = self.create_publisher(Temperature, temp_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # State for odometry integration
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.last_enc_stamp: Optional[float] = None  # seconds (ROS clock)

        # Open serial
        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        timeout_s = float(self.get_parameter("serial_timeout_s").value)

        try:
            self.ser = serial.Serial(port, baud, timeout=timeout_s)
            # Flush initial garbage
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {port}: {e}")
            raise

        # Timer to poll serial (fast)
        self.timer = self.create_timer(0.005, self._poll_serial)  # 200 Hz poll

    def _poll_serial(self):
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                # Ignore header lines
                if line.lower().startswith("t_ms") or "ax" in line.lower():
                    continue
                self._handle_line(line)
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def _handle_line(self, line: str):
        parts = [p.strip() for p in line.split(",") if p.strip() != ""]
        n = len(parts)

        # Format A (10 fields)
        if n == 10:
            # ts, ax, ay, az, gx, gy, gz, temp, L, R
            floats = _parse_csv_floats(parts[:8])
            ints = _parse_csv_ints(parts[8:])
            if floats is None or ints is None:
                return

            ts_ms = int(floats[0])
            imu = ImuSample(
                stamp_ms=ts_ms,
                ax=floats[1], ay=floats[2], az=floats[3],
                gx=floats[4], gy=floats[5], gz=floats[6],
                temp_c=floats[7],
            )
            enc = EncSample(stamp_ms=ts_ms, left_ticks=ints[0], right_ticks=ints[1])

            self._publish_imu(imu)
            self._publish_temp(imu)
            self._update_and_publish_odom(enc)

        # Format B IMU (8 fields)
        elif n == 8:
            floats = _parse_csv_floats(parts)
            if floats is None:
                return
            ts_ms = int(floats[0])
            imu = ImuSample(
                stamp_ms=ts_ms,
                ax=floats[1], ay=floats[2], az=floats[3],
                gx=floats[4], gy=floats[5], gz=floats[6],
                temp_c=floats[7],
            )
            self._publish_imu(imu)
            self._publish_temp(imu)

        # Format B Encoder (3 fields)
        elif n == 3:
            ints = _parse_csv_ints(parts)
            if ints is None:
                return
            enc = EncSample(stamp_ms=ints[0], left_ticks=ints[1], right_ticks=ints[2])
            self._update_and_publish_odom(enc)

        else:
            # Unknown line length; ignore
            return

    def _publish_imu(self, s: ImuSample):
        incoming_accel_in_g = bool(self.get_parameter("incoming_accel_in_g").value)
        incoming_gyro_in_dps = bool(self.get_parameter("incoming_gyro_in_dps").value)

        ax, ay, az = s.ax, s.ay, s.az
        gx, gy, gz = s.gx, s.gy, s.gz

        if incoming_accel_in_g:
            # g -> m/s^2
            g_to_ms2 = 9.80665
            ax *= g_to_ms2
            ay *= g_to_ms2
            az *= g_to_ms2

        if incoming_gyro_in_dps:
            # deg/s -> rad/s
            dps_to_rads = math.pi / 180.0
            gx *= dps_to_rads
            gy *= dps_to_rads
            gz *= dps_to_rads

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("imu_frame").value

        # No orientation from raw MPU6500 (unless you run a filter).
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Simple diagonal covariances (tune later)
        gyro_cov = float(self.get_parameter("gyro_cov").value)
        accel_cov = float(self.get_parameter("accel_cov").value)

        msg.angular_velocity_covariance = [
            gyro_cov, 0.0, 0.0,
            0.0, gyro_cov, 0.0,
            0.0, 0.0, gyro_cov,
        ]
        msg.linear_acceleration_covariance = [
            accel_cov, 0.0, 0.0,
            0.0, accel_cov, 0.0,
            0.0, 0.0, accel_cov,
        ]

        self.imu_pub.publish(msg)

    def _publish_temp(self, s: ImuSample):
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("imu_frame").value
        msg.temperature = float(s.temp_c)
        msg.variance = 0.0
        self.temp_pub.publish(msg)

    def _update_and_publish_odom(self, e: EncSample):
        wheel_r = float(self.get_parameter("wheel_radius_m").value)
        wheel_sep = float(self.get_parameter("wheel_separation_m").value)
        ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        enc_is_cumulative = bool(self.get_parameter("enc_is_cumulative").value)

        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        if self.last_enc_stamp is None:
            self.last_enc_stamp = now_s

        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = e.left_ticks
            self.last_right_ticks = e.right_ticks
            return

        if enc_is_cumulative:
            dL_ticks = e.left_ticks - self.last_left_ticks
            dR_ticks = e.right_ticks - self.last_right_ticks
            self.last_left_ticks = e.left_ticks
            self.last_right_ticks = e.right_ticks
        else:
            dL_ticks = e.left_ticks
            dR_ticks = e.right_ticks

        dt = max(1e-4, now_s - self.last_enc_stamp)
        self.last_enc_stamp = now_s

        # ticks -> wheel travel (meters)
        meters_per_tick = (2.0 * math.pi * wheel_r) / ticks_per_rev
        dL = dL_ticks * meters_per_tick
        dR = dR_ticks * meters_per_tick

        d_center = 0.5 * (dR + dL)
        d_theta = (dR - dL) / wheel_sep

        # Integrate pose
        self.yaw += d_theta
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # wrap

        self.x += d_center * math.cos(self.yaw)
        self.y += d_center * math.sin(self.yaw)

        # Velocities
        v = d_center / dt
        w = d_theta / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.get_parameter("odom_frame").value
        odom.child_frame_id = self.get_parameter("base_frame").value

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # yaw -> quaternion
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Covariances (very rough defaults)
        xy_cov = float(self.get_parameter("odom_xy_cov").value)
        yaw_cov = float(self.get_parameter("odom_yaw_cov").value)

        odom.pose.covariance = [
            xy_cov, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   xy_cov, 0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   999.0,  0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   999.0,  0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   999.0,  0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   yaw_cov
        ]

        self.odom_pub.publish(odom)

        # TF odom -> base_link
        if bool(self.get_parameter("publish_tf").value):
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.get_parameter("odom_frame").value
            t.child_frame_id = self.get_parameter("base_frame").value
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = RoverSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()