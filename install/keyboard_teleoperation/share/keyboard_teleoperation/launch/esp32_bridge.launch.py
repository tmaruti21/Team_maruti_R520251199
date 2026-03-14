from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_base_m = LaunchConfiguration('wheel_base_m')
    max_linear_speed = LaunchConfiguration('max_linear_speed')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('wheel_base_m', default_value='0.24'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.35'),
        LogInfo(msg='Start keyboard control in another terminal:'),
        LogInfo(msg='ros2 run keyboard_teleoperation keyboard_teleop'),
        Node(
            package='keyboard_teleoperation',
            executable='esp32_cmd_bridge',
            name='esp32_cmd_bridge',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'wheel_base_m': wheel_base_m,
                'max_linear_speed': max_linear_speed,
            }],
        ),
    ])
