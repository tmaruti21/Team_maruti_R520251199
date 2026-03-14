from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    wheel_base_m = LaunchConfiguration('wheel_base_m')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    manual_timeout_s = LaunchConfiguration('manual_timeout_s')
    auto_timeout_s = LaunchConfiguration('auto_timeout_s')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('wheel_base_m', default_value='0.24'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.35'),
        DeclareLaunchArgument('manual_timeout_s', default_value='0.50'),
        DeclareLaunchArgument('auto_timeout_s', default_value='0.70'),

        LogInfo(msg='Starting auto_nav pipeline: infer_node -> cmd_vel_mux -> esp32_auto_bridge'),
        LogInfo(msg='Keyboard teleop stays available via /cmd_vel_manual'),
        LogInfo(msg='Run keyboard control in another terminal when needed:'),
        LogInfo(msg='ros2 run keyboard_teleoperation keyboard_teleop --ros-args -p cmd_topic:=/cmd_vel_manual'),

        Node(
            package='custom_follow',
            executable='infer_node',
            name='lane_infer_node',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_auto')],
        ),

        Node(
            package='auto_nav',
            executable='cmd_vel_mux',
            name='cmd_vel_mux',
            output='screen',
            parameters=[{
                'auto_topic': '/cmd_vel_auto',
                'manual_topic': '/cmd_vel_manual',
                'output_topic': '/cmd_vel_out',
                'manual_priority': True,
                'manual_timeout_s': manual_timeout_s,
                'auto_timeout_s': auto_timeout_s,
                'publish_rate_hz': 30.0,
            }],
        ),

        Node(
            package='auto_nav',
            executable='esp32_auto_bridge',
            name='esp32_auto_bridge',
            output='screen',
            parameters=[{
                'cmd_topic': '/cmd_vel_out',
                'serial_port': serial_port,
                'baud_rate': baud_rate,
                'wheel_base_m': wheel_base_m,
                'max_linear_speed': max_linear_speed,
            }],
        ),
    ])
