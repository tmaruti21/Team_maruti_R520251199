from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """
    Combined launch file for Orbbec camera and lane detection
    Starts camera, lane detection, and point cloud processing
    """
    
    # Declare arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Name of the camera'
    )
    
    # Camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'astra_pro_plus.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'enable_point_cloud': 'true',
            'enable_colored_point_cloud': 'false',
            'enable_depth': 'true',
            'enable_color': 'true',
        }.items()
    )
    
    # Lane detection launch
    lane_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lane_detection'),
                'launch',
                'lane_detection.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
        }.items()
    )
    
    return LaunchDescription([
        camera_name_arg,
        camera_launch,
        lane_detection_launch,
    ])
