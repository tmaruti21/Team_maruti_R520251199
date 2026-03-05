from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for lane detection with depth and point cloud processing
    """
    
    # Declare arguments
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Name of the camera'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug mode'
    )
    
    # Lane detection node
    lane_detection_node = Node(
        package='lane_detection',
        executable='lane_detection_node',
        name='lane_detection',
        output='screen',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'debug': LaunchConfiguration('debug'),
            'canny_low': 50,
            'canny_high': 150,
            'roi_top_ratio': 0.6,
            'hough_threshold': 50,
            'min_line_length': 50,
            'max_line_gap': 50,
        }]
    )
    
    # Point cloud processor node
    pointcloud_processor_node = Node(
        package='lane_detection',
        executable='pointcloud_processor',
        name='pointcloud_processor',
        output='screen',
        parameters=[{
            'camera_name': LaunchConfiguration('camera_name'),
            'voxel_size': 0.05,
            'z_filter_min': 0.3,
            'z_filter_max': 5.0,
        }]
    )
    
    return LaunchDescription([
        camera_name_arg,
        debug_arg,
        lane_detection_node,
        pointcloud_processor_node,
    ])
