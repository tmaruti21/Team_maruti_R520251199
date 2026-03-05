#!/usr/bin/env python3
"""
Point Cloud Processor Node
Processes point cloud data from Orbbec camera for 3D lane mapping
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import struct


class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        # Parameters
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('voxel_size', 0.05)  # Downsample voxel size in meters
        self.declare_parameter('z_filter_min', 0.3)  # Min distance in meters
        self.declare_parameter('z_filter_max', 5.0)  # Max distance in meters
        
        camera_name = self.get_parameter('camera_name').value
        
        # Subscriber
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            f'/{camera_name}/depth/points',
            self.pointcloud_callback,
            10
        )
        
        # Publishers
        self.filtered_pc_pub = self.create_publisher(
            PointCloud2,
            '/lane_detection/filtered_pointcloud',
            10
        )
        
        self.ground_plane_pub = self.create_publisher(
            PointCloud2,
            '/lane_detection/ground_plane',
            10
        )
        
        self.get_logger().info('Point Cloud Processor Node initialized')
    
    def pointcloud_callback(self, msg):
        """Process incoming point cloud"""
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if points is None or len(points) == 0:
                return
            
            # Filter by distance
            filtered_points = self.filter_by_distance(points)
            
            # Downsample
            downsampled_points = self.downsample_voxel(filtered_points)
            
            # Detect ground plane
            ground_points, non_ground_points = self.detect_ground_plane(downsampled_points)
            
            # Publish filtered point cloud
            if len(non_ground_points) > 0:
                filtered_msg = self.array_to_pointcloud2(
                    non_ground_points,
                    msg.header.frame_id
                )
                self.filtered_pc_pub.publish(filtered_msg)
            
            # Publish ground plane
            if len(ground_points) > 0:
                ground_msg = self.array_to_pointcloud2(
                    ground_points,
                    msg.header.frame_id
                )
                self.ground_plane_pub.publish(ground_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        try:
            points_list = []
            
            for point in pc2.read_points(cloud_msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2]])
            
            if len(points_list) > 0:
                return np.array(points_list, dtype=np.float32)
            return None
        
        except Exception as e:
            self.get_logger().error(f'Error converting point cloud: {e}')
            return None
    
    def filter_by_distance(self, points):
        """Filter points by distance (z-axis)"""
        z_min = self.get_parameter('z_filter_min').value
        z_max = self.get_parameter('z_filter_max').value
        
        # Filter by z distance
        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        return points[mask]
    
    def downsample_voxel(self, points):
        """Downsample point cloud using voxel grid"""
        if len(points) == 0:
            return points
        
        voxel_size = self.get_parameter('voxel_size').value
        
        # Simple voxel grid downsampling
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        
        # Get unique voxels
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        
        return points[unique_indices]
    
    def detect_ground_plane(self, points):
        """
        Simple ground plane detection using RANSAC-like approach
        Returns: ground_points, non_ground_points
        """
        if len(points) < 3:
            return np.array([]), points
        
        # Assume ground is roughly along y-axis (negative y is down)
        # Simple threshold-based approach
        y_threshold = np.percentile(points[:, 1], 80)  # Top 20% in y
        
        ground_mask = points[:, 1] > y_threshold
        
        ground_points = points[ground_mask]
        non_ground_points = points[~ground_mask]
        
        return ground_points, non_ground_points
    
    def array_to_pointcloud2(self, points, frame_id):
        """Convert numpy array to PointCloud2 message"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create PointCloud2 message
        cloud_msg = pc2.create_cloud(header, fields, points)
        
        return cloud_msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
