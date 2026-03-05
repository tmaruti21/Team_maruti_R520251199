#!/usr/bin/env python3
"""
Lane Detection Node with Depth and Point Cloud Integration
Subscribes to color, depth images and point clouds from Orbbec camera
Processes lanes and outputs with 3D spatial information
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # Parameters
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('debug', True)
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('roi_top_ratio', 0.6)  # Top of region of interest
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('min_line_length', 50)
        self.declare_parameter('max_line_gap', 50)
        
        camera_name = self.get_parameter('camera_name').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Store latest images
        self.latest_color = None
        self.latest_depth = None
        self.latest_depth_array = None
        self.camera_info = None
        
        # Subscribers
        self.color_sub = self.create_subscription(
            Image,
            f'/{camera_name}/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            f'/{camera_name}/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            f'/{camera_name}/depth/points',
            self.pointcloud_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f'/{camera_name}/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.lane_viz_pub = self.create_publisher(
            Image,
            '/lane_detection/visualization',
            10
        )
        
        self.lane_depth_pub = self.create_publisher(
            Image,
            '/lane_detection/depth_overlay',
            10
        )
        
        self.depth_colored_pub = self.create_publisher(
            Image,
            '/lane_detection/depth_colormap',
            10
        )
        
        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz
        
        self.get_logger().info('Lane Detection Node initialized')
        self.get_logger().info(f'Subscribing to: /{camera_name}/color/image_raw')
        self.get_logger().info(f'Subscribing to: /{camera_name}/depth/image_raw')
        self.get_logger().info(f'Subscribing to: /{camera_name}/depth/points')
    
    def color_callback(self, msg):
        """Store latest color image"""
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error (color): {e}')
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            # Depth is usually in 16UC1 or 32FC1 format
            self.latest_depth = msg
            self.latest_depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error (depth): {e}')
    
    def pointcloud_callback(self, msg):
        """Process point cloud data"""
        # Store point cloud message for 3D processing if needed
        pass
    
    def camera_info_callback(self, msg):
        """Store camera calibration info"""
        self.camera_info = msg
    
    def detect_lanes(self, image):
        """
        Detect lanes using Canny edge detection and Hough transform
        Returns: lines, annotated image
        """
        if image is None:
            return None, None
        
        # Get parameters
        canny_low = self.get_parameter('canny_low').value
        canny_high = self.get_parameter('canny_high').value
        roi_top_ratio = self.get_parameter('roi_top_ratio').value
        hough_threshold = self.get_parameter('hough_threshold').value
        min_line_length = self.get_parameter('min_line_length').value
        max_line_gap = self.get_parameter('max_line_gap').value
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blur, canny_low, canny_high)
        
        # Define region of interest (ROI)
        height, width = edges.shape
        roi_top = int(height * roi_top_ratio)
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (0, roi_top),
            (width, roi_top),
            (width, height)
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=hough_threshold,
            minLineLength=min_line_length,
            maxLineGap=max_line_gap
        )
        
        # Create output image
        output = image.copy()
        
        # Draw detected lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        return lines, output
    
    def get_depth_at_point(self, x, y):
        """Get depth value at specific pixel coordinates"""
        if self.latest_depth_array is None:
            return None
        
        height, width = self.latest_depth_array.shape
        if 0 <= x < width and 0 <= y < height:
            depth = self.latest_depth_array[y, x]
            # Convert to meters (assuming depth is in mm)
            return depth / 1000.0 if depth > 0 else None
        return None
    
    def overlay_depth_on_lanes(self, lane_image, lines):
        """Draw depth information on detected lanes"""
        if lines is None or self.latest_depth_array is None:
            return lane_image
        
        output = lane_image.copy()
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # Get depth at start and end points
            depth1 = self.get_depth_at_point(x1, y1)
            depth2 = self.get_depth_at_point(x2, y2)
            
            # Draw depth information
            if depth1 is not None:
                text = f"{depth1:.2f}m"
                cv2.putText(output, text, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            if depth2 is not None:
                text = f"{depth2:.2f}m"
                cv2.putText(output, text, (x2, y2 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return output
    
    def create_depth_colormap(self):
        """Create a colored depth visualization"""
        if self.latest_depth_array is None:
            return None
        
        # Normalize depth for visualization
        depth_normalized = cv2.normalize(
            self.latest_depth_array,
            None,
            0, 255,
            cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
        )
        
        # Apply colormap
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        
        return depth_colormap
    
    def process_frame(self):
        """Main processing loop"""
        if self.latest_color is None:
            return
        
        # Detect lanes
        lines, lane_viz = self.detect_lanes(self.latest_color)
        
        if lane_viz is not None:
            # Publish lane visualization
            try:
                lane_msg = self.bridge.cv2_to_imgmsg(lane_viz, encoding='bgr8')
                self.lane_viz_pub.publish(lane_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Error publishing lane viz: {e}')
            
            # Overlay depth information
            if self.latest_depth_array is not None:
                depth_overlay = self.overlay_depth_on_lanes(lane_viz, lines)
                try:
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_overlay, encoding='bgr8')
                    self.lane_depth_pub.publish(depth_msg)
                except CvBridgeError as e:
                    self.get_logger().error(f'Error publishing depth overlay: {e}')
        
        # Publish colored depth map
        depth_colormap = self.create_depth_colormap()
        if depth_colormap is not None:
            try:
                depth_color_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
                self.depth_colored_pub.publish(depth_color_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Error publishing depth colormap: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
