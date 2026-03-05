#!/usr/bin/env python3
"""
Multi-view visualization for lane detection system
Displays color, depth, lane detection, and overlays in a grid
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class MultiViewVisualizer(Node):
    def __init__(self):
        super().__init__('multi_view_visualizer')
        
        self.declare_parameter('camera_name', 'camera')
        camera_name = self.get_parameter('camera_name').value
        
        self.bridge = CvBridge()
        
        # Storage for latest images
        self.color_img = None
        self.depth_img = None
        self.lane_viz_img = None
        self.depth_overlay_img = None
        self.depth_colormap_img = None
        
        # Subscribers
        self.color_sub = self.create_subscription(
            Image, f'/{camera_name}/color/image_raw',
            self.color_callback, 10
        )
        
        self.depth_sub = self.create_subscription(
            Image, f'/{camera_name}/depth/image_raw',
            self.depth_callback, 10
        )
        
        self.lane_viz_sub = self.create_subscription(
            Image, '/lane_detection/visualization',
            self.lane_viz_callback, 10
        )
        
        self.depth_overlay_sub = self.create_subscription(
            Image, '/lane_detection/depth_overlay',
            self.depth_overlay_callback, 10
        )
        
        self.depth_colormap_sub = self.create_subscription(
            Image, '/lane_detection/depth_colormap',
            self.depth_colormap_callback, 10
        )
        
        # Timer for display update
        self.timer = self.create_timer(0.03, self.display_callback)  # ~30 Hz
        
        self.get_logger().info('Multi-view Visualizer started')
        self.get_logger().info('Press "q" to quit')
    
    def color_callback(self, msg):
        try:
            self.color_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Color error: {e}')
    
    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Convert to displayable format
            depth_normalized = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
            self.depth_img = cv2.cvtColor(depth_normalized.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f'Depth error: {e}')
    
    def lane_viz_callback(self, msg):
        try:
            self.lane_viz_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Lane viz error: {e}')
    
    def depth_overlay_callback(self, msg):
        try:
            self.depth_overlay_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Depth overlay error: {e}')
    
    def depth_colormap_callback(self, msg):
        try:
            self.depth_colormap_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Depth colormap error: {e}')
    
    def create_placeholder(self, text, width=640, height=480):
        """Create placeholder image with text"""
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(img, text, (width//4, height//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        return img
    
    def resize_to_fit(self, img, target_width, target_height):
        """Resize image maintaining aspect ratio"""
        if img is None:
            return self.create_placeholder("No Image", target_width, target_height)
        
        h, w = img.shape[:2]
        scale = min(target_width/w, target_height/h)
        new_w, new_h = int(w*scale), int(h*scale)
        resized = cv2.resize(img, (new_w, new_h))
        
        # Create canvas and center image
        canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        y_offset = (target_height - new_h) // 2
        x_offset = (target_width - new_w) // 2
        canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
        
        return canvas
    
    def display_callback(self):
        """Create and display grid of images"""
        # Define grid layout (2x3)
        cell_width, cell_height = 640, 480
        grid_width = cell_width * 3
        grid_height = cell_height * 2
        
        # Create grid
        grid = np.zeros((grid_height, grid_width, 3), dtype=np.uint8)
        
        # Prepare images with labels
        images = [
            (self.color_img, "Color Image"),
            (self.depth_img, "Depth Image"),
            (self.depth_colormap_img, "Depth Colormap"),
            (self.lane_viz_img, "Lane Detection"),
            (self.depth_overlay_img, "Depth Overlay"),
            (self.create_placeholder("Reserved", cell_width, cell_height), "Reserved")
        ]
        
        # Fill grid
        for idx, (img, label) in enumerate(images):
            row = idx // 3
            col = idx % 3
            
            # Resize and place image
            if img is None:
                img = self.create_placeholder(f"Waiting for\n{label}", cell_width, cell_height)
            else:
                img = self.resize_to_fit(img, cell_width, cell_height)
            
            # Add label
            cv2.putText(img, label, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Place in grid
            y_start = row * cell_height
            y_end = y_start + cell_height
            x_start = col * cell_width
            x_end = x_start + cell_width
            grid[y_start:y_end, x_start:x_end] = img
        
        # Add overall title
        cv2.putText(grid, "Lane Detection with Depth - Multi View", (20, grid_height - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 0), 2)
        
        # Display
        cv2.imshow('Lane Detection Multi-View', grid)
        
        # Handle key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quitting...')
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MultiViewVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
