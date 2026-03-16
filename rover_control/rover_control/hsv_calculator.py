import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ClickHSVCalibrator(Node):
    def __init__(self):
        super().__init__('click_hsv_calibrator')
        
        # Subscribe to your Astra camera feed
        self.subscription = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_callback, 
            10)
        self.bridge = CvBridge()
        
        self.current_frame = None
        
        # Create the window and attach the mouse click event
        cv2.namedWindow('Click on the Lane Color')
        cv2.setMouseCallback('Click on the Lane Color', self.mouse_callback)
        
        self.get_logger().info("Point-and-Click HSV Calibrator Ready!")
        self.get_logger().info("Click anywhere on the video feed to auto-generate the HSV range.")

    def image_callback(self, msg):
        # Update the current frame to display
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('Click on the Lane Color', self.current_frame)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        # Trigger only on left mouse click
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            
            # 1. Grab the BGR pixel exactly where you clicked
            bgr_pixel = self.current_frame[y, x]
            
            # 2. Convert that single pixel to a 1x1 image, then to HSV
            bgr_image = np.uint8([[bgr_pixel]])
            hsv_pixel = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)[0][0]
            
            h, s, v = hsv_pixel
            
            # 3. Create a range around the clicked color to generate lower/upper bounds
            # Hue is narrow (+/- 10), Saturation and Value are wider (+/- 50) to account for lighting
            lower_h = max(0, h - 10)
            upper_h = min(179, h + 10)
            
            lower_s = max(0, s - 50)
            upper_s = min(255, s + 50)
            
            lower_v = max(0, v - 50)
            upper_v = min(255, v + 50)
            
            # 4. Print the ready-to-use Python code to the terminal
            self.get_logger().info("\n--- COPY THESE INTO YOUR lane_follower.py ---")
            self.get_logger().info(f"Clicked exact HSV: [{h}, {s}, {v}]")
            self.get_logger().info(f"self.lower_color = np.array([{lower_h}, {lower_s}, {lower_v}])")
            self.get_logger().info(f"self.upper_color = np.array([{upper_h}, {upper_s}, {upper_v}])")
            self.get_logger().info("---------------------------------------------\n")

def main(args=None):
    rclpy.init(args=args)
    node = ClickHSVCalibrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down click calibrator.')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()