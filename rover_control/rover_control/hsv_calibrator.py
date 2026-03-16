import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# OpenCV requires a callback function for trackbars, even if it does nothing
def empty_callback(x):
    pass

class HSVCalibrator(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        
        # Subscribe to the camera feed published by your Astra node
        self.subscription = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_callback, 
            10)
        self.bridge = CvBridge()

        # Create a window with sliders (trackbars)
        cv2.namedWindow('Trackbars')
        cv2.resizeWindow('Trackbars', 400, 300)
        
        # Initial trackbar positions (H: 0-179, S: 0-255, V: 0-255)
        cv2.createTrackbar('H Min', 'Trackbars', 0, 179, empty_callback)
        cv2.createTrackbar('S Min', 'Trackbars', 0, 255, empty_callback)
        cv2.createTrackbar('V Min', 'Trackbars', 0, 255, empty_callback)
        cv2.createTrackbar('H Max', 'Trackbars', 179, 179, empty_callback)
        cv2.createTrackbar('S Max', 'Trackbars', 255, 255, empty_callback)
        cv2.createTrackbar('V Max', 'Trackbars', 255, 255, empty_callback)
        
        self.get_logger().info("HSV Calibrator ready. Adjust the sliders!")
        self.get_logger().info("Press 'p' with the video window selected to print the current values.")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Read the current positions of all 6 trackbars
        h_min = cv2.getTrackbarPos('H Min', 'Trackbars')
        s_min = cv2.getTrackbarPos('S Min', 'Trackbars')
        v_min = cv2.getTrackbarPos('V Min', 'Trackbars')
        h_max = cv2.getTrackbarPos('H Max', 'Trackbars')
        s_max = cv2.getTrackbarPos('S Max', 'Trackbars')
        v_max = cv2.getTrackbarPos('V Max', 'Trackbars')

        # Create upper and lower bounds based on slider positions
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # Generate the mask (white where the color matches, black elsewhere)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Apply the mask to the original image to see what is being isolated
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Display the windows
        cv2.imshow('Original Feed', frame)
        cv2.imshow('Mask (Make the boundary white, everything else black)', mask)
        cv2.imshow('Isolated Color', result)

        # Wait 1ms for a key press
        key = cv2.waitKey(1) & 0xFF
        
        # If 'p' is pressed, print the arrays to the terminal so you can copy-paste them
        if key == ord('p'):
            self.get_logger().info("\n--- COPY THESE INTO YOUR lane_follower.py ---")
            self.get_logger().info(f"self.lower_yellow = np.array([{h_min}, {s_min}, {v_min}])")
            self.get_logger().info(f"self.upper_yellow = np.array([{h_max}, {s_max}, {v_max}])\n")

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down calibrator.')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()