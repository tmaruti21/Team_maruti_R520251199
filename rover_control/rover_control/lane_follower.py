# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class LaneFollower(Node):
#     def __init__(self):
#         super().__init__('lane_follower')
        
#         # Subscribe to the Astra camera RGB feed (update topic name as needed)
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/color/image_raw', 
#             self.image_callback,
#             10)
            
#         # Publisher for motor commands
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
#         self.bridge = CvBridge()
        
#         # Define yellow/orange HSV range (You will need to tune this!)
#         self.lower_yellow = np.array([111, 57, 5])
#         self.upper_yellow = np.array([131, 157, 105])

#     def image_callback(self, msg):
#         # 1. Convert ROS Image to OpenCV Image
#         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         height, width, _ = frame.shape
        
#         # 2. Define Region of Interest (ROI) - Bottom half of the screen
#         roi = frame[int(height/2):height, 0:width]
        
#         # 3. Convert to HSV and create a mask for yellow/orange
#         hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
#         # 4. Find the center of the lane (Centroid of the mask)
#         M = cv2.moments(mask)
        
#         twist = Twist()
        
#         if M["m00"] > 0:
#             # Calculate the X coordinate of the center of the colored strips
#             cx = int(M["m10"] / M["m00"])
            
#             # The ideal center is the middle of the camera frame
#             image_center = width // 2
#             error = cx - image_center
            
#             # 5. Simple Control Logic (Deadband of 50 pixels to prevent jitter)
#             if error > 50:
#                 self.get_logger().info("Steering Right")
#                 twist.linear.x = 0.1   # Move forward slightly
#                 twist.angular.z = -0.3 # Turn Right
#             elif error < -50:
#                 self.get_logger().info("Steering Left")
#                 twist.linear.x = 0.1   # Move forward slightly
#                 twist.angular.z = 0.3  # Turn Left
#             else:
#                 self.get_logger().info("Going Forward")
#                 twist.linear.x = 0.2   # Move forward straight
#                 twist.angular.z = 0.0
                
#             # Optional: Visualize what the robot sees
#             cv2.circle(roi, (cx, int(roi.shape[0]/2)), 5, (0, 0, 255), -1)
#         else:
#             self.get_logger().info("No lane detected! Stopping.")
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0

#         # Publish the command
#         self.publisher.publish(twist)
        
#         # Show the mask for debugging (Comment out in production to save CPU)
#         cv2.imshow("Mask", mask)
#         cv2.imshow("ROI", roi)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LaneFollower()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



###########################################################################################################################################################3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class DualStripFollower(Node):
    def __init__(self):
        super().__init__('dual_strip_follower')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw', 
            self.image_callback,
            10)
            
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        
        # Paste the arrays you get from the hsv_calibrator.py node here!
        self.lower_color = np.array([0, 0, 0])   # REPLACE ME
        self.upper_color = np.array([179, 255, 255]) # REPLACE ME

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = frame.shape
        
        # Crop to the bottom half to ignore the horizon
        roi = frame[int(height/2):height, 0:width]
        roi_height, roi_width, _ = roi.shape
        
        # Convert to HSV and create the mask
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        
        # Split the mask into left and right halves
        midpoint = roi_width // 2
        left_mask = mask[:, :midpoint]
        right_mask = mask[:, midpoint:]
        
        # Calculate moments for both halves
        M_left = cv2.moments(left_mask)
        M_right = cv2.moments(right_mask)
        
        cx_left = None
        cx_right = None
        
        if M_left["m00"] > 0:
            cx_left = int(M_left["m10"] / M_left["m00"])
        
        if M_right["m00"] > 0:
            # Add the midpoint offset because the right mask starts at the middle of the image
            cx_right = int(M_right["m10"] / M_right["m00"]) + midpoint

        twist = Twist()
        road_center = None
        
        # Determine the target center of the road
        if cx_left is not None and cx_right is not None:
            # Both lines visible: Target is exactly between them
            road_center = (cx_left + cx_right) // 2
        elif cx_left is not None:
            # Only left line visible: Guess the center is a fixed distance to the right
            # (You will need to tune this 300 pixel offset based on your camera's field of view)
            road_center = cx_left + 300 
        elif cx_right is not None:
            # Only right line visible: Guess the center is a fixed distance to the left
            road_center = cx_right - 300
            
        if road_center is not None:
            error = road_center - midpoint
            
            # Draw targeting visuals for debugging
            cv2.circle(roi, (road_center, roi_height//2), 8, (255, 0, 0), -1) # Target center (Blue)
            if cx_left: cv2.circle(roi, (cx_left, roi_height//2), 5, (0, 255, 0), -1) # Left line (Green)
            if cx_right: cv2.circle(roi, (cx_right, roi_height//2), 5, (0, 255, 0), -1) # Right line (Green)
            
            # Control Logic (Tune the angular.z multipliers based on your skid-steer sensitivity)
            if error > 40:
                self.get_logger().info("Correcting Right")
                twist.linear.x = 0.15 
                twist.angular.z = -0.4 
            elif error < -40:
                self.get_logger().info("Correcting Left")
                twist.linear.x = 0.15 
                twist.angular.z = 0.4  
            else:
                self.get_logger().info("Centered")
                twist.linear.x = 0.25 
                twist.angular.z = 0.0
        else:
            self.get_logger().warning("Lost both lines! Stopping.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)
        
        cv2.imshow("Dual Strip ROI", roi)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DualStripFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()