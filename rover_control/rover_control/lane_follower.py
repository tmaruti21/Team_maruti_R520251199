# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import Image
# # from geometry_msgs.msg import Twist
# # from cv_bridge import CvBridge
# # import cv2
# # import numpy as np

# # class LaneFollower(Node):
# #     def __init__(self):
# #         super().__init__('lane_follower')
        
# #         # Subscribe to the Astra camera RGB feed (update topic name as needed)
# #         self.subscription = self.create_subscription(
# #             Image,
# #             '/camera/color/image_raw', 
# #             self.image_callback,
# #             10)
            
# #         # Publisher for motor commands
# #         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
# #         self.bridge = CvBridge()
        
# #         # Define yellow/orange HSV range (You will need to tune this!)
# #         self.lower_yellow = np.array([111, 57, 5])
# #         self.upper_yellow = np.array([131, 157, 105])

# #     def image_callback(self, msg):
# #         # 1. Convert ROS Image to OpenCV Image
# #         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
# #         height, width, _ = frame.shape
        
# #         # 2. Define Region of Interest (ROI) - Bottom half of the screen
# #         roi = frame[int(height/2):height, 0:width]
        
# #         # 3. Convert to HSV and create a mask for yellow/orange
# #         hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
# #         mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
# #         # 4. Find the center of the lane (Centroid of the mask)
# #         M = cv2.moments(mask)
        
# #         twist = Twist()
        
# #         if M["m00"] > 0:
# #             # Calculate the X coordinate of the center of the colored strips
# #             cx = int(M["m10"] / M["m00"])
            
# #             # The ideal center is the middle of the camera frame
# #             image_center = width // 2
# #             error = cx - image_center
            
# #             # 5. Simple Control Logic (Deadband of 50 pixels to prevent jitter)
# #             if error > 50:
# #                 self.get_logger().info("Steering Right")
# #                 twist.linear.x = 0.1   # Move forward slightly
# #                 twist.angular.z = -0.3 # Turn Right
# #             elif error < -50:
# #                 self.get_logger().info("Steering Left")
# #                 twist.linear.x = 0.1   # Move forward slightly
# #                 twist.angular.z = 0.3  # Turn Left
# #             else:
# #                 self.get_logger().info("Going Forward")
# #                 twist.linear.x = 0.2   # Move forward straight
# #                 twist.angular.z = 0.0
                
# #             # Optional: Visualize what the robot sees
# #             cv2.circle(roi, (cx, int(roi.shape[0]/2)), 5, (0, 0, 255), -1)
# #         else:
# #             self.get_logger().info("No lane detected! Stopping.")
# #             twist.linear.x = 0.0
# #             twist.angular.z = 0.0

# #         # Publish the command
# #         self.publisher.publish(twist)
        
# #         # Show the mask for debugging (Comment out in production to save CPU)
# #         cv2.imshow("Mask", mask)
# #         cv2.imshow("ROI", roi)
# #         cv2.waitKey(1)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = LaneFollower()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()



# ###########################################################################################################################################################3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class VisionCommandNode(Node):
#     def __init__(self):
#         super().__init__('vision_command_node')
        
#         # Subscribe to the camera feed
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/color/image_raw', 
#             self.image_callback,
#             10)
            
#         # Publish simple string commands to a new topic
#         self.publisher = self.create_publisher(String, '/lane_commands', 10)
#         self.bridge = CvBridge()
        
#         # INSERT YOUR CALIBRATED HSV VALUES HERE
#         self.lower_color = np.array([111, 57, 5])   
#         self.upper_color = np.array([131, 157, 105]) 

#     def image_callback(self, msg):
#         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         height, width, _ = frame.shape
        
#         # Crop to the bottom 40% of the screen to ignore background
#         roi = frame[int(height * 0.6):height, 0:width]
#         roi_height, roi_width, _ = roi.shape
        
#         # Convert to HSV and mask
#         hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        
#         # Split mask to find left and right strips independently
#         midpoint = roi_width // 2
#         left_mask = mask[:, :midpoint]
#         right_mask = mask[:, midpoint:]
        
#         M_left = cv2.moments(left_mask)
#         M_right = cv2.moments(right_mask)
        
#         cx_left = None
#         cx_right = None
        
#         if M_left["m00"] > 0:
#             cx_left = int(M_left["m10"] / M_left["m00"])
        
#         if M_right["m00"] > 0:
#             cx_right = int(M_right["m10"] / M_right["m00"]) + midpoint

#         road_center = None
        
#         # Determine where the center of the 100cm road is
#         if cx_left is not None and cx_right is not None:
#             road_center = (cx_left + cx_right) // 2
#         elif cx_left is not None:
#             # TUNE THIS: Pixel distance from left strip to the center of the 100cm track
#             road_center = cx_left + 250 
#         elif cx_right is not None:
#             # TUNE THIS: Pixel distance from right strip to the center of the 100cm track
#             road_center = cx_right - 250
            
#         command_msg = String()

#         if road_center is not None:
#             # Calculate how far off-center we are
#             error = road_center - midpoint
            
#             # Draw visuals for debugging
#             cv2.circle(roi, (road_center, roi_height//2), 8, (255, 0, 0), -1) 
#             if cx_left: cv2.circle(roi, (cx_left, roi_height//2), 5, (0, 255, 0), -1) 
#             if cx_right: cv2.circle(roi, (cx_right, roi_height//2), 5, (0, 255, 0), -1) 
            
#             # --- DECISION LOGIC ---
#             # Deadband of 40 pixels: If the error is small, just drive forward.
#             # If the error is large, turn to correct it.
#             if error > 40:
#                 self.get_logger().info("Decision: Right ('l')")
#                 command_msg.data = 'l'
#             elif error < -40:
#                 self.get_logger().info("Decision: Left ('r')")
#                 command_msg.data = 'r'
#             else:
#                 self.get_logger().info("Decision: Forward ('f')")
#                 command_msg.data = 'f'
#         else:
#             # If it completely loses the track, send a stop ('s') or backward ('b') command
#             self.get_logger().warning("Lost Track! Decision: Stop ('s')")
#             command_msg.data = 's' 

#         # Publish the character
#         self.publisher.publish(command_msg)
        
#         # Show debug window
#         cv2.imshow("Lane Vision Brain", roi)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = VisionCommandNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

########################################################################################################################################3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionCommandNode(Node):
    def __init__(self):
        super().__init__('vision_command_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(String, '/lane_commands', 10)
        self.bridge = CvBridge()

        # Calibrated HSV values
        self.lower_color = np.array([20, 100, 100])
        self.upper_color = np.array([40, 255, 255])

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = frame.shape

        # Bottom 40% ROI
        roi = frame[int(height * 0.6):height, 0:width]
        roi_height, roi_width, _ = roi.shape

        # HSV mask
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        # Split into left and right halves
        midpoint = roi_width // 2
        left_mask = mask[:, :midpoint]
        right_mask = mask[:, midpoint:]

        M_left = cv2.moments(left_mask)
        M_right = cv2.moments(right_mask)

        cx_left = None
        cx_right = None

        if M_left["m00"] > 0:
            cx_left = int(M_left["m10"] / M_left["m00"])

        if M_right["m00"] > 0:
            cx_right = int(M_right["m10"] / M_right["m00"]) + midpoint

        road_center = None

        if cx_left is not None and cx_right is not None:
            road_center = (cx_left + cx_right) // 2
        elif cx_left is not None:
            road_center = cx_left + 250
        elif cx_right is not None:
            road_center = cx_right - 250

        command_msg = String()

        if road_center is not None:
            error = road_center - midpoint

            # Decision logic
            if error > 40:
                self.get_logger().info("Decision: Right ('l')")
                command_msg.data = 'l'
            elif error < -40:
                self.get_logger().info("Decision: Left ('r')")
                command_msg.data = 'r'
            else:
                self.get_logger().info("Decision: Forward ('f')")
                command_msg.data = 'f'
        else:
            self.get_logger().warning("Lost Track! Decision: Stop ('s')")
            command_msg.data = 's'

        self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()