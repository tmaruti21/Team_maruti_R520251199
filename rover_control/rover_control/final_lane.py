import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionBrain(Node):
    def __init__(self):
        super().__init__('vision_brain')
        
        # Subscribe to Orbbec Astra raw image
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
            
        # Publish to your custom ESP32 bridge topic
        self.command_publisher = self.create_publisher(String, '/lane_commands', 10)
        self.bridge = CvBridge()
        
        # Tune these based on the arena lighting!
        self.lower_yellow = np.array([20, 100, 100]) 
        self.upper_yellow = np.array([40, 255, 255])
        
        # Control Deadzone (in pixels)
        self.deadzone = 50 

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width = frame.shape[:2]

        # 1. REGION OF INTEREST (Keep only bottom 60% of the image)
        roi = frame[int(height*0.4):height, 0:width]
        roi_h, roi_w = roi.shape[:2]

        # 2. BIRD'S EYE VIEW (Perspective Transform)
        # YOU MUST TUNE THESE POINTS! Place a 100cm square on the ground and map it.
        # src_points = np.float32([[0, roi_h], [roi_w, roi_h], [roi_w//2 + 100, 0], [roi_w//2 - 100, 0]])
        src_points = np.float32([[29, 512], [1152, 511], [971, 48], [221, 53]])
        dst_points = np.float32([[0, roi_h], [roi_w, roi_h], [roi_w, 0], [0, 0]])
        matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        birdseye = cv2.warpPerspective(roi, matrix, (roi_w, roi_h))

        # 3. HSV MASKING
        hsv = cv2.cvtColor(birdseye, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        # Remove carpet noise
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 4. CONTOUR FILTERING (Ignore Yellow Cones)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        left_x, right_x = None, None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500: # Ignore tiny specks of dust
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                
                # A cone will be somewhat square in top-down view. Tape is a thin line.
                # If it's a long vertical line (aspect ratio < 0.5) or long horizontal
                if aspect_ratio < 0.8 or aspect_ratio > 1.2:
                    # Is it on the left or right side of the screen?
                    if x < roi_w // 2:
                        left_x = x + (w//2) # Center of left tape
                    else:
                        right_x = x + (w//2) # Center of right tape

        # 5. CONTROL LOGIC
        command = 's' # Default to stop if completely lost
        center_of_screen = roi_w // 2

        if left_x and right_x:
            # We see both lines! Calculate midpoint.
            track_center = (left_x + right_x) // 2
        elif left_x:
            # Only see left line (maybe turning right). Estimate track center.
            # Assuming track is roughly 100cm (which maps to X pixels in your Bird's Eye view)
            track_center = left_x + 300 # TUNE THIS OFFSET
        elif right_x:
            # Only see right line
            track_center = right_x - 300 # TUNE THIS OFFSET
        else:
            track_center = center_of_screen # No lines, go straight and pray, or stop.

        # 6. BANG-BANG DIFFERENTIAL DRIVE COMMANDS
        error = track_center - center_of_screen
        
        cmd_msg = String()
        if error < -self.deadzone:
            cmd_msg.data = 'l'
        elif error > self.deadzone:
            cmd_msg.data = 'r'
        else:
            cmd_msg.data = 'f'

        self.command_publisher.publish(cmd_msg)
        
        # (Optional) cv2.imshow for debugging here

def main(args=None):
    rclpy.init(args=args)
    node = VisionBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()