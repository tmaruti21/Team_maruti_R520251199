import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class AstraCameraPublisher(Node):
    def __init__(self):
        super().__init__('astra_camera_publisher')
        
        # Topic name to publish the video feed to
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        
        # 0 is usually the default ID for the first connected USB camera (/dev/video0)
        # If it fails, try changing this to 1, 2, or whatever index your Astra mounts as.
        self.video_capture = cv2.VideoCapture(0)
        
        if not self.video_capture.isOpened():
            self.get_logger().error('Could not open video device. Check the camera index!')
            
        self.bridge = CvBridge()
        
        # Set a timer to read and publish frames at ~30 frames per second
        timer_period = 1.0 / 30.0 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Camera publisher node has started.')

    def timer_callback(self):
        ret, frame = self.video_capture.read()
        
        if ret:
            # Convert the OpenCV image (BGR) to a ROS2 Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(image_msg)
        else:
            self.get_logger().warning('Ignored empty frame from camera.')

    def destroy_node(self):
        # Release the camera hardware cleanly on shutdown
        self.video_capture.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AstraCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera publisher.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()