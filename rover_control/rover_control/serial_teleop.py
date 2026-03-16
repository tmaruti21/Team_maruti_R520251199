import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import sys
import select
import termios
import tty
import threading

class SerialTeleopNode(Node):
    def __init__(self):
        super().__init__('serial_teleop_node')
        
        # --- SERIAL CONFIGURATION ---
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200 
        self.ser = None
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2) # Give ESP32 time to reset
            self.get_logger().info(f"Connected to ESP32 on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to ESP32: {e}")

        # Subscribe to the vision brain
        self.subscription = self.create_subscription(
            String,
            '/lane_commands',
            self.command_callback,
            10)
            
        self.get_logger().info("Listening to /lane_commands...")
        self.get_logger().info("--- MANUAL KEYBOARD OVERRIDE ACTIVE ---")
        self.get_logger().info("W: Forward ('f') | S: Backward ('b')")
        self.get_logger().info("A: Left    ('l') | D: Right    ('r')")
        self.get_logger().info("Spacebar: Stop ('s') | Ctrl+C: Exit")
        
        # Start the background thread for keyboard input
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def send_to_esp(self, command_char, source="Auto"):
        """Helper function to actually write to the serial port"""
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write((command_char + '\n').encode('utf-8'))
                self.get_logger().info(f"[{source}] Sent: {command_char}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial write failed: {e}")
        else:
            self.get_logger().warning(f"Serial not connected. Ignored: {command_char}")

    def command_callback(self, msg):
        # Triggered when the lane follower node publishes a command
        self.send_to_esp(msg.data, source="Auto")

    def keyboard_loop(self):
        """Background thread that listens for raw keystrokes in the terminal"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                # Non-blocking check for keyboard input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    if key == 'w':
                        self.send_to_esp('f', source="Manual")
                    elif key == 's':
                        self.send_to_esp('b', source="Manual")
                    elif key == 'a':
                        self.send_to_esp('l', source="Manual")
                    elif key == 'd':
                        self.send_to_esp('r', source="Manual")
                    elif key == ' ':
                        self.send_to_esp('s', source="Manual")
                    elif key == '\x03': # Ctrl+C to exit cleanly
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.send_to_esp('s', source="Shutdown") # Force stop motors
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # Caught gracefully by ROS
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()