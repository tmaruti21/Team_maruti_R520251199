# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class HardcodedPathNode(Node):
#     def __init__(self):
#         super().__init__('hardcoded_path_node')
        
#         # Publish directly to the topic that serial_teleop.py is already listening to
#         self.publisher = self.create_publisher(String, '/lane_commands', 10)
        
#         # --- YOUR HARDCODED SEQUENCE ---
#         in = "flrffffrlfffffllllfllflfffffff"
#         self.command_sequence = in[::-1] #"fflrffffrlfffffllllfllflfffffff"
#         self.current_index = 0
        
#         # --- TIMING CONFIGURATION ---
#         # 0.05 seconds = 50 milliseconds per character
#         self.tick_rate_seconds = 0.5 
        
#         # Create a timer that fires every 50ms
#         self.timer = self.create_timer(self.tick_rate_seconds, self.timer_callback)
        
#         self.get_logger().info(f"Starting hardcoded path.")
#         self.get_logger().info(f"Each character runs for {self.tick_rate_seconds * 1000} ms.")
#         self.get_logger().info(f"Total path duration: {(len(self.command_sequence) * self.tick_rate_seconds):.2f} seconds.")

#     def timer_callback(self):
#         msg = String()
        
#         # Check if we still have commands left in the string
#         if self.current_index < len(self.command_sequence):
#             # Grab the current character and publish it
#             msg.data = self.command_sequence[self.current_index]
#             self.publisher.publish(msg)
            
#             self.get_logger().info(f"Sent: '{msg.data}' | Step: {self.current_index + 1}/{len(self.command_sequence)}")
            
#             # Move to the next character for the next timer tick
#             self.current_index += 1
#         else:
#             # We reached the end of the string. Stop the rover.
#             msg.data = 's'
#             self.publisher.publish(msg)
#             self.get_logger().info("Path complete! Sending Stop ('s') and shutting down timer.")
            
#             # Cancel the timer so it doesn't keep running
#             self.timer.cancel()

# def main(args=None):
#     rclpy.init(args=args)
#     node = HardcodedPathNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Hardcoded path interrupted.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HardcodedPathNode(Node):
    def __init__(self):
        super().__init__('hardcoded_path_node')
        
        self.publisher = self.create_publisher(String, '/lane_commands', 10)
        
        # --- YOUR HARDCODED SEQUENCE ---
        self.forward_sequence = "fflrffffrlfffrlfrlfffffllllfllfllffffffrlfrlffffffrlrrrrrrffffrlfrlfrlf"
        
        # Automatically generate the backtrack sequence
        self.backtrack_sequence = self.generate_backtrack(self.forward_sequence)
        
        # --- TIMING CONFIGURATION ---
        self.tick_rate_seconds = 0.05  # 50 milliseconds per character
        
        # State machine tracking
        self.current_index = 0
        self.state = 'FORWARD' # States: FORWARD, PAUSE, BACKTRACK, DONE
        self.pause_ticks = int(2.0 / self.tick_rate_seconds) # Pause for 2 seconds (40 ticks)
        
        self.timer = self.create_timer(self.tick_rate_seconds, self.timer_callback)
        
        self.get_logger().info("--- Starting Auto-Backtrack Mission ---")
        self.get_logger().info(f"Forward Path:   {self.forward_sequence}")
        self.get_logger().info(f"Backtrack Path: {self.backtrack_sequence}")

    def generate_backtrack(self, sequence):
        """Reverses the string and inverts the directional commands"""
        inverse_map = {'f': 'b', 'b': 'f', 'l': 'r', 'r': 'l', 's': 's'}
        backtrack_seq = ""
        
        # Read the sequence backwards and swap the letters
        for char in reversed(sequence):
            backtrack_seq += inverse_map.get(char, 's')
            
        return backtrack_seq

    def timer_callback(self):
        msg = String()
        
        if self.state == 'FORWARD':
            if self.current_index < len(self.forward_sequence):
                msg.data = self.forward_sequence[self.current_index]
                self.publisher.publish(msg)
                self.current_index += 1
            else:
                self.get_logger().info("Reached destination. Pausing for 2 seconds...")
                self.state = 'PAUSE'
                
        elif self.state == 'PAUSE':
            msg.data = 's'
            self.publisher.publish(msg)
            self.pause_ticks -= 1
            
            if self.pause_ticks <= 0:
                self.get_logger().info("Heading home! Starting backtrack...")
                self.state = 'BACKTRACK'
                self.current_index = 0 # Reset index for the return trip
                
        elif self.state == 'BACKTRACK':
            if self.current_index < len(self.backtrack_sequence):
                msg.data = self.backtrack_sequence[self.current_index]
                self.publisher.publish(msg)
                self.current_index += 1
            else:
                self.get_logger().info("Mission Complete. Rover returned to base.")
                self.state = 'DONE'
                
        elif self.state == 'DONE':
            msg.data = 's'
            self.publisher.publish(msg)
            self.timer.cancel() # Stop the timer

def main(args=None):
    rclpy.init(args=args)
    node = HardcodedPathNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Path execution aborted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()