#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
#from nav_action_pkg.msg import UserCommand 

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('user_interface_node')
        # Publisher for the UI commands
        self.publisher_ = self.create_publisher(UserCommand, 'user_command_topic', 10)
        self.get_logger().info("User Interface Node started. Ready for commands.")

    def run_input_loop(self):
        """Runs a blocking loop to capture keyboard input."""
        while rclpy.ok():
            user_input = input("\nEnter target (x, y, theta) separated by spaces, or type 'cancel': ").strip().lower()

            msg = UserCommand()

            if user_input == 'cancel':
                msg.is_cancel = True
                self.publisher_.publish(msg)
                self.get_logger().info("Cancel command sent to the Action Client.")
            else:
                try:
                    # Parse x, y, theta
                    parts = user_input.split()
                    if len(parts) != 3:
                        raise ValueError("Expected exactly 3 values.")
                    
                    msg.x = float(parts[0])
                    msg.y = float(parts[1])
                    msg.theta = float(parts[2])
                    msg.is_cancel = False
                    
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Target sent: x={msg.x}, y={msg.y}, theta={msg.theta}")
                
                except ValueError:
                    self.get_logger().warning("Invalid input. Please enter 3 numbers (e.g., '1.5 2.0 0.0') or 'cancel'.")

def main(args=None):
    rclpy.init(args=args)
    node = UserInterfaceNode()
    
    # We handle the loop manually inside run_input_loop so input() doesn't block callbacks.
    # We don't need rclpy.spin(node) since this node only publishes.
    try:
        node.run_input_loop()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down UI...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()