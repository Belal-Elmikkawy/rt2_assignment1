#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_action_pkg.msg import UserCommand

class UserInterfaceNode(Node):
    """
    Provides a command-line interface for the user to input target coordinates
    or cancel ongoing navigation actions.
    """
    def __init__(self):
        super().__init__('user_interface_node')

        # Setup publisher to broadcast user commands to the action client
        self.publisher_ = self.create_publisher(UserCommand, 'user_command_topic', 10)
        self.get_logger().info("User Interface Node started. Ready for commands.")

    def run_input_loop(self):
        """Runs a blocking loop to capture keyboard input."""

        # Keep asking for input as long as the ROS context is valid
        while rclpy.ok():
            user_input = input("\nEnter target (x, y, theta) separated by spaces, or type 'cancel': ").strip().lower()

            # Initialize an empty custom message
            msg = UserCommand()

            # Handle goal cancellation requests
            if user_input == 'cancel':
                msg.is_cancel = True
                self.publisher_.publish(msg)
                self.get_logger().info("Cancel command sent to the Action Client.")

            # Handle new target coordinate requests
            else:
                try:
                    # Parse the input string into individual x, y, theta components
                    parts = user_input.split()

                    # Ensure the user provided exactly three values
                    if len(parts) != 3:
                        raise ValueError("Expected exactly 3 values.")

                    # Convert input strings to floating point numbers
                    msg.x = float(parts[0])
                    msg.y = float(parts[1])
                    msg.theta = float(parts[2])
                    msg.is_cancel = False

                    # Broadcast the new target coordinates
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Target sent: x={msg.x}, y={msg.y}, theta={msg.theta}")

                # Catch invalid input formats like letters or missing coordinates
                except ValueError:
                    self.get_logger().warning("Invalid input. Please enter 3 numbers (e.g., '1.5 2.0 0.0') or 'cancel'.")

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Instantiate the user interface node
    node = UserInterfaceNode()

    # We handle the loop manually inside run_input_loop so input() doesn't block callbacks.
    # We don't need rclpy.spin(node) since this node only publishes.
    try:
        node.run_input_loop()
    except KeyboardInterrupt:
        # Gracefully handle the user pressing Ctrl+C
        node.get_logger().info("Keyboard interrupt, shutting down UI...")
    finally:
        # Cleanup node and ROS context upon exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
