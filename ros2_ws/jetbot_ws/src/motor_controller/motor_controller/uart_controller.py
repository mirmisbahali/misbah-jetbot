#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class UART_Comms(Node): 
    def __init__(self):
        super().__init__("uart_comms") # Node Name
        
        # Create subscriber to /cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info("UART Communications node started, subscribing to /cmd_vel")
    
    def cmd_vel_callback(self, msg):
        # Log all Twist message values
        self.get_logger().info(f"Linear velocity - x: {msg.linear.x:.3f}, y: {msg.linear.y:.3f}, z: {msg.linear.z:.3f}")
        self.get_logger().info(f"Angular velocity - x: {msg.angular.x:.3f}, y: {msg.angular.y:.3f}, z: {msg.angular.z:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = UART_Comms() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


