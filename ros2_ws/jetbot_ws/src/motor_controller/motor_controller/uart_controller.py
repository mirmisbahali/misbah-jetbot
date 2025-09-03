#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class UART_Comms(Node): 
    def __init__(self):
        super().__init__("uart_comms") # Node Name
        
        # Initialize UART connection to Pico (pins 8,10 on Jetson Nano)
        try:
            self.uart_connection = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)
            self.get_logger().info("UART connection established on /dev/ttyTHS1")
        except Exception as e:
            self.get_logger().error(f"Failed to establish UART connection: {e}")
            self.uart_connection = None
        
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
        
        # Send velocity data to Pico via UART
        if self.uart_connection:
            try:
                # Format velocity data as string
                velocity_data = f"Linear: x={msg.linear.x:.3f} y={msg.linear.y:.3f} z={msg.linear.z:.3f}, Angular: x={msg.angular.x:.3f} y={msg.angular.y:.3f} z={msg.angular.z:.3f}\n"
                self.uart_connection.write(velocity_data.encode())
            except Exception as e:
                self.get_logger().error(f"UART write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UART_Comms() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        if hasattr(node, 'uart_connection') and node.uart_connection:
            node.uart_connection.close()
            node.get_logger().info("UART connection closed")
        rclpy.shutdown()


if __name__ == "__main__":
    main()


