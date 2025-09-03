#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class UART_Comms(Node): 
    def __init__(self):
        super().__init__("uart_comms") # Node Name


def main(args=None):
    rclpy.init(args=args)
    node = UART_Comms() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


