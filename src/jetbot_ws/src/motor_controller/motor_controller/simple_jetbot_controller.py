#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time


class SimpleJetbotController(Node):
    def __init__(self):
        super().__init__('simple_jetbot_controller')
        
        # MX1508 Motor Driver GPIO pins
        # Left motor
        self.left_motor_pin1 = 32  # IN1 for left motor
        self.left_motor_pin2 = 35  # IN2 for left motor
        
        # Right motor  
        self.right_motor_pin1 = 33  # IN1 for right motor
        self.right_motor_pin2 = 37  # IN2 for right motor
        
        # Setup GPIO
        self.setup_gpio()
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Simple Jetbot Controller started')
        self.get_logger().info(f'GPIO pins - Left motor: {self.left_motor_pin1},{self.left_motor_pin2} Right motor: {self.right_motor_pin1},{self.right_motor_pin2}')
        
    def setup_gpio(self):
        """Initialize GPIO pins for motor control"""
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.left_motor_pin1, GPIO.OUT)
            GPIO.setup(self.left_motor_pin2, GPIO.OUT) 
            GPIO.setup(self.right_motor_pin1, GPIO.OUT)
            GPIO.setup(self.right_motor_pin2, GPIO.OUT)
            
            # Initialize all pins to LOW (motors stopped)
            self.stop_motors()
            
            self.get_logger().info('GPIO setup completed')
            
        except Exception as e:
            self.get_logger().error(f'GPIO setup failed: {str(e)}')
            
    def stop_motors(self):
        """Stop both motors"""
        GPIO.output(self.left_motor_pin1, GPIO.LOW)
        GPIO.output(self.left_motor_pin2, GPIO.LOW)
        GPIO.output(self.right_motor_pin1, GPIO.LOW)
        GPIO.output(self.right_motor_pin2, GPIO.LOW)
        
    def left_motor_forward(self):
        """Move left motor forward"""
        GPIO.output(self.left_motor_pin1, GPIO.HIGH)
        GPIO.output(self.left_motor_pin2, GPIO.LOW)
        
    def left_motor_backward(self):
        """Move left motor backward"""
        GPIO.output(self.left_motor_pin1, GPIO.LOW)
        GPIO.output(self.left_motor_pin2, GPIO.HIGH)
        
    def right_motor_forward(self):
        """Move right motor forward"""
        GPIO.output(self.right_motor_pin1, GPIO.HIGH)
        GPIO.output(self.right_motor_pin2, GPIO.LOW)
        
    def right_motor_backward(self):
        """Move right motor backward"""
        GPIO.output(self.right_motor_pin1, GPIO.LOW)
        GPIO.output(self.right_motor_pin2, GPIO.HIGH)
        
    def cmd_vel_callback(self, msg):
        """Process cmd_vel messages and control motors"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Thresholds for detecting movement commands
        linear_threshold = 0.01
        angular_threshold = 0.01
        
        # Debug logging - show all received values
        self.get_logger().info(f'Received cmd_vel: linear_x={linear_x:.4f}, angular_z={angular_z:.4f}')
        self.get_logger().info(f'Thresholds: linear={linear_threshold}, angular={angular_threshold}')
        
        # Stop motors first
        self.stop_motors()
        
        # Check for forward movement
        if linear_x > linear_threshold:
            self.get_logger().info(f'FORWARD: linear_x({linear_x:.4f}) > threshold({linear_threshold})')
            self.left_motor_forward()
            self.right_motor_forward()
            
        # Check for backward movement  
        elif linear_x < -linear_threshold:
            self.get_logger().info(f'BACKWARD: linear_x({linear_x:.4f}) < -threshold({-linear_threshold})')
            self.left_motor_backward()
            self.right_motor_backward()
            
        # Check for right turn (negative angular_z)
        elif angular_z < -angular_threshold:
            self.get_logger().info(f'RIGHT TURN: angular_z({angular_z:.4f}) < -threshold({-angular_threshold})')
            self.left_motor_forward()
            self.right_motor_backward()
            
        # Check for left turn (positive angular_z)
        elif angular_z > angular_threshold:
            self.get_logger().info(f'LEFT TURN: angular_z({angular_z:.4f}) > threshold({angular_threshold})')
            self.left_motor_backward()
            self.right_motor_forward()
            
        else:
            # Stop if no significant movement command
            self.get_logger().info(f'STOP: No significant movement - linear_x={linear_x:.4f}, angular_z={angular_z:.4f}')
            self.stop_motors()
            
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.get_logger().info('Cleaning up GPIO')
        self.stop_motors()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleJetbotController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in simple jetbot controller: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()