import rclpy, serial, time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PicoBridge(Node):
    def __init__(self):
        super().__init__('pico_bridge')
        self.get_logger().info('PicoBridge node starting...')

        # params (adjust port if using USB instead of UART)
        self.declare_parameter('port', '/dev/ttyTHS1')  # Nano header UART2
        self.declare_parameter('baud', 115200)
        self.declare_parameter('scale', 1.0)            # cap max speed (0..1)
        self.declare_parameter('rate', 30.0)            # send Hz

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.scale = float(self.get_parameter('scale').value)
        period = 1.0 / float(self.get_parameter('rate').value)

        self.get_logger().info(f'Parameters: port={port}, baud={baud}, scale={self.scale}, rate={1.0/period}Hz')

        try:
            self.get_logger().info(f'Attempting to connect to serial port {port}...')
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
            time.sleep(0.2)
            self.get_logger().info(f'Connected to Pico on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {port}: {e}')
            raise

        self.left = 0.0
        self.right = 0.0

        self.get_logger().info('Creating /cmd_vel subscription...')
        self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.get_logger().info('Creating timer for motor commands...')
        self.timer = self.create_timer(period, self.send)
        self.get_logger().info('PicoBridge initialization complete')

    def on_twist(self, msg: Twist):
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)
        self.get_logger().debug(f'Received cmd_vel: linear.x={lin}, angular.z={ang}')
        
        L = lin - ang
        R = lin + ang
        m = max(1.0, abs(L), abs(R))
        self.left  = max(-1.0, min(1.0, (L/m)*self.scale))
        self.right = max(-1.0, min(1.0, (R/m)*self.scale))
        
        self.get_logger().debug(f'Motor commands: left={self.left:.3f}, right={self.right:.3f}')

    def send(self):
        try:
            cmd = f"L={self.left:.3f} R={self.right:.3f}\n"
            self.ser.write(cmd.encode('ascii'))
            self.get_logger().debug(f'Sent to Pico: {cmd.strip()}')
        except Exception as e:
            self.get_logger().warn(f'serial write failed: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down PicoBridge node...')
        try:
            if getattr(self, "ser", None) and self.ser.is_open:
                self.get_logger().info('Sending stop command to Pico and closing serial port')
                self.ser.write(b"L=0 R=0\n")
                self.ser.close()
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
        finally:
            self.get_logger().info('PicoBridge shutdown complete')
            return super().destroy_node()

def main():
    print('Starting PicoBridge main...')
    rclpy.init()
    print('ROS2 initialized')
    try:
        n = PicoBridge()
        print('PicoBridge node created, spinning...')
        rclpy.spin(n)
    except KeyboardInterrupt:
        print('Received interrupt signal')
    except Exception as e:
        print(f'Exception in main: {e}')
    finally:
        print('Cleaning up...')
        if 'n' in locals():
            n.destroy_node()
        rclpy.shutdown()
        print('PicoBridge main complete')

if __name__ == '__main__':
    main()
