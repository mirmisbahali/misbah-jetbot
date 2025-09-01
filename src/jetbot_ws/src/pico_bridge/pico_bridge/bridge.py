import rclpy, serial, time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PicoBridge(Node):
    def __init__(self):
        super().__init__('pico_bridge')

        # params (adjust port if using USB instead of UART)
        self.declare_parameter('port', '/dev/ttyTHS1')  # Nano header UART2
        self.declare_parameter('baud', 115200)
        self.declare_parameter('scale', 1.0)            # cap max speed (0..1)
        self.declare_parameter('rate', 30.0)            # send Hz

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.scale = float(self.get_parameter('scale').value)
        period = 1.0 / float(self.get_parameter('rate').value)

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
            time.sleep(0.2)
            self.get_logger().info(f'Connected to Pico on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open {port}: {e}')
            raise

        self.left = 0.0
        self.right = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)
        self.timer = self.create_timer(period, self.send)

    def on_twist(self, msg: Twist):
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)
        L = lin - ang
        R = lin + ang
        m = max(1.0, abs(L), abs(R))
        self.left  = max(-1.0, min(1.0, (L/m)*self.scale))
        self.right = max(-1.0, min(1.0, (R/m)*self.scale))

    def send(self):
        try:
            cmd = f"L={self.left:.3f} R={self.right:.3f}\n"
            self.ser.write(cmd.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f'serial write failed: {e}')

    def destroy_node(self):
        try:
            if getattr(self, "ser", None) and self.ser.is_open:
                self.ser.write(b"L=0 R=0\n")
                self.ser.close()
        finally:
            return super().destroy_node()

def main():
    rclpy.init()
    n = PicoBridge()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
