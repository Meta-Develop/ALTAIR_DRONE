import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
import time

class PicoVerifier(Node):
    def __init__(self):
        super().__init__('pico_verifier')
        
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/pico/esc_telemetry',
            self.callback,
            qos
        )
        self.last_time = time.time()
        self.count = 0
        self.get_logger().info("Verifier Started. Waiting for Pico data...")

    def callback(self, msg):
        self.count += 1
        now = time.time()
        if now - self.last_time > 1.0:
            rate = self.count / (now - self.last_time)
            self.get_logger().info(f"Receiving Data! Rate: {rate:.1f} Hz | Sample: {msg.data}")
            self.count = 0
            self.last_time = now

def main():
    rclpy.init()
    node = PicoVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
