import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import time

class TelemetryVerifier(Node):
    def __init__(self):
        super().__init__('telemetry_verifier')
        
        # QoS Profile: Best Effort (SensorData) to match Pico
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/pico/imu_raw',
            self.imu_callback,
            qos_policy
        )
        
        self.esc_sub = self.create_subscription(
            Float32MultiArray,
            '/pico/esc_telemetry',
            self.esc_callback,
            qos_policy
        )

        self.imu_count = 0
        self.esc_count = 0
        self.start_time = time.time()
        
        self.get_logger().info("Waiting for telemetry data...")

    def imu_callback(self, msg):
        self.imu_count += 1
        if self.imu_count % 10 == 0: # Print every 10th message
            self.get_logger().info(f"IMU [{self.imu_count}]: Accel({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})")

    def esc_callback(self, msg):
        self.esc_count += 1
        if self.esc_count % 10 == 0:
            self.get_logger().info(f"ESC [{self.esc_count}]: RPMs({msg.data})")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryVerifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
