import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class BusWatchdog(Node):
    def __init__(self):
        super().__init__('system_monitor')

        # QoS - SensorData (Best Effort)
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # QoS - Reliable
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_imu = self.create_subscription(
            Imu, '/pico/imu_raw', self.imu_callback, qos_sensor)
        
        self.sub_actuator = self.create_subscription(
            Float32MultiArray, '/control/actuator_commands', self.actuator_callback, qos_sensor)

        self.sub_esc = self.create_subscription(
            Float32MultiArray, '/pico/esc_telemetry', self.esc_callback, qos_sensor)

        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, qos_sensor)

        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_reliable)

        self.sub_override = self.create_subscription(
            Float32MultiArray, '/control/manual_override', self.override_callback, qos_reliable)

        # State
        self.counts = {
            'imu': 0,
            'actuator': 0,
            'esc': 0,
            'odom': 0,
            'cmd_vel': 0,
            'override': 0
        }
        self.last_time = time.time()
        
        self.get_logger().info("Bus Watchdog Started - Monitoring All Topics")

    def imu_callback(self, msg):
        self.counts['imu'] += 1

    def actuator_callback(self, msg):
        self.counts['actuator'] += 1

    def esc_callback(self, msg):
        self.counts['esc'] += 1

    def odom_callback(self, msg):
        self.counts['odom'] += 1

    def cmd_vel_callback(self, msg):
        self.counts['cmd_vel'] += 1

    def override_callback(self, msg):
        self.counts['override'] += 1

    def check_freq(self):
        now = time.time()
        dt = now - self.last_time
        
        if dt >= 1.0:
            freqs = {k: v / dt for k, v in self.counts.items()}
            
            # Reset
            for k in self.counts:
                self.counts[k] = 0
            self.last_time = now

            # Log
            log_msg = " | ".join([f"{k.upper()}: {v:.1f}Hz" for k, v in freqs.items()])
            self.get_logger().info(log_msg)

            # Alerts
            if freqs['imu'] < 380.0:
                self.get_logger().warn(f"LOW IMU FREQUENCY: {freqs['imu']:.2f} Hz (Threshold: 380 Hz)")
            
            if freqs['esc'] > 0 and freqs['esc'] < 10.0:
                 self.get_logger().warn(f"LOW ESC TELEMETRY: {freqs['esc']:.2f} Hz")

        # Re-schedule
        self.timer = self.create_timer(1.0, self.check_freq)

def main(args=None):
    rclpy.init(args=args)
    node = BusWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
