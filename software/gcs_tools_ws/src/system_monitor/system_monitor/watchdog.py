import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
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

        # Subscribers
        self.sub_imu = self.create_subscription(
            Imu, '/pico/imu_raw', self.imu_callback, qos_sensor)
        
        self.sub_actuator = self.create_subscription(
            Float32MultiArray, '/control/actuator_commands', self.actuator_callback, qos_sensor)

        # State
        self.imu_count = 0
        self.imu_last_time = time.time()
        self.imu_freq = 0.0
        
        self.act_count = 0
        self.act_last_time = time.time()
        self.act_freq = 0.0

        # Timer for checking/printing stats (1Hz)
        self.timer = self.create_timer(1.0, self.check_freq)
        
        self.get_logger().info("Bus Watchdog Started")

    def imu_callback(self, msg):
        self.imu_count += 1

    def actuator_callback(self, msg):
        self.act_count += 1

    def check_freq(self):
        now = time.time()
        
        # Calculate IMU Freq
        dt_imu = now - self.imu_last_time
        if dt_imu > 0:
            self.imu_freq = self.imu_count / dt_imu
            self.imu_count = 0
            self.imu_last_time = now

        # Calculate Actuator Freq
        dt_act = now - self.act_last_time
        if dt_act > 0:
            self.act_freq = self.act_count / dt_act
            self.act_count = 0
            self.act_last_time = now

        # Log
        self.get_logger().info(f"IMU: {self.imu_freq:.2f} Hz | Actuators: {self.act_freq:.2f} Hz")

        # Alerts
        if self.imu_freq < 380.0:
            self.get_logger().warn(f"LOW IMU FREQUENCY: {self.imu_freq:.2f} Hz (Threshold: 380 Hz)")

        # Example threshold for actuators (assuming say 100Hz or similar, not specified but usually high)
        # If not specified, I won't warn, but I'll just log it.

def main(args=None):
    rclpy.init(args=args)
    node = BusWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
