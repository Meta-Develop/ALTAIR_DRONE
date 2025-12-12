import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import time

from rclpy.qos import qos_profile_sensor_data
import rclpy.qos

class SystemIntegrationTest(Node):
    def __init__(self):
        super().__init__('system_integration_test')
        
        # Publisher
        self.pub_control = self.create_publisher(Float32MultiArray, '/control/actuator_commands', 10)
        
        # QoS Profile for Sensors (Best Effort)
        qos_sensor = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(Imu, '/imu/filtered', self.imu_callback, qos_sensor)
        self.create_subscription(Float32MultiArray, '/pico/esc_telemetry', self.esc_callback, qos_sensor)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.start_time = time.time()
        
        self.imu_count = 0
        self.esc_count = 0
        self.odom_count = 0
        self.last_print = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time
        
        # Generate Dummy Control (Sine Waves)
        msg = Float32MultiArray()
        # 6 Motors (0.1 to 0.9) - Keep it safe but moving
        motors = [0.5 + 0.4 * math.sin(t * 2.0 + i) for i in range(6)]
        # 6 Servos (-0.5 to 0.5 rad) - Gentle sweep
        servos = [0.5 * math.sin(t * 1.0 + i) for i in range(6)]
        
        msg.data = motors + servos
        self.pub_control.publish(msg)
        
        # Print Status every 1s
        if time.time() - self.last_print >= 1.0:
            self.get_logger().info(f"--- System Status ---")
            self.get_logger().info(f"Rates | IMU: {self.imu_count} Hz | ESC: {self.esc_count} Hz | Odom: {self.odom_count} Hz")
            
            # Check Data Quality if available
            if hasattr(self, 'last_imu_msg'):
                az = self.last_imu_msg.linear_acceleration.z
                self.get_logger().info(f"Last IMU Accel Z: {az:.2f} m/s^2 (Target: ~9.81)")
            
            if hasattr(self, 'last_odom_msg'):
                pos_z = self.last_odom_msg.pose.pose.position.z
                self.get_logger().info(f"Last Odom Pos Z: {pos_z:.2f} m")

            # Reset counters
            self.imu_count = 0
            self.esc_count = 0
            self.odom_count = 0
            self.last_print = time.time()

    def imu_callback(self, msg):
        self.imu_count += 1
        self.last_imu_msg = msg
        
    def esc_callback(self, msg):
        self.esc_count += 1
        
    def odom_callback(self, msg):
        self.odom_count += 1
        self.last_odom_msg = msg

def main(args=None):
    rclpy.init(args=args)
    node = SystemIntegrationTest()
    print("Starting System Integration Test...")
    print("Publishing to /control/actuator_commands")
    print("Listening to /pico/imu_raw, /pico/esc_telemetry, /odometry/filtered")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
