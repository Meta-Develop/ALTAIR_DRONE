import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QTabWidget, QLabel, QSlider, QCheckBox, 
                             QProgressBar, QGridLayout, QGroupBox)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
import math

# --- Main GCS Node ---
class AltairGCSNode(Node):
    def __init__(self, worker_signals):
        super().__init__('altair_gcs_gui')
        self.worker_signals = worker_signals

        # QoS Profiles
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_reliable)
        self.pub_manual_override = self.create_publisher(Float32MultiArray, '/control/manual_override', qos_reliable)

        # Subscribers
        self.sub_esc_telemetry = self.create_subscription(
            Float32MultiArray, '/pico/esc_telemetry', self.esc_telemetry_callback, qos_sensor)
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, qos_sensor)

        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, qos_sensor) # Joy is usually sensor data? or Reliable? Usually SensorData or System default.

        # Internal State
        self.manual_override_active = False
        self.override_values = [0.0] * 12 # 6 Motors, 6 Servos

        self.get_logger().info("ALTAIR GCS Node Started")

    def esc_telemetry_callback(self, msg):
        # Msg data expected to be 6 floats (RPMs)
        if len(msg.data) >= 6:
            self.worker_signals.telemetry_signal.emit(list(msg.data))

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.worker_signals.odometry_signal.emit(pos.x, pos.y, pos.z)

        # Calculate Attitude (Roll, Pitch, Yaw) from Quaternion
        q = msg.pose.pose.orientation
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Emit degrees
        self.worker_signals.attitude_signal.emit(
            math.degrees(roll), 
            math.degrees(pitch), 
            math.degrees(yaw)
        )

    def joy_callback(self, msg):
        # Map joy to cmd_vel
        # Simple mapping: Axis 1->LinX, Axis 0->LinY, Axis 3->AngZ, Axis 4->LinZ (Example)
        # Also update GUI indicators
        self.worker_signals.joy_signal.emit(list(msg.axes), list(msg.buttons))
        
        if not self.manual_override_active:
            twist = Twist()
            # Basic mapping assumption (Xbox controller standard)
            # Left Stick Y (1) -> Linear X
            # Left Stick X (0) -> Linear Y
            # Right Stick X (3) -> Angular Z
            # Right Stick Y (4) -> Linear Z (Throttle-ish?) or Buttons?
            # Let's assume simple 4-axis control
            if len(msg.axes) >= 4:
                twist.linear.x = msg.axes[1] * 2.0 # Scale factor
                twist.linear.y = msg.axes[0] * 2.0 
                twist.angular.z = msg.axes[3] * 2.0
                if len(msg.axes) > 4:
                    twist.linear.z = msg.axes[4] * 2.0
            
            self.pub_cmd_vel.publish(twist)

    def publish_override(self):
        if self.manual_override_active:
            msg = Float32MultiArray()
            msg.data = self.override_values
            self.pub_manual_override.publish(msg)

# --- Main Window ---
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ALTAIR Hexacopter GCS")
        self.resize(800, 600)

        # Tabs
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Init Tabs
        self.init_flight_tab()
        self.init_maintenance_tab()

        # Timer for publishing override (10Hz)
        self.override_timer = QTimer()
        self.override_timer.timeout.connect(self.publish_override_timer)
        self.override_timer.start(100)

    def init_flight_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Indicators
        self.lbl_pos = QLabel("Position: X=0.00, Y=0.00, Z=0.00")
        self.lbl_att = QLabel("Attitude: R=0.00, P=0.00, Y=0.00")
        self.lbl_battery = QLabel("Battery: N/A") # Placeholder
        
        # Joy Indicators (Progress bars for axes)
        joy_group = QGroupBox("Joystick Input")
        joy_layout = QGridLayout()
        self.joy_bars = []
        for i in range(4):
            lbl = QLabel(f"Axis {i}")
            bar = QProgressBar()
            bar.setRange(-100, 100)
            bar.setValue(0)
            bar.setFormat("%v")
            joy_layout.addWidget(lbl, i, 0)
            joy_layout.addWidget(bar, i, 1)
            self.joy_bars.append(bar)
        joy_group.setLayout(joy_layout)

        layout.addWidget(self.lbl_pos)
        layout.addWidget(self.lbl_att)
        layout.addWidget(self.lbl_battery)
        layout.addWidget(joy_group)
        layout.addStretch()
        tab.setLayout(layout)
        self.tabs.addTab(tab, "Flight Control")

    def init_maintenance_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Toggle
        self.chk_override = QCheckBox("Enable Manual Override")
        self.chk_override.toggled.connect(self.toggle_override)
        layout.addWidget(self.chk_override)

        # Sliders
        sliders_layout = QGridLayout()
        self.sliders = []
        self.rpm_bars = []

        # 6 Motors
        for i in range(6):
            lbl = QLabel(f"Motor {i+1}")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000) # Assume 0-1000 control range? Or 0-1 float?
            # Let's assume 0-100 for UI, mapped to whatever the protocol needs. 
            # Actuator commands are Float32. Let's assume 0.0 to 1.0 or similar.
            # I'll use 0-100 in slider and divide by 100.0
            slider.valueChanged.connect(lambda val, idx=i: self.update_slider(idx, val))
            
            bar = QProgressBar() # RPM feedback
            bar.setRange(0, 10000) # Example Max RPM
            
            sliders_layout.addWidget(lbl, i, 0)
            sliders_layout.addWidget(slider, i, 1)
            sliders_layout.addWidget(bar, i, 2)
            
            self.sliders.append(slider)
            self.rpm_bars.append(bar)

        # 6 Servos
        for i in range(6):
            lbl = QLabel(f"Servo {i+1}")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100) # 0-100% (e.g., angle 0-180 or -90 to 90)
            slider.valueChanged.connect(lambda val, idx=i+6: self.update_slider(idx, val))
            
            sliders_layout.addWidget(lbl, i+6, 0)
            sliders_layout.addWidget(slider, i+6, 1)
            
            self.sliders.append(slider)

        layout.addLayout(sliders_layout)
        tab.setLayout(layout)
        self.tabs.addTab(tab, "Maintenance")

    def toggle_override(self, checked):
        self.node.manual_override_active = checked
        # Reset sliders if unchecked? Or keep last?
        # Let's keep last.

    def update_slider(self, index, value):
        # Update node internal state
        # Map 0-100 (or 1000) to float. 
        # Assuming direct mapping for now.
        # Let's say motors are 0-1.0 and servos are -1.0 to 1.0? 
        # Without spec, I'll just pass the raw slider value divided by range to normalize if needed, 
        # but Float32MultiArray might expect raw PWM.
        # I'll pass the integer value as float for now.
        self.node.override_values[index] = float(value)

    def publish_override_timer(self):
        self.node.publish_override()

    def update_telemetry(self, data):
        # Data is list of 6 floats (RPMs)
        for i, rpm in enumerate(data):
            if i < len(self.rpm_bars):
                self.rpm_bars[i].setValue(int(rpm))

    def update_odometry(self, x, y, z):
        self.lbl_pos.setText(f"Position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

    def update_attitude(self, r, p, y):
        self.lbl_att.setText(f"Attitude: R={r:.2f}, P={p:.2f}, Y={y:.2f}")

    def update_joy(self, axes, buttons):
        # Update Joy bars
        for i, bar in enumerate(self.joy_bars):
            if i < len(axes):
                bar.setValue(int(axes[i] * 100))

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    
    # Worker Signals
    # Better: Create signals object, pass to Node.
    
    class Signals(QObject):
        telemetry_signal = pyqtSignal(list)
        odometry_signal = pyqtSignal(float, float, float)
        attitude_signal = pyqtSignal(float, float, float)
        joy_signal = pyqtSignal(list, list)
    
    signals = Signals()
    
    node = AltairGCSNode(signals)
    window = MainWindow(node)
    
    # Connect signals
    signals.telemetry_signal.connect(window.update_telemetry)
    signals.odometry_signal.connect(window.update_odometry)
    signals.attitude_signal.connect(window.update_attitude)
    signals.joy_signal.connect(window.update_joy)
    
    # Thread for ROS spin
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
