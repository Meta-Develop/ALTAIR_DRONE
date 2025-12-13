import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QTabWidget, QLabel, QSlider, QCheckBox, 
                             QProgressBar, QGridLayout, QGroupBox, QPushButton, QFrame, QStyleFactory, QTextEdit as pyqtTextEdit)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer
from PyQt5.QtGui import QColor, QPalette, QFont

from std_msgs.msg import Float32MultiArray, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry
import math
from .remote_manager import RemoteManager
from .attitude_indicator import AttitudeIndicator



# --- Main Window ---
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ALTAIR Hexacopter GCS")
        self.resize(1000, 700)

        # Remote Manager
        self.remote_manager = RemoteManager(self.node.get_logger())

        # Apply Dark Theme
        self.apply_dark_theme()

        # Tabs
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Init Tabs
        self.init_flight_tab()
        self.init_maintenance_tab()
        self.init_system_tab()
        self.init_monitor_tab()

        # Timer for publishing override (10Hz)
        self.override_timer = QTimer()
        self.override_timer.timeout.connect(self.publish_override_timer)
        self.override_timer.start(100)

    # ... (apply_dark_theme)

    def init_flight_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Top Section: Status & Modes
        top_layout = QHBoxLayout()
        
        # Left Partition: Status & Mode
        left_layout = QVBoxLayout()

        # Status Group
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout()
        
        self.lbl_pos = QLabel("Position: X=0.00, Y=0.00, Z=0.00")
        self.lbl_pos.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_att = QLabel("Attitude: R=0.00, P=0.00, Y=0.00")
        self.lbl_att.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_imu_raw = QLabel("IMU Raw: Ax=0.0, Ay=0.0, Az=0.0")
        self.lbl_imu_raw.setStyleSheet("color: #42A5F5;")
        self.lbl_battery = QLabel("Battery: N/A")
        self.lbl_battery.setStyleSheet("color: orange;")

        status_layout.addWidget(self.lbl_pos)
        status_layout.addWidget(self.lbl_att)
        status_layout.addWidget(self.lbl_imu_raw)
        status_layout.addWidget(self.lbl_battery)
        status_group.setLayout(status_layout)
        
        # Mode Control Group
        mode_group = QGroupBox("Mode Selection")
        mode_layout = QVBoxLayout()
        
        mode_btn_layout = QHBoxLayout()
        self.btn_test_mode = QPushButton("TEST MODE (Safe)")
        self.btn_test_mode.setStyleSheet("background-color: #2E7D32; color: white; padding: 10px;")
        self.btn_test_mode.clicked.connect(lambda: self.node.set_pico_mode(0))
        
        self.btn_actual_mode = QPushButton("ACTUAL MODE (Armed)")
        self.btn_actual_mode.setStyleSheet("background-color: #C62828; color: white; padding: 10px;")
        self.btn_actual_mode.clicked.connect(lambda: self.node.set_pico_mode(1))
        
        mode_btn_layout.addWidget(self.btn_test_mode)
        mode_btn_layout.addWidget(self.btn_actual_mode)
        
        mode_layout.addLayout(mode_btn_layout)
        mode_layout.addWidget(QLabel("Control Mode:"))
        
        ctrl_btn_layout = QHBoxLayout()
        self.btn_pid = QPushButton("PID Control")
        self.btn_pid.clicked.connect(lambda: self.node.set_control_mode(0))
        self.btn_nmpc = QPushButton("NMPC Control")
        self.btn_nmpc.clicked.connect(lambda: self.node.set_control_mode(1))
        
        ctrl_btn_layout.addWidget(self.btn_pid)
        ctrl_btn_layout.addWidget(self.btn_nmpc)
        mode_layout.addLayout(ctrl_btn_layout)
        mode_group.setLayout(mode_layout)

        left_layout.addWidget(status_group)
        left_layout.addWidget(mode_group)
        
        # Right Partition: Attitude Indicator
        right_layout = QVBoxLayout()
        att_group = QGroupBox("Artificial Horizon")
        att_layout = QVBoxLayout()
        self.attitude_indicator = AttitudeIndicator()
        att_layout.addWidget(self.attitude_indicator)
        att_group.setLayout(att_layout)
        right_layout.addWidget(att_group)

        top_layout.addLayout(left_layout, 60)
        top_layout.addLayout(right_layout, 40)
        layout.addLayout(top_layout)

        # Joy Indicators
        joy_group = QGroupBox("Joystick Input")
        joy_layout = QGridLayout()
        # ... (joy code same)


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
        self.pub_pico_mode = self.create_publisher(Int8, '/pico/mode', qos_reliable)
        self.pub_control_mode = self.create_publisher(Int8, '/control/mode', qos_reliable)

        # Subscribers
        self.sub_esc_telemetry = self.create_subscription(
            Float32MultiArray, '/pico/esc_telemetry', self.esc_telemetry_callback, qos_sensor)
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/predicted', self.odom_callback, qos_sensor)

        self.sub_joy = self.create_subscription(
            Joy, '/joy', self.joy_callback, qos_sensor)
            
        self.sub_imu_raw = self.create_subscription(
            Imu, '/imu/filtered', self.imu_raw_callback, qos_sensor)

        # Internal State
        self.manual_override_active = False
        self.override_values = [0.0] * 12 # 6 Motors, 6 Servos
        self.imu_packet_count = 0

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
        self.worker_signals.joy_signal.emit(list(msg.axes), list(msg.buttons))
        
        if not self.manual_override_active:
            twist = Twist()
            # Basic mapping assumption (Xbox controller standard)
            if len(msg.axes) >= 4:
                twist.linear.x = msg.axes[1] * 2.0 # Scale factor
                twist.linear.y = msg.axes[0] * 2.0 
                twist.angular.z = msg.axes[3] * 2.0
                if len(msg.axes) > 4:
                    twist.linear.z = msg.axes[4] * 2.0
            
            self.pub_cmd_vel.publish(twist)

    def imu_raw_callback(self, msg):
        # Extract linear accel and angular velocity
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        
        self.imu_packet_count += 1
        self.worker_signals.imu_signal.emit(ax, ay, az, gx, gy, gz)

    def publish_override(self):
        if self.manual_override_active:
            msg = Float32MultiArray()
            msg.data = self.override_values
            self.pub_manual_override.publish(msg)

    def set_pico_mode(self, mode):
        msg = Int8()
        msg.data = mode
        self.pub_pico_mode.publish(msg)
        self.get_logger().info(f"Published Pico Mode: {mode}")

    def set_control_mode(self, mode):
        msg = Int8()
        msg.data = mode
        self.pub_control_mode.publish(msg)
        self.get_logger().info(f"Published Control Mode: {mode}")

# --- Main Window ---
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("ALTAIR Hexacopter GCS")
        self.resize(1000, 700)

        # Remote Manager
        self.remote_manager = RemoteManager(self.node.get_logger())

        # Apply Dark Theme
        self.apply_dark_theme()

        # Tabs
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Init Tabs
        self.init_flight_tab()
        self.init_maintenance_tab()
        self.init_system_tab()
        self.init_monitor_tab()

        # Timer for publishing override (10Hz)
        self.override_timer = QTimer()
        self.override_timer.timeout.connect(self.publish_override_timer)
        self.override_timer.start(100)

    def apply_dark_theme(self):
        QApplication.setStyle(QStyleFactory.create("Fusion"))
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, Qt.black)
        QApplication.setPalette(palette)

    def init_flight_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Top Section: Status & Modes
        top_layout = QHBoxLayout()
        
        # Status Group
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout()
        
        self.lbl_pos = QLabel("Position: X=0.00, Y=0.00, Z=0.00")
        self.lbl_pos.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_att = QLabel("Attitude: R=0.00, P=0.00, Y=0.00")
        self.lbl_att.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_imu_raw = QLabel("IMU Raw: Ax=0.0, Ay=0.0, Az=0.0")
        self.lbl_imu_raw.setStyleSheet("color: #42A5F5;")
        self.lbl_battery = QLabel("Battery: N/A")
        self.lbl_battery.setStyleSheet("color: orange;")

        status_layout.addWidget(self.lbl_pos)
        status_layout.addWidget(self.lbl_att)
        status_layout.addWidget(self.lbl_imu_raw)
        status_layout.addWidget(self.lbl_battery)
        status_group.setLayout(status_layout)
        
        # Mode Control Group
        mode_group = QGroupBox("Mode Selection")
        mode_layout = QVBoxLayout()
        
        mode_btn_layout = QHBoxLayout()
        self.btn_test_mode = QPushButton("TEST MODE (Safe)")
        self.btn_test_mode.setStyleSheet("background-color: #2E7D32; color: white; padding: 10px;")
        self.btn_test_mode.clicked.connect(lambda: self.node.set_pico_mode(0))
        
        self.btn_actual_mode = QPushButton("ACTUAL MODE (Armed)")
        self.btn_actual_mode.setStyleSheet("background-color: #C62828; color: white; padding: 10px;")
        self.btn_actual_mode.clicked.connect(lambda: self.node.set_pico_mode(1))
        
        mode_btn_layout.addWidget(self.btn_test_mode)
        mode_btn_layout.addWidget(self.btn_actual_mode)
        
        mode_layout.addLayout(mode_btn_layout)
        mode_layout.addWidget(QLabel("Control Mode:"))
        
        ctrl_btn_layout = QHBoxLayout()
        self.btn_pid = QPushButton("PID Control")
        self.btn_pid.clicked.connect(lambda: self.node.set_control_mode(0))
        self.btn_nmpc = QPushButton("NMPC Control")
        self.btn_nmpc.clicked.connect(lambda: self.node.set_control_mode(1))
        
        ctrl_btn_layout.addWidget(self.btn_pid)
        ctrl_btn_layout.addWidget(self.btn_nmpc)
        mode_layout.addLayout(ctrl_btn_layout)
        
        mode_group.setLayout(mode_layout)

        top_layout.addWidget(status_group)
        top_layout.addWidget(mode_group)
        layout.addLayout(top_layout)

        # Joy Indicators
        joy_group = QGroupBox("Joystick Input")
        joy_layout = QGridLayout()
        self.joy_bars = []
        for i in range(4):
            lbl = QLabel(f"Axis {i}")
            bar = QProgressBar()
            bar.setRange(-100, 100)
            bar.setValue(0)
            bar.setFormat("%v")
            bar.setStyleSheet("QProgressBar::chunk { background-color: #42A5F5; }")
            joy_layout.addWidget(lbl, i, 0)
            joy_layout.addWidget(bar, i, 1)
            self.joy_bars.append(bar)
        joy_group.setLayout(joy_layout)

        layout.addWidget(joy_group)
        layout.addStretch()
        tab.setLayout(layout)
        self.tabs.addTab(tab, "Flight Control")

    def init_maintenance_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Toggle
        self.chk_override = QCheckBox("Enable Manual Override")
        self.chk_override.setStyleSheet("font-size: 14px; font-weight: bold; color: #FF5252;")
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
            slider.setRange(0, 1000)
            slider.valueChanged.connect(lambda val, idx=i: self.update_slider(idx, val))
            
            bar = QProgressBar() # RPM feedback
            bar.setRange(0, 10000)
            bar.setStyleSheet("QProgressBar::chunk { background-color: #66BB6A; }")
            
            sliders_layout.addWidget(lbl, i, 0)
            sliders_layout.addWidget(slider, i, 1)
            sliders_layout.addWidget(bar, i, 2)
            
            self.sliders.append(slider)
            self.rpm_bars.append(bar)

        # 6 Servos
        for i in range(6):
            lbl = QLabel(f"Servo {i+1}")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 100)
            slider.valueChanged.connect(lambda val, idx=i+6: self.update_slider(idx, val))
            
            sliders_layout.addWidget(lbl, i+6, 0)
            sliders_layout.addWidget(slider, i+6, 1)
            
            self.sliders.append(slider)

        layout.addLayout(sliders_layout)
        tab.setLayout(layout)
        self.tabs.addTab(tab, "Maintenance")

    def init_system_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        
        # 1. Connection Status
        conn_group = QGroupBox("Connection")
        conn_layout = QHBoxLayout()
        self.lbl_conn_status = QLabel("Status: Disconnected")
        self.lbl_conn_status.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        
        btn_refresh = QPushButton("Check Connection")
        btn_refresh.clicked.connect(self.check_connection)
        
        conn_layout.addWidget(self.lbl_conn_status)
        conn_layout.addWidget(btn_refresh)
        conn_group.setLayout(conn_layout)
        
        # 2. Node Management
        node_group = QGroupBox("Remote Node Manager")
        node_layout = QGridLayout()
        
        # Bridge (Node B)
        node_layout.addWidget(QLabel("Node B (Bridge):"), 0, 0)
        btn_start_bridge = QPushButton("Launch Bridge")
        btn_start_bridge.setStyleSheet("background-color: #4CAF50; color: white;")
        btn_start_bridge.clicked.connect(self.remote_manager.start_bridge_node)
        
        btn_stop_bridge = QPushButton("Kill Bridge")
        btn_stop_bridge.setStyleSheet("background-color: #F44336; color: white;")
        btn_stop_bridge.clicked.connect(self.remote_manager.stop_bridge_node)
        
        node_layout.addWidget(btn_start_bridge, 0, 1)
        node_layout.addWidget(btn_stop_bridge, 0, 2)
        
        # Controller (Node C)
        node_layout.addWidget(QLabel("Node C (Controller):"), 1, 0)
        btn_start_ctrl = QPushButton("Launch Controller")
        btn_start_ctrl.setStyleSheet("background-color: #4CAF50; color: white;")
        btn_start_ctrl.clicked.connect(self.remote_manager.start_controller_node)
        
        btn_stop_ctrl = QPushButton("Kill Controller")
        btn_stop_ctrl.setStyleSheet("background-color: #F44336; color: white;")
        btn_stop_ctrl.clicked.connect(self.remote_manager.stop_controller_node)
        
        node_layout.addWidget(btn_start_ctrl, 1, 1)
        node_layout.addWidget(btn_stop_ctrl, 1, 2)
        
        node_group.setLayout(node_layout)
        
        # 3. System Actions
        sys_group = QGroupBox("System Actions")
        sys_layout = QHBoxLayout()
        
        btn_reboot = QPushButton("Reboot Node B")
        btn_reboot.setStyleSheet("background-color: #FF9800; color: black;")
        btn_reboot.clicked.connect(self.remote_manager.reboot_rpi)
        
        sys_layout.addWidget(btn_reboot)
        sys_group.setLayout(sys_layout)

        layout.addWidget(conn_group)
        layout.addWidget(node_group)
        layout.addWidget(sys_group)
        layout.addStretch()
        
        tab.setLayout(layout)
    def init_monitor_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        
        # 1. Frequency Counter
        freq_layout = QHBoxLayout()
        self.lbl_imu_freq = QLabel("IMU Rate: 0 Hz")
        self.lbl_imu_freq.setFont(QFont("Arial", 14, QFont.Bold))
        self.lbl_imu_freq.setStyleSheet("color: #00E676;") # Green
        freq_layout.addWidget(self.lbl_imu_freq)
        freq_layout.addStretch()
        layout.addLayout(freq_layout)
        
        # 2. Scrolling Log
        self.txt_log = pyqtTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet("background-color: black; color: #00E676; font-family: Consolas; font-size: 10pt;")
        layout.addWidget(self.txt_log)
        
        # 3. Controls
        btn_clear = QPushButton("Clear Log")
        btn_clear.clicked.connect(self.txt_log.clear)
        layout.addWidget(btn_clear)
        
        tab.setLayout(layout)
        self.tabs.addTab(tab, "Data Monitor")
        
        # Timer for frequency update (1Hz)
        self.freq_timer = QTimer()
        self.freq_timer.timeout.connect(self.update_frequency)
        self.freq_timer.start(1000)

    def update_frequency(self):
        # Calculate actual Hz based on packet count
        freq = self.node.imu_packet_count
        self.node.imu_packet_count = 0 # Reset count
        self.lbl_imu_freq.setText(f"IMU Rate: {freq} Hz")
        
        if freq < 100:
             self.lbl_imu_freq.setStyleSheet("color: #F44336;") # Red (Low)
        elif freq < 800:
             self.lbl_imu_freq.setStyleSheet("color: #FF9800;") # Orange (Warning)
        else:
             self.lbl_imu_freq.setStyleSheet("color: #00E676;") # Green (Good)

    def check_connection(self):
        is_connected = self.remote_manager.check_connection()
        if is_connected:
            self.lbl_conn_status.setText("Status: Connected (Node B)")
            self.lbl_conn_status.setStyleSheet("color: #66BB6A; font-weight: bold; font-size: 14px;")
        else:
            self.lbl_conn_status.setText("Status: Disconnected")
            self.lbl_conn_status.setStyleSheet("color: #F44336; font-weight: bold; font-size: 14px;")

    def toggle_override(self, checked):
        self.node.manual_override_active = checked

    def update_slider(self, index, value):
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
        self.attitude_indicator.setRoll(r)
        self.attitude_indicator.setPitch(p)

    def update_joy(self, axes, buttons):
        # Update Joy bars
        for i, bar in enumerate(self.joy_bars):
            if i < len(axes):
                bar.setValue(int(axes[i] * 100))

    def update_imu(self, ax, ay, az, gx, gy, gz):
        self.lbl_imu_raw.setText(f"IMU Raw: Ax={ax:.2f}, Ay={ay:.2f}, Az={az:.2f} | Gx={gx:.2f}, Gy={gy:.2f}, Gz={gz:.2f}")
        
        # Update Monitor Log (Limit to last 1000 chars to prevent freeze)
        log_entry = f"Ax:{ax:6.2f} Ay:{ay:6.2f} Az:{az:6.2f} | Gx:{gx:6.2f} Gy:{gy:6.2f} Gz:{gz:6.2f}"
        self.txt_log.append(log_entry)
        
        # Auto scroll logic is implicit in append, but we might want to truncate if too long
        # Implementation Detail: Checking length is expensive in GUI thread, so just rely on circular buffer if needed or trust user to clear
        # But to be safe, let's clear if line count > 500
        if self.txt_log.document().blockCount() > 500:
             self.txt_log.clear()

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    
    # Worker Signals
    class Signals(QObject):
        telemetry_signal = pyqtSignal(list)
        odometry_signal = pyqtSignal(float, float, float)
        attitude_signal = pyqtSignal(float, float, float)
        imu_signal = pyqtSignal(float, float, float, float, float, float)
        joy_signal = pyqtSignal(list, list)
    
    signals = Signals()
    
    node = AltairGCSNode(signals)
    window = MainWindow(node)
    
    # Connect signals
    signals.telemetry_signal.connect(window.update_telemetry)
    signals.odometry_signal.connect(window.update_odometry)
    signals.attitude_signal.connect(window.update_attitude)
    signals.imu_signal.connect(window.update_imu)
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
