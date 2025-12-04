#include "actuator_bridge/actuator_bridge_node.hpp"

using std::placeholders::_1;

ActuatorBridgeNode::ActuatorBridgeNode()
: Node("actuator_bridge"),
  maintenance_mode_(false),
  portHandler_(nullptr),
  packetHandler_(nullptr),
  groupSyncWrite_(nullptr),
  dxl_device_port_("/dev/ttyUSB0"),
  dxl_baudrate_(57600),
  dxl_protocol_version_(2.0),
  ignore_servo_errors_(false)
{
    // Declare Parameters
    this->declare_parameter("dxl_device", "/dev/ttyUSB0");
    this->declare_parameter("dxl_baudrate", 57600);
    this->declare_parameter("dxl_protocol_version", 2.0);
    this->declare_parameter("ignore_servo_errors", false);

    // Get Parameters
    dxl_device_port_ = this->get_parameter("dxl_device").as_string();
    dxl_baudrate_ = this->get_parameter("dxl_baudrate").as_int();
    dxl_protocol_version_ = this->get_parameter("dxl_protocol_version").as_double();
    ignore_servo_errors_ = this->get_parameter("ignore_servo_errors").as_bool();

    // QoS Profiles
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
    rclcpp::QoS reliable_qos = rclcpp::QoS(10).reliable();

    // Subscribers
    control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/control/actuator_commands", sensor_qos,
        std::bind(&ActuatorBridgeNode::controlCallback, this, _1));

    override_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/control/manual_override", reliable_qos,
        std::bind(&ActuatorBridgeNode::overrideCallback, this, _1));

    // Publishers
    pico_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/pico/motor_commands", sensor_qos);

    // Timer for Failsafe (run at 20Hz check)
    failsafe_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ActuatorBridgeNode::timerCallback, this));

    last_msg_time_ = std::chrono::steady_clock::now();

    // Initialize Dynamixel
    if (!initDynamixel()) {
        RCLCPP_WARN(this->get_logger(), "Failed to initialize Dynamixel SDK on %s. Running in EMULATION MODE (No Servos).", dxl_device_port_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Dynamixel initialized successfully on %s at %d baud.", dxl_device_port_.c_str(), dxl_baudrate_);
    }

    RCLCPP_INFO(this->get_logger(), "Actuator Bridge Node Started.");
}

ActuatorBridgeNode::~ActuatorBridgeNode()
{
    setSafeState();
    if (portHandler_) {
        portHandler_->closePort();
        delete portHandler_;
    }
    if (packetHandler_) {
        delete packetHandler_;
    }
    if (groupSyncWrite_) {
        delete groupSyncWrite_;
    }
}

bool ActuatorBridgeNode::initDynamixel()
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(dxl_device_port_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(dxl_protocol_version_);

    if (portHandler_->openPort()) {
        if (portHandler_->setBaudRate(dxl_baudrate_)) {
            // Enable Torque
            // Enable Torque
            if (!ignore_servo_errors_) {
                for (auto id : DXL_IDS) {
                    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1);
                    if (dxl_comm_result != COMM_SUCCESS) {
                        RCLCPP_WARN(this->get_logger(), "Failed to enable torque for ID %d", id);
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Skipping Torque Enable (ignore_servo_errors=True)");
            }
            
            // Initialize GroupSyncWrite
            groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
            
            return true;
        }
    }
    return false;
}

void ActuatorBridgeNode::controlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (maintenance_mode_) {
        return; 
    }
    
    last_msg_time_ = std::chrono::steady_clock::now();
    if (msg->data.size() >= 12) {
        processCommands(msg->data);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received control command with insufficient size: %zu", msg->data.size());
    }
}

void ActuatorBridgeNode::overrideCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    maintenance_mode_ = true; // Activate maintenance mode
    last_msg_time_ = std::chrono::steady_clock::now(); // Reset timeout
    
    if (msg->data.size() >= 12) {
        processCommands(msg->data);
    } else {
        RCLCPP_WARN(this->get_logger(), "Received override command with insufficient size: %zu", msg->data.size());
    }
}

void ActuatorBridgeNode::timerCallback()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_msg_time_).count();

    if (duration > 100) {
        if (maintenance_mode_) {
             RCLCPP_WARN(this->get_logger(), "Maintenance Override Timed Out. Resetting mode.");
             maintenance_mode_ = false;
        }
        setSafeState();
    }
}

void ActuatorBridgeNode::processCommands(const std::vector<float>& data)
{
    // Data format: [M1, M2, M3, M4, M5, M6, S1, S2, S3, S4, S5, S6]
    
    // 1. Motors -> Pico
    std_msgs::msg::Float32MultiArray motor_msg;
    for (int i = 0; i < 6; ++i) {
        motor_msg.data.push_back(data[i]);
    }
    pico_pub_->publish(motor_msg);

    // 2. Servos -> Dynamixel
    std::vector<float> servo_angles;
    for (int i = 6; i < 12; ++i) {
        servo_angles.push_back(data[i]);
    }
    writeServos(servo_angles);
}

void ActuatorBridgeNode::writeServos(const std::vector<float>& angles)
{
    if (!groupSyncWrite_) return;

    groupSyncWrite_->clearParam();

    for (size_t i = 0; i < angles.size() && i < DXL_IDS.size(); ++i) {
        // Convert rad to DXL value (e.g. 0 rad = 2048, 1 rad approx 651 ticks for XM430)
        int32_t goal_pos = 2048 + (angles[i] * 651.89);
        
        // Prepare byte array for SyncWrite (Little Endian)
        uint8_t param_goal_position[4];
        param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos));
        param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos));
        param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_pos));
        param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_pos));

        // Add to SyncWrite
        groupSyncWrite_->addParam(DXL_IDS[i], param_goal_position);
    }

    // Transmit all at once
    groupSyncWrite_->txPacket();
    groupSyncWrite_->clearParam();
}

void ActuatorBridgeNode::setSafeState()
{
    // 0 Thrust
    std_msgs::msg::Float32MultiArray stop_msg;
    stop_msg.data.assign(6, 0.0f);
    pico_pub_->publish(stop_msg);

    // Home Servos (0 Rad)
    std::vector<float> home_angles(6, 0.0f);
    writeServos(home_angles);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
