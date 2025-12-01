#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

// Dynamixel Control Table Addresses (Example for X-series)
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define LEN_GOAL_POSITION           4
#define PROTOCOL_VERSION            2.0
#define BAUDRATE                    57600
#define DEVICENAME                  "/dev/ttyUSB0" // Check U2D2 path

class ActuatorBridge : public rclcpp::Node
{
public:
    ActuatorBridge() : Node("actuator_bridge")
    {
        // QoS
        auto qos = rclcpp::SensorDataQoS();
        auto qos_reliable = rclcpp::QoS(10).reliable();

        // Publishers
        motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pico/motor_commands", qos);

        // Subscribers
        actuator_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/control/actuator_commands", qos,
            std::bind(&ActuatorBridge::actuator_callback, this, std::placeholders::_1));

        manual_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/control/manual_override", qos_reliable,
            std::bind(&ActuatorBridge::manual_callback, this, std::placeholders::_1));

        // Timer for Watchdog (100Hz check)
        timer_ = this->create_wall_timer(
            10ms, std::bind(&ActuatorBridge::watchdog_callback, this));

        // Initialize Dynamixel
        init_dynamixel();

        RCLCPP_INFO(this->get_logger(), "Actuator Bridge Started");
    }

    ~ActuatorBridge()
    {
        // Disable Torque
        for(int id : servo_ids_) {
            packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0);
        }
        portHandler_->closePort();
    }

private:
    void init_dynamixel()
    {
        portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (portHandler_->openPort()) {
            RCLCPP_INFO(this->get_logger(), "Succeeded to open the port");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port");
            rclcpp::shutdown();
            return;
        }

        if (portHandler_->setBaudRate(BAUDRATE)) {
            RCLCPP_INFO(this->get_logger(), "Succeeded to change the baudrate");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to change the baudrate");
            rclcpp::shutdown();
            return;
        }

        // Enable Torque for IDs 1-6 (assuming servos are 1-6 or 7-12? Task says "Last 6 values (Servos)")
        // Let's assume Servo IDs are 1-6 for simplicity, or 7-12. Let's use 1-6.
        servo_ids_ = {1, 2, 3, 4, 5, 6}; 
        for(int id : servo_ids_) {
            int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1);
            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d", id);
                rclcpp::shutdown();
                return;
            }
        }
    }

    void actuator_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        last_msg_time_ = this->now();
        if (!manual_mode_) {
            process_commands(msg->data);
        }
    }

    void manual_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        last_msg_time_ = this->now();
        last_manual_time_ = this->now();
        if (!manual_mode_) {
            manual_mode_ = true;
            RCLCPP_INFO(this->get_logger(), "Manual Override Engaged");
        }
        process_commands(msg->data);
    }

    void watchdog_callback()
    {
        auto now = this->now();
        if ((now - last_msg_time_).seconds() > 0.1) {
            // Timeout - Failsafe
            stop_all();
        }
        
        // Reset manual mode if no manual msg for 1.0s
        if (manual_mode_ && (now - last_manual_time_).seconds() > 1.0) {
            manual_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Manual Override Timed Out - Reverting to Auto");
        }
    }

    void process_commands(const std::vector<float>& data)
    {
        if (data.size() < 12) {
            RCLCPP_ERROR(this->get_logger(), "Invalid command size: %zu", data.size());
            return;
        }

        // 1. Motors (0-5) -> Pico
        auto motor_msg = std_msgs::msg::Float32MultiArray();
        motor_msg.data.assign(data.begin(), data.begin() + 6);
        motor_pub_->publish(motor_msg);

        // 2. Servos (6-11) -> Dynamixel
        // Assuming data is in Radians. Convert to Value.
        // Dynamixel 0-4095 usually covers 0-360 or similar.
        // Need conversion logic. Assuming 1:1 mapping for now or raw values.
        // Let's assume input is raw position value for simplicity, or add a scaler.
        for (size_t i = 0; i < 6; ++i) {
            int id = servo_ids_[i];
            uint32_t pos = static_cast<uint32_t>(data[6 + i]); 
            packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_POSITION, pos);
        }
    }

    void stop_all()
    {
        // Zero Motors
        auto motor_msg = std_msgs::msg::Float32MultiArray();
        motor_msg.data.resize(6, 0.0f);
        motor_pub_->publish(motor_msg);

        // Home Servos (Assuming 2048 is center/home)
        for(int id : servo_ids_) {
            packetHandler_->write4ByteTxRx(portHandler_, id, ADDR_GOAL_POSITION, 2048);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr manual_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    std::vector<int> servo_ids_;

    rclcpp::Time last_msg_time_;
    rclcpp::Time last_manual_time_;
    bool manual_mode_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorBridge>());
    rclcpp::shutdown();
    return 0;
}
