#ifndef ACTUATOR_BRIDGE_NODE_HPP_
#define ACTUATOR_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <vector>
#include <memory>

// Dynamixel SDK header
#include "dynamixel_sdk/dynamixel_sdk.h"
// specific header might not be needed if dynamixel_sdk.h includes it, but usually it's separate or scoped.
// Checking standard SDK usage: dynamixel_sdk.h usually includes everything.

class ActuatorBridgeNode : public rclcpp::Node
{
public:
    ActuatorBridgeNode();
    ~ActuatorBridgeNode();

private:
    // Callbacks
    void controlCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void overrideCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void timerCallback();

    // Logic
    void processCommands(const std::vector<float>& data);
    void setSafeState();
    bool initDynamixel();
    void writeServos(const std::vector<float>& angles);

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr override_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pico_pub_;
    rclcpp::TimerBase::SharedPtr failsafe_timer_;

    // State
    std::chrono::steady_clock::time_point last_msg_time_;
    bool maintenance_mode_;
    
    // Dynamixel
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncWrite *groupSyncWrite_;

    // Dynamixel Configuration
    std::string dxl_device_port_;
    int dxl_baudrate_;
    float dxl_protocol_version_;
    
    // ID mapping for servos 1-6
    const std::vector<uint8_t> DXL_IDS = {1, 2, 3, 4, 5, 6}; 
    const uint16_t ADDR_GOAL_POSITION = 116; // Example address for X-series
    const uint16_t ADDR_TORQUE_ENABLE = 64;
    const uint16_t LEN_GOAL_POSITION = 4;
};

#endif // ACTUATOR_BRIDGE_NODE_HPP_
