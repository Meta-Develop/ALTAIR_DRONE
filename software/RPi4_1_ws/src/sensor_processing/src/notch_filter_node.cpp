#include <memory>
#include <vector>
#include <cmath>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// Simple 2nd Order IIR Notch Filter
class NotchFilter {
public:
    NotchFilter() { reset(); }

    void update_coefficients(float center_freq, float sample_freq, float width = 10.0f) {
        if (center_freq <= 0 || sample_freq <= 0) return;

        float w0 = 2.0f * M_PI * center_freq / sample_freq;
        float alpha = sin(w0) * sinh(log(2.0f) / 2.0f * width * w0 / sin(w0));

        b0 = 1.0f;
        b1 = -2.0f * cos(w0);
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos(w0);
        a2 = 1.0f - alpha;

        // Normalize
        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;
    }

    float apply(float input) {
        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        return output;
    }

    void reset() {
        x1 = x2 = y1 = y2 = 0.0f;
        b0 = 1.0f; b1 = 0.0f; b2 = 0.0f;
        a0 = 1.0f; a1 = 0.0f; a2 = 0.0f;
    }

private:
    float x1, x2, y1, y2;
    float b0, b1, b2, a0, a1, a2;
};

class NotchFilterNode : public rclcpp::Node
{
public:
    NotchFilterNode() : Node("notch_filter_node")
    {
        auto qos = rclcpp::SensorDataQoS();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", qos);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/pico/imu_raw", qos,
            std::bind(&NotchFilterNode::imu_callback, this, std::placeholders::_1));

        telemetry_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pico/esc_telemetry", qos,
            std::bind(&NotchFilterNode::telemetry_callback, this, std::placeholders::_1));

        // Initialize filters for Gyro X, Y, Z
        filters_.resize(3);
        
        // Default params
        sample_freq_ = 400.0f; // IMU rate
        pole_pairs_ = 7.0f;

        RCLCPP_INFO(this->get_logger(), "Notch Filter Node Started");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto filtered_msg = *msg;

        // Apply filters to Gyro
        filtered_msg.angular_velocity.x = filters_[0].apply(msg->angular_velocity.x);
        filtered_msg.angular_velocity.y = filters_[1].apply(msg->angular_velocity.y);
        filtered_msg.angular_velocity.z = filters_[2].apply(msg->angular_velocity.z);

        imu_pub_->publish(filtered_msg);
    }

    void telemetry_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        // Calculate average RPM
        float sum_rpm = std::accumulate(msg->data.begin(), msg->data.end(), 0.0f);
        float avg_rpm = sum_rpm / msg->data.size();

        // Calculate Center Frequency
        // F_elec = RPM / 60 * PolePairs
        // F_vib = F_elec (fundamental)
        float center_freq = (avg_rpm / 60.0f) * pole_pairs_;

        // Update filters
        // Avoid low frequencies or zero
        if (center_freq > 20.0f) {
            for (auto& filter : filters_) {
                filter.update_coefficients(center_freq, sample_freq_);
            }
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telemetry_sub_;

    std::vector<NotchFilter> filters_;
    float sample_freq_;
    float pole_pairs_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NotchFilterNode>());
    rclcpp::shutdown();
    return 0;
}
