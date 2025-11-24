#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <cmath>

class SensorProcessingNode : public rclcpp::Node
{
public:
    SensorProcessingNode()
    : Node("sensor_processing_node"),
      imu_sample_rate_(100.0),
      gyro_x_filter_(NotchFilter(100.0, 30.0, 0.8)),
      gyro_y_filter_(NotchFilter(100.0, 30.0, 0.8)),
      gyro_z_filter_(NotchFilter(100.0, 30.0, 0.8))
    {
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
        
        // Declare and get parameter
        this->declare_parameter("imu_sample_rate", 100.0);
        imu_sample_rate_ = this->get_parameter("imu_sample_rate").as_double();

        // Re-initialize filters with correct sample rate
        // (Initial frequency 30Hz is just a placeholder until ESC telemetry arrives)
        gyro_x_filter_.reconfigure(imu_sample_rate_, 30.0, 0.8);
        gyro_y_filter_.reconfigure(imu_sample_rate_, 30.0, 0.8);
        gyro_z_filter_.reconfigure(imu_sample_rate_, 30.0, 0.8);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/pico/imu_raw", sensor_qos,
            std::bind(&SensorProcessingNode::imuCallback, this, std::placeholders::_1));

        esc_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pico/esc_telemetry", sensor_qos,
            std::bind(&SensorProcessingNode::escCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", sensor_qos);
        
        RCLCPP_INFO(this->get_logger(), "Sensor Processing Node Started. IMU Rate: %.1f Hz", imu_sample_rate_);
    }

private:
    // A simple second-order notch filter implementation
    class NotchFilter {
    public:
        NotchFilter(double fs = 100.0, double f0 = 30.0, double Q = 0.8) 
        : b0_(1.0), b1_(0.0), b2_(0.0), a0_(1.0), a1_(0.0), a2_(0.0),
          x1_(0.0), x2_(0.0), y1_(0.0), y2_(0.0) {
            reconfigure(fs, f0, Q);
        }

        void reconfigure(double fs, double f0, double Q) {
            // Avoid division by zero
            if (fs <= 0 || f0 <= 0 || Q <= 0) return;
            
            // Nyquist Safety Check: If f0 is near or above fs/2, the filter is unstable.
            if (f0 >= (fs / 2.0) * 0.95) {
                return; // Do not update coefficients if unsafe
            }
            
            double w0 = 2.0 * M_PI * f0 / fs;
            double alpha = sin(w0) / (2.0 * Q);

            b0_ = 1.0;
            b1_ = -2.0 * cos(w0);
            b2_ = 1.0;
            a0_ = 1.0 + alpha;
            a1_ = -2.0 * cos(w0);
            a2_ = 1.0 - alpha;

            // Pre-normalize coefficients
            b0_ /= a0_;
            b1_ /= a0_;
            b2_ /= a0_;
            a1_ /= a0_;
            a2_ /= a0_;
        }

        double apply(double x0) {
            double y0 = b0_ * x0 + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
            x2_ = x1_;
            x1_ = x0;
            y2_ = y1_;
            y1_ = y0;
            return y0;
        }
    private:
        double b0_, b1_, b2_, a0_, a1_, a2_;
        double x1_, x2_, y1_, y2_;
    };

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Copy message, we'll modify it
        auto filtered_msg = *msg;

        // Apply filter to gyro data
        filtered_msg.angular_velocity.x = gyro_x_filter_.apply(msg->angular_velocity.x);
        filtered_msg.angular_velocity.y = gyro_y_filter_.apply(msg->angular_velocity.y);
        filtered_msg.angular_velocity.z = gyro_z_filter_.apply(msg->angular_velocity.z);

        imu_pub_->publish(filtered_msg);
    }

    void escCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        // Average RPM from all motors
        double total_rpm = 0.0;
        for (float rpm : msg->data) {
            total_rpm += rpm;
        }
        double avg_rpm = total_rpm / msg->data.size();
        
        // Convert RPM to Hz (assuming RPM is rotations per minute)
        double motor_hz = avg_rpm / 60.0;

        // Update notch filter center frequency
        // This should have some safety logic (e.g. if motor_hz is 0)
        if (motor_hz > 1.0) { // Only update if motors are spinning reasonably
            double quality_factor = 0.8;
            
            gyro_x_filter_.reconfigure(imu_sample_rate_, motor_hz, quality_factor);
            gyro_y_filter_.reconfigure(imu_sample_rate_, motor_hz, quality_factor);
            gyro_z_filter_.reconfigure(imu_sample_rate_, motor_hz, quality_factor);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr esc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    double imu_sample_rate_;
    NotchFilter gyro_x_filter_, gyro_y_filter_, gyro_z_filter_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorProcessingNode>());
    rclcpp::shutdown();
    return 0;
}
