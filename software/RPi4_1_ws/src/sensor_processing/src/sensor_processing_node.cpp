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
      imu_sample_rate_(1000.0), // 1kHz native sample rate
      gyro_x_filter_(NotchFilter(1000.0, 30.0, 0.8)),
      gyro_y_filter_(NotchFilter(1000.0, 30.0, 0.8)),
      gyro_z_filter_(NotchFilter(1000.0, 30.0, 0.8))
    {
        // QoS: Must match Micro-ROS (Best Effort, Volatile)
        rclcpp::QoS sensor_qos(10);
        sensor_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sensor_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        rclcpp::QoS pub_qos = rclcpp::SystemDefaultsQoS(); // Reliable for EKF
        
        // Declare and get parameter
        this->declare_parameter("imu_sample_rate", 1000.0);
        imu_sample_rate_ = this->get_parameter("imu_sample_rate").as_double();

        // Re-initialize filters with correct sample rate
        // (Initial frequency 30Hz is just a placeholder until ESC telemetry arrives)
        gyro_x_filter_.reconfigure(imu_sample_rate_, 30.0, 0.8);
        gyro_y_filter_.reconfigure(imu_sample_rate_, 30.0, 0.8);
        gyro_z_filter_.reconfigure(imu_sample_rate_, 30.0, 0.8);

        // Subscribe to Batch Data
        imu_batch_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pico/imu_batch", sensor_qos,
            std::bind(&SensorProcessingNode::imuBatchCallback, this, std::placeholders::_1));

        esc_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pico/esc_telemetry", sensor_qos,
            std::bind(&SensorProcessingNode::escCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/filtered", pub_qos);
        
        RCLCPP_INFO(this->get_logger(), "Sensor Processing Node Started. Mode: 1kHz Batch Processing");
    }

private:
    // A simple second-order notch filter implementation
    class NotchFilter {
    public:
        NotchFilter(double fs = 1000.0, double f0 = 30.0, double Q = 0.8) 
        : b0_(1.0), b1_(0.0), b2_(0.0), a0_(1.0), a1_(0.0), a2_(0.0),
          x1_(0.0), x2_(0.0), y1_(0.0), y2_(0.0) {
            reconfigure(fs, f0, Q);
        }

        void reconfigure(double fs, double f0, double Q) {
            // Pre-warp frequency for bilinear transform or use standard discrete mapping?
            // Using standard audio EQ cookbook formula associated with rbj usage
            double omega = 2.0 * M_PI * f0 / fs;
            double sn = sin(omega);
            double cs = cos(omega);
            double alpha = sn / (2.0 * Q);

            double b0 = 1.0;
            double b1 = -2.0 * cs;
            double b2 = 1.0;
            double a0 = 1.0 + alpha;
            double a1 = -2.0 * cs;
            double a2 = 1.0 - alpha;

            // Normalize
            b0_ = b0 / a0;
            b1_ = b1 / a0;
            b2_ = b2 / a0;
            a1_ = a1 / a0;
            a2_ = a2 / a0;
        }
        double apply(double input) {
            double output = b0_*input + b1_*x1_ + b2_*x2_ - a1_*y1_ - a2_*y2_;
            x2_ = x1_;
            x1_ = input;
            y2_ = y1_;
            y1_ = output;
            return output;
        }

    private:
        double b0_, b1_, b2_, a0_, a1_, a2_;
        double x1_, x2_, y1_, y2_;
    };

    void imuBatchCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.empty() || msg->data.size() % 6 != 0) return;

        int samples = msg->data.size() / 6;
        rclcpp::Time now = this->get_clock()->now();
        uint64_t now_ns = now.nanoseconds();
        uint64_t dt_ns = 1000000;
        
        const double ACCEL_SCALE = 0.0047884; 
        const double GYRO_SCALE = 0.0010642;

        for (int i = 0; i < samples; i++) {
            // Reconstruct IMU message
            sensor_msgs::msg::Imu imu_msg;
            
            // Calculate timestamp for this sample
            // sample 0 (oldest) -> time: now - (samples-1-i)*dt
            // sample 19 (newest) -> time: now
            uint64_t sample_time_ns = now_ns - ((samples - 1 - i) * dt_ns);
            imu_msg.header.stamp = rclcpp::Time(sample_time_ns);
            imu_msg.header.frame_id = "base_link";

            // Unpack Data (Order: ax, ay, az, gx, gy, gz)
            int base_idx = i * 6;
            
            // Apply Scaling
            imu_msg.linear_acceleration.x = msg->data[base_idx + 0] * ACCEL_SCALE;
            imu_msg.linear_acceleration.y = msg->data[base_idx + 1] * ACCEL_SCALE;
            imu_msg.linear_acceleration.z = msg->data[base_idx + 2] * ACCEL_SCALE;
            
            // Gyro (Apply Scaling AND Filter)
            double gx = msg->data[base_idx + 3] * GYRO_SCALE;
            double gy = msg->data[base_idx + 4] * GYRO_SCALE;
            double gz = msg->data[base_idx + 5] * GYRO_SCALE;

            imu_msg.angular_velocity.x = gyro_x_filter_.apply(gx);
            imu_msg.angular_velocity.y = gyro_y_filter_.apply(gy);
            imu_msg.angular_velocity.z = gyro_z_filter_.apply(gz);
            
            // No Orientation (Raw)
            imu_msg.orientation_covariance[0] = -1;

            // Publish individualized message
            imu_pub_->publish(imu_msg);
        }
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

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_batch_sub_;
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
