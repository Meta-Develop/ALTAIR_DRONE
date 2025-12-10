#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

class ImuUnpackerNode : public rclcpp::Node
{
public:
    ImuUnpackerNode()
    : Node("imu_unpacker_node")
    {
        rclcpp::QoS sensor_qos(10);
        sensor_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sensor_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        // Batch Subscriber
        batch_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pico/imu_batch", sensor_qos,
            std::bind(&ImuUnpackerNode::batchCallback, this, std::placeholders::_1));

        // IMU Publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/pico/imu_raw", sensor_qos);
        
        RCLCPP_INFO(this->get_logger(), "IMU Unpacker Node Started.");
    }

private:
    void batchCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.empty() || msg->data.size() % 6 != 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid batch size: %zu", msg->data.size());
            return;
        }

        size_t num_samples = msg->data.size() / 6;
        rclcpp::Time now = this->get_clock()->now();
        
        // Time step: 1kHz = 1ms
        double dt = 0.001; 

        // SCALING FACTORS (BNO055/MPU6050 AMG Mode)
        // Configured in Pico for +/- 16G and +/- 2000 dps
        
        // Accel: Range +/- 16G. Sensitivity = 2048 LSB/g
        // 1 LSB = 1/2048 g = 0.00048828 g
        // 1 g = 9.80665 m/s^2
        // Scale = (1/2048) * 9.80665 = 0.0047884
        const double ACCEL_SCALE = 0.0047884; 

        // Gyro: Range 2000 dps. Sensitivity = 16.4 LSB/dps
        // 1 LSB = 1/16.4 dps = 0.0609756 dps
        // Scale (rad/s) = (1/16.4) * (PI/180) = 0.0010642
        const double GYRO_SCALE = 0.0010642;

        for (size_t i = 0; i < num_samples; ++i) {
            sensor_msgs::msg::Imu imu_msg;
            
            // Timestamp back-calculation
            // Sample i=0 is oldest, i=num_samples-1 is newest (arrived 'now')
            double offset = (double)(num_samples - 1 - i) * dt;
            imu_msg.header.stamp = now - rclcpp::Duration::from_seconds(offset);
            imu_msg.header.frame_id = "imu_link";

            float ax_raw = msg->data[i*6 + 0];
            float ay_raw = msg->data[i*6 + 1];
            float az_raw = msg->data[i*6 + 2];
            float gx_raw = msg->data[i*6 + 3];
            float gy_raw = msg->data[i*6 + 4];
            float gz_raw = msg->data[i*6 + 5];

            // Apply Scaling
            imu_msg.linear_acceleration.x = ax_raw * ACCEL_SCALE;
            imu_msg.linear_acceleration.y = ay_raw * ACCEL_SCALE;
            imu_msg.linear_acceleration.z = az_raw * ACCEL_SCALE;

            imu_msg.angular_velocity.x = gx_raw * GYRO_SCALE;
            imu_msg.angular_velocity.y = gy_raw * GYRO_SCALE;
            imu_msg.angular_velocity.z = gz_raw * GYRO_SCALE;

            // No Orientation/Quaternion in AMG mode (Raw Only)
            imu_msg.orientation_covariance[0] = -1; // -1 indicates no orientation data

            imu_pub_->publish(imu_msg);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr batch_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuUnpackerNode>());
    rclcpp::shutdown();
    return 0;
}
