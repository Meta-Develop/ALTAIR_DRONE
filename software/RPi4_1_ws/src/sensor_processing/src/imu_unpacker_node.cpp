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
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
        
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
        // Assuming 1kHz sampling (1ms dt)
        // The last sample is 'now' (approx), previous are older.
        // Or we can use system time if we trust the transport latency is stable.
        // Better: Use arrival time and back-calculate.
        
        double dt = 0.001; // 1ms

        for (size_t i = 0; i < num_samples; ++i) {
            sensor_msgs::msg::Imu imu_msg;
            
            // Calculate timestamp for this sample
            // Sample i=0 is oldest, i=num_samples-1 is newest.
            // Time of sample i = now - (num_samples - 1 - i) * dt
            double offset = (double)(num_samples - 1 - i) * dt;
            imu_msg.header.stamp = now - rclcpp::Duration::from_seconds(offset);
            imu_msg.header.frame_id = "imu_link";

            // Unpack Data (Raw integers converted to float on Pico)
            // Scaling factors for BNO055:
            // Accel: 1 LSB = 1 mg = 0.00981 m/s^2 (Default 4G range? No, 4G range is 1 LSB = ??)
            // Wait, default is 1 LSB = 1 mg?
            // Datasheet:
            // Accel: 1 m/s^2 = 100 LSB (Configurable).
            // By default in Fusion mode it's 100 LSB = 1 m/s^2.
            // In Raw mode (AMG), it depends on range.
            // +/- 4G range. 
            // If 1 LSB = 1 mg, then +/- 4G = +/- 4000 LSB? No, int16 is +/- 32768.
            // Usually 4G range means full scale is 4G.
            // Resolution: 4G / 32768?
            // BNO055 Datasheet Table 3-17:
            // Accel 4G range: 1 LSB = 1.95 mg?
            // Wait, let's check standard driver or assume 1 LSB = 1 mg for now and calibrate later.
            // Actually, BNO055 default unit is 1 LSB = 1 mg (if unit selection is default).
            // But in AMG mode, it outputs raw register values.
            // Let's assume 1 LSB = 1 mg = 0.00981 m/s^2 for now.
            // Gyro: 2000 dps range.
            // 1 LSB = 1/16 dps = 0.0625 dps?
            // Datasheet: 2000 dps -> 1 LSB = 1/16 dps.
            
            float ax_raw = msg->data[i*6 + 0];
            float ay_raw = msg->data[i*6 + 1];
            float az_raw = msg->data[i*6 + 2];
            float gx_raw = msg->data[i*6 + 3];
            float gy_raw = msg->data[i*6 + 4];
            float gz_raw = msg->data[i*6 + 5];

            imu_msg.linear_acceleration.x = ax_raw * 0.00981; // mg to m/s^2 (Approx)
            imu_msg.linear_acceleration.y = ay_raw * 0.00981;
            imu_msg.linear_acceleration.z = az_raw * 0.00981;

            double dps_to_rads = 0.0174533 / 16.0; // 1/16 dps to rad/s
            imu_msg.angular_velocity.x = gx_raw * dps_to_rads;
            imu_msg.angular_velocity.y = gy_raw * dps_to_rads;
            imu_msg.angular_velocity.z = gz_raw * dps_to_rads;

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
