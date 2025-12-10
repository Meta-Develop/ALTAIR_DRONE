#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Mpu6050EkfNode : public rclcpp::Node {
public:
    Mpu6050EkfNode() : Node("mpu6050_ekf_node") {
        // QoS: Best Effort, Volatile (Match Unpacker)
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();

        // Subscribe to Unpacked Imu
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/pico/imu_raw", qos,
            std::bind(&Mpu6050EkfNode::imuCallback, this, std::placeholders::_1));

        // Publish Odometry
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/predicted", 10);

        // Init State
        q_est_ = Eigen::Quaterniond::Identity();
        last_time_ = this->now();
        first_msg_ = true;
        
        RCLCPP_INFO(this->get_logger(), "MPU6050 EKF Node Started (Standard Message Mode)");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time current_time = msg->header.stamp;
        
        if (first_msg_) {
            last_time_ = current_time;
            first_msg_ = false;
            return;
        }

        // Calculate dt
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Sanity Check
        if (dt <= 0.0 || dt > 0.1) {
            // Force 1ms if jitter is bad or clocks are weird, 
            // but ideally we trust the timestamp from unpacker (which is back-calculated correctly)
            // If unpacker uses Ros Time Now, it might be jittery.
            // But unpacker tries to smooth it?
            // Unpacker uses: now - offset.
            return; 
        }

        // Gyro is already rad/s in standard message
        double gx = msg->angular_velocity.x;
        double gy = msg->angular_velocity.y;
        double gz = msg->angular_velocity.z;

        // Predict Step (Simple Gyro Integration)
        Eigen::Vector3d omega(gx, gy, gz);
        double theta = omega.norm() * dt;
        
        Eigen::Quaterniond q_delta;
        if (theta > 1e-6) {
            Eigen::Vector3d axis = omega.normalized();
             q_delta = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
        } else {
             // Small angle approx
             q_delta = Eigen::Quaterniond(1.0, 0.5*dt*omega.x(), 0.5*dt*omega.y(), 0.5*dt*omega.z()).normalized();
        }

        q_est_ = q_est_ * q_delta;
        q_est_.normalize();

        // Publish
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.orientation.w = q_est_.w();
        odom.pose.pose.orientation.x = q_est_.x();
        odom.pose.pose.orientation.y = q_est_.y();
        odom.pose.pose.orientation.z = q_est_.z();
        
        // Twist (in body frame)
        odom.twist.twist.angular.x = gx;
        odom.twist.twist.angular.y = gy;
        odom.twist.twist.angular.z = gz;

        pub_odom_->publish(odom);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    Eigen::Quaterniond q_est_;
    rclcpp::Time last_time_;
    bool first_msg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mpu6050EkfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Mpu6050EkfNode : public rclcpp::Node {
public:
    Mpu6050EkfNode() : Node("mpu6050_ekf_node") {
        // QoS: Best Effort, Volatile
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();

        // Subscribe to Pico IMU
        sub_imu_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/pico/imu", qos,
            std::bind(&Mpu6050EkfNode::imuCallback, this, std::placeholders::_1));

        // Publish Odometry
        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/predicted", 10);

        // Init State
        q_est_ = Eigen::Quaterniond::Identity();
        last_timestamp_us_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "MPU6050 EKF Node Started");
    }

private:
    void imuCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 7) return;

        // Extract Data
        uint64_t timestamp_us = (uint64_t)msg->data[0];
        float ax = msg->data[1]; // Raw int16 cast to float, needs scaling? 
                                 // Firmware sends raw int16? 
                                 // Wait, MPU6050 raw is int16.
                                 // 4g range: 8192 LSB/g.
                                 // 2000dps range: 16.4 LSB/dps.
        float ay = msg->data[2];
        float az = msg->data[3];
        float gx = msg->data[4];
        float gy = msg->data[5];
        float gz = msg->data[6];

        // Sanity Check Timestamp
        if (last_timestamp_us_ == 0) {
            last_timestamp_us_ = timestamp_us;
            return;
        }

        double dt = (double)(timestamp_us - last_timestamp_us_) / 1000000.0;
        last_timestamp_us_ = timestamp_us;

        // Jitter Compensation / Sanity Clamp
        if (dt <= 0.0 || dt > 0.1) {
            // Drop packet or assume 1ms?
            // EKF explodes if dt is bad.
            // Let's assume dropped packet if dt > 0.1?
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Bad dt: %f", dt);
            return; 
        }

        // Convert Range
        // Gyro: deg/s -> rad/s
        double gx_rad = (gx / 16.4) * (M_PI / 180.0);
        double gy_rad = (gy / 16.4) * (M_PI / 180.0);
        double gz_rad = (gz / 16.4) * (M_PI / 180.0);

        // Predict Step (Simple Gyro Integration)
        // q_new = q_old * q_delta
        // q_delta from angular velocity
        Eigen::Vector3d omega(gx_rad, gy_rad, gz_rad);
        double theta = omega.norm() * dt;
        
        Eigen::Quaterniond q_delta;
        if (theta > 1e-6) {
            Eigen::Vector3d axis = omega.normalized();
             q_delta = Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
        } else {
             // Small angle approx
             q_delta = Eigen::Quaterniond(1.0, 0.5*dt*omega.x(), 0.5*dt*omega.y(), 0.5*dt*omega.z()).normalized();
        }

        q_est_ = q_est_ * q_delta;
        q_est_.normalize();

        // Publish
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now(); // Wall time for ROS, or strictly valid time?
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.orientation.w = q_est_.w();
        odom.pose.pose.orientation.x = q_est_.x();
        odom.pose.pose.orientation.y = q_est_.y();
        odom.pose.pose.orientation.z = q_est_.z();
        
        // Also put rates in twist
        odom.twist.twist.angular.x = gx_rad;
        odom.twist.twist.angular.y = gy_rad;
        odom.twist.twist.angular.z = gz_rad;

        pub_odom_->publish(odom);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    Eigen::Quaterniond q_est_;
    uint64_t last_timestamp_us_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mpu6050EkfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
