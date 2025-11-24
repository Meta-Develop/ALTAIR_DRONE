#ifndef ALTAIR_CONTROLLER_NODE_HPP
#define ALTAIR_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <atomic>

namespace altair_controller {

enum class ControlMode {
    PID_MIXER,
    NMPC
};

class AltairController : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AltairController();
    ~AltairController();

private:
    // Initialization
    void load_urdf_parameters();
    void compute_allocation_matrix();

    // Callbacks
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Control Loops
    void control_loop(void); // 250Hz
    void mpc_thread_func(void); // 20Hz

    // Logic
    Eigen::VectorXf calculate_pid_output(const Eigen::VectorXd& state, const Eigen::VectorXd& setpoint);
    Eigen::VectorXf calculate_nmpc_output();

    // ROS Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_actuators_;
    
    rclcpp::TimerBase::SharedPtr timer_control_;
    
    // State & Config
    ControlMode mode_;
    std::string urdf_path_;
    
    // Watchdog
    rclcpp::Time last_cmd_time_;
    bool emergency_mode_;

    // Geometry / Physics
    double mass_;
    Eigen::Matrix3d inertia_;
    Eigen::MatrixXf allocation_matrix_; // 12x6 (Actuators x Wrench)

    // Data Buffers (Mutex protected if shared)
    std::mutex state_mutex_;
    Eigen::VectorXd current_state_; // [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
    Eigen::VectorXd target_setpoint_; // [vx, vy, vz, wz] (from cmd_vel)
    
    // NMPC specific
    std::thread mpc_thread_;
    std::atomic<bool> mpc_running_;
    Eigen::VectorXf mpc_next_u_; // Next control input from MPC
    std::mutex mpc_mutex_;

    // PID Gains
    double kp_pos_, kd_pos_, kp_att_, kd_att_;
};

} // namespace altair_controller

#endif // ALTAIR_CONTROLLER_NODE_HPP
