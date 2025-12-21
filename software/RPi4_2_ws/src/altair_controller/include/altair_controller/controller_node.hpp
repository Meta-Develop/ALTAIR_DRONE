#ifndef ALTAIR_CONTROLLER_NODE_HPP
#define ALTAIR_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <atomic>

// Acados C Interfaces
extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_altair_drone.h"
}

namespace altair_controller {

enum class ControlMode {
    PID_MIXER,
    NMPC,
    HIERARCHICAL
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
    void control_mode_callback(const std_msgs::msg::Int8::SharedPtr msg);
    
    // Control Loops
    void control_loop(void); // 250Hz
    void mpc_thread_func(void); // 20Hz

    // Logic
    Eigen::VectorXf calculate_pid_output(const Eigen::VectorXd& state, const Eigen::VectorXd& setpoint);


    // ROS Interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_control_mode_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_esc_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_servos_;
    
    rclcpp::TimerBase::SharedPtr timer_control_;
    
    // State & Config
    ControlMode mode_;
    std::string urdf_path_;
    
    // Watchdog
    rclcpp::Time last_cmd_time_;
    bool emergency_mode_;

    // Geometry / Physics
    double mass_;
    double arm_length_, k_t_, k_m_;
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
    Eigen::VectorXf mpc_next_x_; // Next optimal state from MPC (for feedback)
    std::mutex mpc_mutex_;

    // Acados Structs
    altair_drone_solver_capsule* capsule_;
    ocp_nlp_config* nlp_config_;
    ocp_nlp_dims* nlp_dims_;
    ocp_nlp_in* nlp_in_;
    ocp_nlp_out* nlp_out_;
    ocp_nlp_solver* nlp_solver_;

    // PID Gains
    double kp_pos_, kd_pos_, kp_att_, kd_att_;
    double kp_nmpc_; 
};

} // namespace altair_controller

#endif // ALTAIR_CONTROLLER_NODE_HPP
