#include "altair_controller/controller_node.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <chrono>

using namespace std::chrono_literals;

namespace altair_controller {

AltairController::AltairController()
    : Node("altair_controller"),
      mode_(ControlMode::PID_MIXER),
      last_cmd_time_(this->now()),
      emergency_mode_(false)
{
    // Parameters
    this->declare_parameter("mode", "PID");
    this->declare_parameter("urdf_path", "");
    
    std::string mode_str = this->get_parameter("mode").as_string();
    if (mode_str == "NMPC") {
        mode_ = ControlMode::NMPC;
        RCLCPP_INFO(this->get_logger(), "Mode: NMPC");
    } else {
        mode_ = ControlMode::PID_MIXER;
        RCLCPP_INFO(this->get_logger(), "Mode: PID Mixer");
    }

    urdf_path_ = this->get_parameter("urdf_path").as_string();

    // Initialization
    load_urdf_parameters();
    compute_allocation_matrix();

    // QoS
    rclcpp::SensorDataQoS sensor_qos;
    rclcpp::QoS reliable_qos(10);

    // Subscribers
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", reliable_qos, 
        std::bind(&AltairController::cmd_vel_callback, this, std::placeholders::_1));
    
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", sensor_qos, 
        std::bind(&AltairController::odom_callback, this, std::placeholders::_1));

    // Publishers
    pub_actuators_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/control/actuator_commands", sensor_qos);

    // Timers
    timer_control_ = this->create_wall_timer(
        4ms, std::bind(&AltairController::control_loop, this)); // 250Hz

    // NMPC Thread
    if (mode_ == ControlMode::NMPC) {
        mpc_running_ = true;
        mpc_thread_ = std::thread(&AltairController::mpc_thread_func, this);
    }

    // Initialize State
    current_state_ = Eigen::VectorXd::Zero(12);
    target_setpoint_ = Eigen::VectorXd::Zero(4); // vx, vy, vz, wz

    // PID Gains (Dummy values)
    kp_pos_ = 1.0; kd_pos_ = 0.5;
    kp_att_ = 2.0; kd_att_ = 0.1;
}

AltairController::~AltairController() {
    if (mpc_running_) {
        mpc_running_ = false;
        if (mpc_thread_.joinable()) mpc_thread_.join();
    }
}

void AltairController::load_urdf_parameters() {
    urdf::Model model;
    if (urdf_path_.empty()) {
         // Fallback to search relative path if not provided
         // Note: In a real ROS node, this should be passed via launch file.
         // We assume the file exists at the specified location.
         RCLCPP_WARN(this->get_logger(), "URDF path not provided, waiting for parameter...");
         return;
    }

    std::ifstream f(urdf_path_);
    if (!f.good()) {
        RCLCPP_ERROR(this->get_logger(), "URDF file not found at: %s", urdf_path_.c_str());
        return; 
    }

    // Parse XML string (reading file content first)
    std::string xml_string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    
    if (!model.initString(xml_string)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "URDF Parsed successfully: %s", model.getName().c_str());
    
    // Get Mass
    if (model.getRoot()->inertial) {
        mass_ = model.getRoot()->inertial->mass;
        RCLCPP_INFO(this->get_logger(), "Total Mass: %f", mass_);
        // Simplified inertia access
        inertia_ << model.getRoot()->inertial->ixx, model.getRoot()->inertial->ixy, model.getRoot()->inertial->ixz,
                    model.getRoot()->inertial->ixy, model.getRoot()->inertial->iyy, model.getRoot()->inertial->iyz,
                    model.getRoot()->inertial->ixz, model.getRoot()->inertial->iyz, model.getRoot()->inertial->izz;
    } else {
        RCLCPP_WARN(this->get_logger(), "Root link has no inertia, using default.");
        mass_ = 2.0; // Default
        inertia_ = Eigen::Matrix3d::Identity();
    }

    // In a real scenario, we would parse the tree to find rotor transforms.
    // Here we will simulate extracting 6 rotors if specific link names aren't found.
    // Assuming 6-arm geometry for allocation matrix calculation.
}

void AltairController::compute_allocation_matrix() {
    // 12 actuators, 6 Wrench components.
    // Wrench (6x1) = M (6x12) * u (12x1)
    // We want Allocation = pinv(M) -> 12x6.
    
    Eigen::MatrixXf M = Eigen::MatrixXf::Zero(6, 12);
    
    // Simplified Voliro Model around Hover (Tilt=0)
    // We assume 6 arms.
    // Actuators: [Thrust1, Tilt1, Thrust2, Tilt2, ... Thrust6, Tilt6]
    // Geometry: Arms at angles 0, 60, 120, 180, 240, 300.
    // Arm length 'd', Thrust coeff 'kt', Moment coeff 'km'.
    
    float d = 0.25f; // Arm length (approx)
    float k_t = 1.0f; // Thrust coeff
    float k_m = 0.02f; // Drag torque coeff
    
    for (int i = 0; i < 6; ++i) {
        float angle = i * 60.0f * M_PI / 180.0f;
        float cx = cos(angle);
        float cy = sin(angle);
        
        int idx_thrust = i * 2;
        int idx_tilt = i * 2 + 1;
        
        // Thrust i contributes to:
        // Fz: 1
        // Tx: d * sin(angle) * Force
        // Ty: -d * cos(angle) * Force
        // Tz: -1^i * k_m * Force (Drag torque, direction alternates)
        
        M(2, idx_thrust) = 1.0f; // Fz
        M(3, idx_thrust) = d * cy; // Tx
        M(4, idx_thrust) = -d * cx; // Ty
        M(5, idx_thrust) = (i % 2 == 0 ? 1.0f : -1.0f) * k_m; // Tz (Spin direction)

        // Tilt i (small angle approx) contributes to lateral forces?
        // Tilt produces force in plane perpendicular to arm.
        // Tangential force? Radial force?
        // Usually tilt is around the arm axis.
        // Produces lateral force.
        
        // Placeholder for Tilt authority
        M(0, idx_tilt) = -cy; // Fx
        M(1, idx_tilt) = cx;  // Fy
        // Tilting also creates torque
    }

    // Compute Pseudo Inverse (Right Inverse for Fat Matrix)
    // M * u = Wrench -> u = pinv(M) * Wrench
    // pinv(M) can be computed by solving M * X = I
    allocation_matrix_ = M.completeOrthogonalDecomposition().solve(Eigen::MatrixXf::Identity(6, 6));
    
    RCLCPP_INFO(this->get_logger(), "Allocation Matrix Computed (12x6).");
}

void AltairController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_time_ = this->now();
    emergency_mode_ = false;
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_setpoint_ << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z;
}

void AltairController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Map Odometry to State Vector [x,y,z, r,p,y, vx,vy,vz, wx,wy,wz]
    // Orientation q -> r,p,y
    auto q_msg = msg->pose.pose.orientation;
    
    // Manual conversion to Euler (ZYX convention) -> Roll, Pitch, Yaw
    // to ensure consistent -pi to pi range.
    double sinr_cosp = 2 * (q_msg.w * q_msg.x + q_msg.y * q_msg.z);
    double cosr_cosp = 1 - 2 * (q_msg.x * q_msg.x + q_msg.y * q_msg.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q_msg.w * q_msg.y - q_msg.z * q_msg.x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2 * (q_msg.w * q_msg.z + q_msg.x * q_msg.y);
    double cosy_cosp = 1 - 2 * (q_msg.y * q_msg.y + q_msg.z * q_msg.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    current_state_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
                      roll, pitch, yaw,
                      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
                      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

void AltairController::control_loop() {
    // Watchdog
    if ((this->now() - last_cmd_time_).seconds() > 1.0) {
        emergency_mode_ = true;
        // Hover or Land
        // For safety, we zero everything or set z_ref to descend.
    }

    Eigen::VectorXf u_out = Eigen::VectorXf::Zero(12);

    if (mode_ == ControlMode::PID_MIXER) {
        // Fetch state and setpoint
        Eigen::VectorXd state, setpoint;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state = current_state_;
            setpoint = target_setpoint_;
        }
        
        if (emergency_mode_) {
            setpoint.setZero(); // Simplified safety
        }

        u_out = calculate_pid_output(state, setpoint);
        
    } else if (mode_ == ControlMode::NMPC) {
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        u_out = mpc_next_u_;
        // Add high-freq feedback linearization here if needed
    }

    // Publish
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.resize(12);
    for(int i=0; i<12; ++i) msg.data[i] = u_out[i];
    pub_actuators_->publish(msg);
}

Eigen::VectorXf AltairController::calculate_pid_output(const Eigen::VectorXd& state, const Eigen::VectorXd& setpoint) {
    // Simplified Cascaded PID
    // Ref: Velocity Setpoint -> Attitude Setpoint -> Rate Setpoint -> Torque
    // Just dummy implementation for structure
    
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    
    // Fz (Altitude)
    double error_vz = setpoint(2) - state(8);
    wrench(2) = mass_ * 9.81 + error_vz * 5.0; // Feedforward gravity + P gain

    // T (Attitude) - minimal logic
    wrench(3) = 0.0;
    wrench(4) = 0.0;
    wrench(5) = (setpoint(3) - state(11)) * 1.0;

    // Map Wrench to Actuators
    return allocation_matrix_ * wrench.cast<float>();
}

void AltairController::mpc_thread_func() {
    rclcpp::Rate rate(20);
    while (rclcpp::ok() && mpc_running_) {
        // 1. Get State
        Eigen::VectorXd state_for_mpc;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_for_mpc = current_state_;
        }

        // 2. Call Solver (Acados/CasADi) - Placeholder
        // Using state_for_mpc...
        
        // 3. Store result
        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            mpc_next_u_ = Eigen::VectorXf::Zero(12); 
            // Simulate MPC calculation based on state
            // e.g. counteract gravity slightly
            mpc_next_u_[0] = 0.5 + (state_for_mpc(2) < 1.0 ? 0.1 : 0.0); 
        }
        
        rate.sleep();
    }
}

Eigen::VectorXf AltairController::calculate_nmpc_output() {
    return Eigen::VectorXf::Zero(12);
}

} // namespace altair_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<altair_controller::AltairController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
