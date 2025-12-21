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
    this->declare_parameter("arm_length", 0.40);
    this->declare_parameter("k_t", 1.0);
    this->declare_parameter("k_m", 0.02);

    mpc_running_ = false;
    capsule_ = nullptr;
    
    std::string mode_str = this->get_parameter("mode").as_string();
    if (mode_str == "NMPC") {
        mode_ = ControlMode::NMPC;
        RCLCPP_INFO(this->get_logger(), "Mode: NMPC");
    } else {
        mode_ = ControlMode::PID_MIXER;
        RCLCPP_INFO(this->get_logger(), "Mode: PID Mixer");
    }

    this->get_parameter("arm_length", arm_length_);
    this->get_parameter("k_t", k_t_);
    this->get_parameter("k_m", k_m_);
    
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

    sub_control_mode_ = this->create_subscription<std_msgs::msg::Int8>(
        "/control/mode", reliable_qos,
        std::bind(&AltairController::control_mode_callback, this, std::placeholders::_1));

    // Publishers
    pub_esc_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/control/esc_commands", sensor_qos);
    pub_servos_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/control/servo_commands", sensor_qos);

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

    // NMPC Gains
    kp_nmpc_ = 1.0; 
    mpc_next_x_ = Eigen::VectorXf::Zero(12);

    // Acados Init
    RCLCPP_INFO(this->get_logger(), "Initializing Acados Solver...");
    capsule_ = altair_drone_acados_create_capsule();
    int status = altair_drone_acados_create(capsule_);
    if (status) {
        RCLCPP_ERROR(this->get_logger(), "altair_drone_acados_create() failed: %d", status);
    } else {
        nlp_config_ = altair_drone_acados_get_nlp_config(capsule_);
        nlp_dims_   = altair_drone_acados_get_nlp_dims(capsule_);
        nlp_in_     = altair_drone_acados_get_nlp_in(capsule_);
        nlp_out_    = altair_drone_acados_get_nlp_out(capsule_);
        nlp_solver_ = altair_drone_acados_get_nlp_solver(capsule_);
        RCLCPP_INFO(this->get_logger(), "Acados Solver Initialized Successfully.");
    }
}

AltairController::~AltairController() {
    if (mpc_running_) {
        mpc_running_ = false;
        if (mpc_thread_.joinable()) mpc_thread_.join();
    }
    
    // Free Acados
    if (capsule_) {
        altair_drone_acados_free(capsule_);
        altair_drone_acados_free_capsule(capsule_);
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
    
    // Geometry from Parameters
    float d = arm_length_; 
    float k_t = k_t_; 
    float k_m = k_m_; 
    
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
        
        // Tilting creates tangential force, which creates Yaw Torque
        // Tz = Force * d (Direction depends on definition, assume positive tilt = positive tangential force CCW?)
        // If Tilt > 0 -> F_tangential > 0 (CCW) -> Tz > 0
        M(5, idx_tilt) = d; 
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

void AltairController::control_mode_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    if (msg->data == 0) {
        mode_ = ControlMode::PID_MIXER;
        RCLCPP_INFO(this->get_logger(), "Switched to PID Mixer");
    } else if (msg->data == 1) {
        mode_ = ControlMode::NMPC;
        
        // Start MPC thread if not running
        if (!mpc_running_) {
            mpc_running_ = true;
            mpc_thread_ = std::thread(&AltairController::mpc_thread_func, this);
        }
        
        RCLCPP_INFO(this->get_logger(), "Switched to NMPC");
    } else if (msg->data == 2) {
        mode_ = ControlMode::HIERARCHICAL;
        RCLCPP_INFO(this->get_logger(), "Switched to Hierarchical (Rate Control)");
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid Control Mode: %d", msg->data);
    }
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
        
    } else if (mode_ == ControlMode::HIERARCHICAL) {
        // --- Hierarchical Control ---
        Eigen::VectorXd state, setpoint;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state = current_state_;
            setpoint = target_setpoint_;
        }
        if (emergency_mode_) setpoint.setZero();

        // 1. Force Control
        Eigen::Vector3d v_ref = setpoint.head(3);
        Eigen::Vector3d v_curr = state.segment(6, 3);
        Eigen::Vector3d Kp_vel(4.0, 4.0, 4.0);
        Eigen::Vector3d acc_des = Kp_vel.cwiseProduct(v_ref - v_curr);
        Eigen::Vector3d F_des_world = mass_ * (acc_des + Eigen::Vector3d(0, 0, 9.81));

        double r = state(3), p = state(4), y = state(5);
        Eigen::Matrix3d R_b_w;
        R_b_w = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * 
                Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
        Eigen::Vector3d F_body = R_b_w.transpose() * F_des_world;

        // 2. Attitude -> Rate Setpoint
        // Simplified P-control for Attitude
        double yaw_curr = y;
        Eigen::Vector3d w_ref(0, 0, setpoint(3)); // Feedforward Rate
        
        // Desired Attitude: Flat with target Yaw? Or just Flat?
        // Assuming we want to maintain current Yaw (Rate Controlled)
        // Actually setpoint(3) is Yaw Rate.
        Eigen::Matrix3d R_des = Eigen::AngleAxisd(yaw_curr, Eigen::Vector3d::UnitZ());
        
        // Error
        Eigen::Matrix3d R_err_mat = 0.5 * (R_des.transpose() * R_b_w - R_b_w.transpose() * R_des);
        Eigen::Vector3d att_error(R_err_mat(2, 1), R_err_mat(0, 2), R_err_mat(1, 0)); 
        
        double K_outer = 4.0; 
        Eigen::Vector3d w_cmd = w_ref - K_outer * att_error;

        // Output: [Fx, Fy, Fz, wx, wy, wz]
        u_out = Eigen::VectorXf::Zero(6);
        u_out << F_body(0), F_body(1), F_body(2), w_cmd(0), w_cmd(1), w_cmd(2);

    } else if (mode_ == ControlMode::NMPC) {
        Eigen::VectorXf u_ff(6);
        Eigen::VectorXf x_ref(12);
        
        {
            std::lock_guard<std::mutex> lock(mpc_mutex_);
            u_ff = mpc_next_u_; // [Fx, Fy, Fz, wx, wy, wz]
            x_ref = mpc_next_x_;
        }

        Eigen::VectorXd state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state = current_state_;
        }
        
        Eigen::VectorXf error = x_ref - state.cast<float>();
        
        // Feedback
        Eigen::VectorXf u_fb = Eigen::VectorXf::Zero(6);
        
        // Force Feedback from Velocity Error (indices 6-8)
        u_fb.head(3) = kp_nmpc_ * error.segment(6, 3); 
        
        // Rate Feedback from Rate Error (indices 9-11)
        u_fb.tail(3) = kp_nmpc_ * error.segment(9, 3); 

        u_out = u_ff + u_fb;
    }

    // 5. Ensure we have 12 actuator outputs (Thrusts + Angles)
    // If Mode is NMPC/Hierarchical that outputted Wrench (6), we MUST mix it.
    // However, the current logic for NMPC outputs Wrench (6).
    // The Allocation Matrix is 12x6.
    // u (12) = M * Wrench (6).
    
    // If u_out has only 6 elements (Wrench), apply allocation.
    if (u_out.size() == 6) {
        // Assume it is a Wrench [Fx, Fy, Fz, Tx, Ty, Tz]
        // Apply Mixer
        u_out = allocation_matrix_ * u_out; 
    }

    // Now u_out should be size 12.
    // [0-5]: Motor Throttles (0-1)
    // [6-11]: Servo Angles (0-1 or Radians? Allocation assumes Linear Force ~ Angle)
    // Need to clamp.

    // Publish Split Topics
    auto msg_esc = std_msgs::msg::Float32MultiArray();
    msg_esc.data.resize(6);
    auto msg_servo = std_msgs::msg::Float32MultiArray();
    msg_servo.data.resize(6);

    for(int i=0; i<6; ++i) {
        // Clamp Motor Throttle 0.0 - 1.0
        float val = u_out[i];
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;
        msg_esc.data[i] = val;
        
        // Clamp Servo Command (0.0 - 1.0 maps to Angle Range)
        // Adjust clamping based on Servo limits if needed
        float s_val = u_out[6+i]; // Indices 6-11
        if (s_val < 0.0f) s_val = 0.0f;
        if (s_val > 1.0f) s_val = 1.0f;
        msg_servo.data[i] = s_val;
    }

    pub_esc_->publish(msg_esc);
    pub_servos_->publish(msg_servo);
}

// --- Mock Acados Solver Interface ---
// In a real deployment, this would be replaced by the generated C code from Acados.
// (Mock Solver Removed)



Eigen::VectorXf AltairController::calculate_pid_output(const Eigen::VectorXd& state, const Eigen::VectorXd& setpoint) {
    // Corrected Geometric Controller for VELOCITY Tracking (cmd_vel input)
    // State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
    // Setpoint: [vx_ref, vy_ref, vz_ref, wz_ref]
    
    // 1. Velocity Error (World Frame)
    Eigen::Vector3d v_ref = setpoint.head(3);
    Eigen::Vector3d v_curr = state.segment(6, 3);
    Eigen::Vector3d acc_des = Eigen::Vector3d::Zero();
    
    // Gains
    Eigen::Vector3d Kp_vel(4.0, 4.0, 4.0);
    
    // a_des = Kp * (v_ref - v)
    acc_des = Kp_vel.cwiseProduct(v_ref - v_curr);
    
    // Add Gravity Compensation
    Eigen::Vector3d F_des_world = mass_ * (acc_des + Eigen::Vector3d(0, 0, 9.81));

    // Rotation Matrix (Body to World)
    double r = state(3), p = state(4), y = state(5);
    Eigen::Matrix3d R_b_w;
    R_b_w = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * 
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) * 
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

    // F_body
    Eigen::Vector3d F_body = R_b_w.transpose() * F_des_world;

    // 2. Attitude Control (Orientation)
    // For omnidirectional, we can decouple orientation from force vector direction.
    // Target Orientation: Keep flat (r=0, p=0) unless commanded otherwise? 
    // cmd_vel only gives yaw rate `wz_ref`. 
    // So we assume Target Attitude = Current Yaw, Zero Roll/Pitch.
    // Ideally we integrate wz_ref to get yaw_ref, OR we just control Rate directly (since we are velocity controlled).
    
    // Let's do Rate Control directly for Yaw, and Attitude Hold for Roll/Pitch.
    
    double yaw_curr = y;
    double yaw_ref = yaw_curr; // Maintain current unless moving? 
    // Actually, `wz_ref` implies we want a Rate.
    
    // Target Rates: [0, 0, wz_ref]
    Eigen::Vector3d w_ref(0, 0, setpoint(3));
    Eigen::Vector3d w_curr = state.segment(9, 3);
    
    // Attitude Error (Roll/Pitch stabilization)
    // We want R_des to be Flat with current Yaw.
    Eigen::Matrix3d R_des = Eigen::AngleAxisd(yaw_curr, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_err_mat = 0.5 * (R_des.transpose() * R_b_w - R_b_w.transpose() * R_des);
    Eigen::Vector3d att_error(R_err_mat(2, 1), R_err_mat(0, 2), R_err_mat(1, 0)); 
    
    // Gains
    Eigen::Vector3d Kp_att(8.0, 8.0, 0.0); // No Yaw P-gain here if doing pure rate? 
    // Actually mixed: Roll/Pitch P-control for stability, Yaw P is 0 because we track Rate.
    
    Eigen::Vector3d Kd_rate(2.0, 2.0, 2.0); // Rate D-gains (acts as P for rate loop)
    
    // Torque = -Kp_att * e_R + Kd_rate * (w_ref - w_curr)
    // Note: Kp_att handles leveling. Kd_rate handles angular velocity tracking.
    Eigen::Vector3d T_body = -Kp_att.cwiseProduct(att_error) + Kd_rate.cwiseProduct(w_ref - w_curr);

    // 3. Construct Wrench
    Eigen::VectorXd wrench(6);
    wrench << F_body(0), F_body(1), F_body(2), T_body(0), T_body(1), T_body(2);

    // 4. Allocation
    return allocation_matrix_ * wrench.cast<float>();
}

// Helper to quat
void euler_to_quat(double r, double p, double y, double* q) {
    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cp = cos(p * 0.5);
    double sp = sin(p * 0.5);
    double cr = cos(r * 0.5);
    double sr = sin(r * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z
}

void AltairController::mpc_thread_func() {
    rclcpp::Rate rate(20); // 20Hz NMPC
    
    // Wait for init
    while(rclcpp::ok() && mpc_running_ && !capsule_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    int N = 20; // Default
    if (capsule_) ocp_nlp_dims_get(nlp_config_, nlp_dims_, "N", &N);

    // Buffers
    double acados_x0[13]; // p(3), v(3), q(4), w(3)
    double yref[19];      // 13 + 6
    double u0[6];         // Wrench
    double x1[13];        // Next State

    while (rclcpp::ok() && mpc_running_) {
        // 1. Prepare Input
        Eigen::VectorXd cur_state;
        Eigen::VectorXd cur_setpoint;
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            cur_state = current_state_;
            cur_setpoint = target_setpoint_;
        }
        
        // Convert to Acados State [p, v, q, w]
        // cur_state: [x,y,z, r,p,y, vx,vy,vz, wx,wy,wz]
        acados_x0[0] = cur_state[0]; acados_x0[1] = cur_state[1]; acados_x0[2] = cur_state[2]; // p
        acados_x0[3] = cur_state[6]; acados_x0[4] = cur_state[7]; acados_x0[5] = cur_state[8]; // v
        
        double q_tmp[4];
        euler_to_quat(cur_state[3], cur_state[4], cur_state[5], q_tmp);
        acados_x0[6] = q_tmp[0]; acados_x0[7] = q_tmp[1]; acados_x0[8] = q_tmp[2]; acados_x0[9] = q_tmp[3]; // q
        
        acados_x0[10] = cur_state[9]; acados_x0[11] = cur_state[10]; acados_x0[12] = cur_state[11]; // w

        // Set Initial Condition (lbx=ubx=x0 at stage 0)
        int idxbx0[13];
        for(int i=0; i<13; i++) idxbx0[i] = i;
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", acados_x0);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", acados_x0);

        // Prepare Reference (yref)
        // [p_ref, v_ref, q_ref, w_ref, u_ref]
        // Mapping setpoint: [vx, vy, vz, wz]
        // Position Ref: Current Pos (Drift free velocity control)
        yref[0] = cur_state[0]; yref[1] = cur_state[1]; yref[2] = cur_state[2];
        yref[3] = cur_setpoint[0]; yref[4] = cur_setpoint[1]; yref[5] = cur_setpoint[2];
        
        // Orientation Ref: Flat, Yaw integrates? Or just current yaw?
        // Let's hold current yaw.
        euler_to_quat(0, 0, cur_state[5], q_tmp);
        yref[6] = q_tmp[0]; yref[7] = q_tmp[1]; yref[8] = q_tmp[2]; yref[9] = q_tmp[3];
        
        yref[10] = 0; yref[11] = 0; yref[12] = cur_setpoint[3]; // w_ref
        
        // Input Ref (U_ref): Hover Wrench
        // Fz = mg
        yref[13] = 0; yref[14] = 0; yref[15] = mass_ * 9.81; 
        yref[16] = 0; yref[17] = 0; yref[18] = 0;

        // Apply Reference to all stages
        for(int i=0; i<N; i++) {
             ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "yref", yref);
        }
        // Terminal Reference (x only)
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, N, "yref", yref);

        // 2. Solve
        int status = altair_drone_acados_solve(capsule_);
        
        if (status == 0 || status == 2) { // 2 = Max Iterations but feasible?
            // Get u0
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", u0);
            
            // Get x1 (Next State)
            ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 1, "x", x1);

            std::lock_guard<std::mutex> lock(mpc_mutex_);
            // Update mpc_next_u_ (6 inputs -> Wrench)
            // But Controller expects 12 inputs (Actuators)? 
            // Wait, previous NMPC logic outputted u_ff (12).
            // But if NMPC outputs Wrench (6), we need Allocation here or in loop.
            // Let's assume AltairController now outputs Wrench-based control if NMPC mode?
            // "Update altair_controller to output Wrench/Rates" was done. 
            // In HIerarchical mode, we use Wench.
            // So we can assume u_ff is Wrench (6).
            // But current_u_out logic resizes to 12.
            
            // Pack Command: [Fx, Fy, Fz, wx, wy, wz]
            // F come from Input u0, Rates come from Next State x1
            mpc_next_u_ = Eigen::VectorXf::Zero(6);
            mpc_next_u_[0] = (float)u0[0]; mpc_next_u_[1] = (float)u0[1]; mpc_next_u_[2] = (float)u0[2];
            mpc_next_u_[3] = (float)x1[10]; mpc_next_u_[4] = (float)x1[11]; mpc_next_u_[5] = (float)x1[12];
            
            // Map x1 (Acados 13) back to State (12 Euler)
            // q -> rpy
            // ... conversion ...
            // Simplified: Copy p, v, w. Skip q check for now, use x1 q directly if needed?
            // No, hierarchical feedback uses error = x* - x. Vectors must match.
            // x* needs [x,y,z, r,p,y, vx,vy,vz, wx,wy,wz].
            
            double r, p, y;
            // q to euler
            double qw=x1[6], qx=x1[7], qy=x1[8], qz=x1[9]; // x1 q indices
            double sinr_cosp = 2 * (qw * qx + qy * qz);
            double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
            r = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2 * (qw * qy - qz * qx);
            if (std::abs(sinp) >= 1) p = std::copysign(3.14159 / 2, sinp);
            else p = std::asin(sinp);

            double siny_cosp = 2 * (qw * qz + qx * qy);
            double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
            y = std::atan2(siny_cosp, cosy_cosp);

            mpc_next_x_ = Eigen::VectorXf::Zero(12);
            mpc_next_x_ << x1[0], x1[1], x1[2], r, p, y, x1[3], x1[4], x1[5], x1[10], x1[11], x1[12];
            
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "NMPC Failed: %d", status);
        }
        
        rate.sleep();
    }
}

// (calculate_nmpc_output removed)

} // namespace altair_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<altair_controller::AltairController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
