import yaml

def export_altair_model():
    model_name = 'altair_drone'

    # Load Parameters
    params_file = os.path.join(os.path.dirname(__file__), '../config/motor_params.yaml')
    motor_params = {}
    if os.path.exists(params_file):
        with open(params_file, 'r') as f:
            motor_params = yaml.safe_load(f)
        print(f"Loaded motor params from {params_file}")
    else:
        print("Warning: motor_params.yaml not found, using defaults.")

    # Defaults
    mass = motor_params.get('mass_override', 2.0)
    g = 9.81
    
    # Inertia
    inertia = motor_params.get('inertia_diag', [0.05, 0.05, 0.1])
    J = ca.diag(inertia)
    
    # ... (States/Dynamics definition per previous) ...
    # Re-use existing dynamics block, just make sure mass is used.

    # States
    p = ca.SX.sym('p', 3) # Position
    v = ca.SX.sym('v', 3) # Velocity
    q = ca.SX.sym('q', 4) # Quaternion (w, x, y, z)
    w = ca.SX.sym('w', 3) # Body Rates

    x = ca.vertcat(p, v, q, w)

    # Controls (Wrench: Fx, Fy, Fz, Tx, Ty, Tz in BODY frame)
    u_F = ca.SX.sym('u_F', 3)
    u_T = ca.SX.sym('u_T', 3)
    u = ca.vertcat(u_F, u_T)

    # Rotation Matrix from Quaternion
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    R = ca.vertcat(
        ca.horzcat(1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw),
        ca.horzcat(2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw),
        ca.horzcat(2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2)
    )

    p_dot = v
    v_dot = (R @ u_F) / mass + ca.vertcat(0, 0, -g)
    
    # q_dot
    Omega = ca.vertcat(
        ca.horzcat(0, -w[0], -w[1], -w[2]),
        ca.horzcat(w[0], 0, w[2], -w[1]),
        ca.horzcat(w[1], -w[2], 0, w[0]),
        ca.horzcat(w[2], w[1], -w[0], 0)
    )
    q_dot = 0.5 * Omega @ q

    # w_dot
    w_dot = ca.inv(J) @ (u_T - ca.cross(w, J @ w))

    f_expl = ca.vertcat(p_dot, v_dot, q_dot, w_dot)

    model = AcadosModel()
    model.f_impl_expr = x - x 
    model.f_expl_expr = f_expl
    model.x = x
    model.u = u
    model.name = model_name
    return model, motor_params

def create_ocp_solver():
    ocp = AcadosOcp()
    model, params = export_altair_model()
    ocp.model = model

    Tf = 1.0
    N = 20
    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # Cost
    Q_mat = 2 * np.diag([10, 10, 10, 1, 1, 1, 10, 10, 10, 10, 1, 1, 1]) 
    ocp.cost.cost_type = 'AUTO'
    ocp.cost.cost_type_e = 'AUTO'
    ocp.cost.W = np.diag([10, 10, 10,  1, 1, 1,  0, 0, 0, 0,  1, 1, 1,  1, 1, 1, 1, 1, 1])
    ocp.cost.W_e = np.diag([10, 10, 10, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1])
    ocp.cost.vx = np.zeros((19, 13))
    ocp.cost.vx[:13, :] = np.eye(13)
    ocp.cost.vu = np.zeros((19, 6))
    ocp.cost.vu[13:, :] = np.eye(6)
    ocp.cost.vx_e = np.eye(13)
    ocp.cost.yref = np.zeros((19,))
    ocp.cost.yref_e = np.zeros((13,))

    # Constraints
    # Input Constraints (Hard - Physical Limits)
    # Calculate Limits from Motor Params
    # F_z_max = 6 * max_thrust
    # T_max = ? (Simplified: 2 Nm)
    
    max_thrust = params.get('max_thrust', 7.0)
    F_z_max = 6 * max_thrust
    # Fx, Fy limits (due to tilt) -> Assume 50% of thrust? 
    # Or just use same max, solver handles allocation feasibility implicitly?
    # No, hard constraints on U must be valid.
    F_lat_max = F_z_max * 0.5 
    
    T_max = 2.0 # Placeholder or calc from arm_length * F * 0.5?
    
    ocp.constraints.lbu = np.array([-F_lat_max, -F_lat_max, 0,       -T_max, -T_max, -T_max])
    ocp.constraints.ubu = np.array([ F_lat_max,  F_lat_max, F_z_max,  T_max,  T_max,  T_max])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5])

    # State Constraints (Soft)
    v_max = 5.0
    w_max = 10.0
    ocp.constraints.lbx = np.array([-v_max, -v_max, -v_max, -w_max, -w_max, -w_max])
    ocp.constraints.ubx = np.array([ v_max,  v_max,  v_max,  w_max,  w_max,  w_max])
    ocp.constraints.idxbx = np.array([3, 4, 5, 10, 11, 12])

    # Soft Constraints Formulation
    # Slack variables 'sl' (lower) and 'su' (upper)
    ns = 6 # Number of soft constraints (3 vel + 3 rate)
    ocp.constraints.lsbx = np.zeros(ns) # Lower slack bounds (usually 0)
    ocp.constraints.usbx = np.zeros(ns) # Upper slack bounds (usually 0)
    ocp.constraints.idxsbx = np.array([0, 1, 2, 3, 4, 5]) # Indices in lbx/ubx to make soft
    
    # Slack Penalties (L1 and L2 norms)
    # Cost += zl * sl + zu * su + 0.5 * Zl * sl^2 + 0.5 * Zu * su^2
    ocp.cost.Zl = 100.0 * np.ones(ns) # Quadratic penalty L
    ocp.cost.Zu = 100.0 * np.ones(ns) # Quadratic penalty U
    ocp.cost.zl = 10.0 * np.ones(ns)  # Linear penalty L
    ocp.cost.zu = 10.0 * np.ones(ns)  # Linear penalty U

    # Solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    acados_source_path = os.environ.get('ACADOS_SOURCE_DIR')
    
    return ocp

if __name__ == '__main__':
    ocp = create_ocp_solver()
    # ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
    print("Acados OCP definition generated. Run with acados python env to generate C code.")
