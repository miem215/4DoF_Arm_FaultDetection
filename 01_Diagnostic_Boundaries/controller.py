import casadi as ca
import numpy as np
from Kinematic import KinematicsEngine

class NMPCController:
    def __init__(self, dt=0.02, horizon=20):
        self.dt = dt
        self.N = horizon
        self.kin = KinematicsEngine()
        
        # Define the permanent optimization problem graph ONCE for maximum speed
        self.opti = ca.Opti()
        
        # Decision variables over the horizon
        self.X = self.opti.variable(8, self.N + 1)
        self.U = self.opti.variable(4, self.N)
        
        # Declare explicit PARAMETERS that update via numerical value-swapping
        self.p_q_curr = self.opti.parameter(4)
        self.p_dq_curr = self.opti.parameter(4)
        self.p_target_pos = self.opti.parameter(3)
        self.p_kstiff_est = self.opti.parameter()  # Scalar stiffness parameter
        
        self._build_optimization_graph()

    def _build_optimization_graph(self):
        cost = 0
        q_home = ca.vertcat(0.0, 0.0, 0.0, 0.0)
        
        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == ca.vertcat(self.p_q_curr, self.p_dq_curr))
        
        for k in range(self.N):
            ee_pos = self.kin.forward_kinematics_sym(self.X[:4, k])
            
            # 1. Cartesian Tracking Cost
            cost += ca.sumsqr(ee_pos - self.p_target_pos) * 500.0
            
            # 2. Control Effort Cost with Stiffness Cost-Scaling
            stiffness_ratio = ca.fmax(self.p_kstiff_est / 50000.0, 0.05)
            cost += ca.sumsqr(self.U[:, k] / stiffness_ratio) * 0.2
            cost += ca.sumsqr(self.X[4:, k]) * 0.2
            
            # 3. Dynamic Posture Weighting
            dist_to_target = ca.norm_2(ee_pos - self.p_target_pos)
            max_dist = 2.0
            dist_ratio = ca.fmax(0.0, ca.fmin(1.0, dist_to_target / max_dist))
            distal_penalty = 0.05 + (0.95 * dist_ratio)
            shoulder_penalty = 0.5 - (0.49 * dist_ratio)
            W_posture = ca.diag(ca.vertcat(0.01, shoulder_penalty, distal_penalty, distal_penalty))
            
            posture_error = self.X[:4, k] - q_home
            cost += ca.mtimes([posture_error.T, W_posture, posture_error])
            
            # 4. Smooth Rigid-Body Integration
            q_next = self.X[:4, k] + self.X[4:, k] * self.dt
            dq_next = self.X[4:, k] + self.U[:, k] * self.dt
            
            self.opti.subject_to(self.X[:, k+1] == ca.vertcat(q_next, dq_next))
            self.opti.subject_to(self.opti.bounded(-3.14, self.X[:4, k], 3.14))
            
        # Terminal Cost
        ee_final_pos = self.kin.forward_kinematics_sym(self.X[:4, self.N])
        cost += ca.sumsqr(ee_final_pos - self.p_target_pos) * 1000.0 
        
        self.opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.opti.solver('ipopt', opts)

    def solve(self, q_curr, dq_curr, target_pos, kstiff_est=50000.0):
        # Update parameter values instantly without re-allocating the graph
        self.opti.set_value(self.p_q_curr, q_curr)
        self.opti.set_value(self.p_dq_curr, dq_curr)
        self.opti.set_value(self.p_target_pos, target_pos)
        self.opti.set_value(self.p_kstiff_est, kstiff_est)
        
        # Warm start initialization
        self.opti.set_initial(self.X, ca.repmat(ca.vertcat(q_curr, dq_curr), 1, self.N + 1))
        
        try:
            sol = self.opti.solve()
            return sol.value(self.U[:, 0])
        except Exception as e:
            q_error = q_home_err = -q_curr
            fallback_u = 20.0 * q_error - 5.0 * dq_curr
            return fallback_u