import casadi as ca
import numpy as np
from Kinematic import KinematicsEngine

class NMPCController:
    def __init__(self, dt=0.02, horizon=20):
        self.dt = dt
        self.N = horizon
        self.kin = KinematicsEngine()
        
        # Define the permanent optimization problem graph ONCE
        self.opti = ca.Opti()
        
        # Decision variables over the horizon (pure rigid body states)
        self.X = self.opti.variable(8, self.N + 1)
        self.U = self.opti.variable(4, self.N)
        
        # Declare explicit PARAMETERS that change every time step
        self.p_q_curr = self.opti.parameter(4)
        self.p_dq_curr = self.opti.parameter(4)
        self.p_target_pos = self.opti.parameter(3)
        self.p_kstiff_est = self.opti.parameter(4)
        self.p_M_inv = self.opti.parameter(4, 4)
        self.p_q_ref = self.opti.parameter(4)
        
        self._build_optimization_graph()

    def _build_optimization_graph(self):
        cost = 0
        
        self.opti.subject_to(self.X[:, 0] == ca.vertcat(self.p_q_curr, self.p_dq_curr))
        
        for k in range(self.N):
            ee_pos = self.kin.forward_kinematics_sym(self.X[:4, k])
            
            # 1. Cartesian Tracking Cost
            cost += ca.sumsqr(ee_pos - self.p_target_pos) * 500.0
            
            # 2. Posture Tracking Cost
            cost += ca.sumsqr(self.X[:4, k] - self.p_q_ref) * 10.0
            
            # 3. Safe Cost-Scaling based on estimated stiffness
            stiffness_ratio = ca.fmax(self.p_kstiff_est / 50000.0, 0.05)
            cost += ca.sumsqr(self.U[:, k] / stiffness_ratio) * 0.2
            cost += ca.sumsqr(self.X[4:, k]) * 0.2
            
            # 4. Pure, smooth rigid-body integration (100% immune to chattering/cholesky failure)
            dq_next = self.X[4:, k] + self.U[:, k] * self.dt
            q_next = self.X[:4, k] + dq_next * self.dt
            
            self.opti.subject_to(self.X[:, k+1] == ca.vertcat(q_next, dq_next))
            self.opti.subject_to(self.opti.bounded(-6.0, self.X[:4, k], 6.0))
            
        ee_final_pos = self.kin.forward_kinematics_sym(self.X[:4, self.N])
        cost += ca.sumsqr(ee_final_pos - self.p_target_pos) * 1000.0 
        
        self.opti.minimize(cost)
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.opti.solver('ipopt', opts)

    def solve(self, q_curr, dq_curr, target_pos, kstiff_est, M_inv, q_ref):
        self.opti.set_value(self.p_q_curr, q_curr)
        self.opti.set_value(self.p_dq_curr, dq_curr)
        self.opti.set_value(self.p_target_pos, target_pos)
        self.opti.set_value(self.p_kstiff_est, kstiff_est)
        self.opti.set_value(self.p_M_inv, M_inv)
        self.opti.set_value(self.p_q_ref, q_ref)
        
        self.opti.set_initial(self.X, ca.repmat(ca.vertcat(q_curr, dq_curr), 1, self.N + 1))
        
        try:
            sol = self.opti.solve()
            return sol.value(self.U[:, 0])
        except Exception as e:
            q_error = q_ref - q_curr
            dq_error = -dq_curr
            fallback_u = 20.0 * q_error + 5.0 * dq_error
            return fallback_u