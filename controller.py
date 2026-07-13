import casadi as ca
import numpy as np
from Kinematic import KinematicsEngine

class NMPCController:
    def __init__(self, dt=0.02, horizon=20):
        self.dt = dt
        self.N = horizon
        self.kin = KinematicsEngine()
        self.opti = ca.Opti()
        # Initialize the optimization variables
        self.X = self.opti.variable(8, self.N + 1)
        self.U = self.opti.variable(4, self.N)
        
    def solve(self, q_curr, dq_curr, target_pos):
        # Reset and clear previous constraints
        X = self.opti.variable(8, self.N + 1)
        U = self.opti.variable(4, self.N)
        
        # FIX 1: Increase safe radius! 
        # (0.05 EE radius + 0.04 Obstacle radius + 0.21 safety buffer)
        safe_radius_sq = 0.3**2
        
        cost = 0
    

          # WARM START: Provide a reasonable initial guess so the solver doesn't start at absolute zero!
        self.opti.set_initial(X, ca.repmat(ca.vertcat(q_curr, dq_curr), 1, self.N + 1))
    

        
        q_home = ca.vertcat(0.0, 0,0,0)

        current_ee_pos = self.kin.forward_kinematics_sym(q_curr)
        dist_to_target = ca.norm_2(current_ee_pos - target_pos)
        
        max_dist = 2.0  
        dist_ratio = ca.fmax(0.0, ca.fmin(1.0, dist_to_target / max_dist))
        
        distal_penalty = 0.05 + (0.95 * dist_ratio)
        shoulder_penalty = 0.5 - (0.49 * dist_ratio)
        W_posture = ca.diag(ca.vertcat(0.01, shoulder_penalty, distal_penalty, distal_penalty))
        
        # --- 1. THE RUNNING COST & CONSTRAINTS (The Journey) ---
        for k in range(self.N):
            ee_pos = self.kin.forward_kinematics_sym(X[:4, k])
            
            cost += ca.sumsqr(ee_pos - target_pos) * 500.0
            cost += ca.sumsqr(U[:, k]) * 0.2

            cost += ca.sumsqr(X[4:, k]) * 0.2

            
            # Postural Objective
            posture_error = X[:4, k] - q_home
            cost += ca.mtimes([posture_error.T, W_posture, posture_error])
            
            # Explicit Euler Dynamics
            q_next = X[:4, k] + X[4:, k] * self.dt
            dq_next = X[4:, k] + U[:, k] * self.dt
            self.opti.subject_to(X[:, k+1] == ca.vertcat(q_next, dq_next))
            
            self.opti.subject_to(self.opti.bounded(-3.14, X[:4, k], 3.14))
            
        # --- 2. THE TERMINAL COST (The Strategic Goal) ---
        ee_final_pos = self.kin.forward_kinematics_sym(X[:4, self.N])
        cost += ca.sumsqr(ee_final_pos - target_pos) * 1000.0 
            
        self.opti.minimize(cost)
        self.opti.subject_to(X[:, 0] == ca.vertcat(q_curr, dq_curr))
            
        # Solver setup
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.opti.solver('ipopt', opts)
        
        try:
            sol = self.opti.solve()
            return sol.value(U[:, 0])
        except Exception as e:
            print("\n=========================================")
            print("🚨 SOLVER CRASHED! ANALYZING VIOLATIONS...")
            print("=========================================")
            # This CasADi debug function prints exactly which constraints failed
            # The '1e-5' means it will only show violations larger than 0.00001
            self.opti.debug.show_infeasibilities(1e-5)
            print("=========================================\n")
            
            # Re-raise the error so main.py can catch it and apply 0 acceleration
            raise e