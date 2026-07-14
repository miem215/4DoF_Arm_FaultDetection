import numpy as np
import scipy.linalg

class UnscentedKalmanFilter:
    def __init__(self, active_idx, dt=0.02):
        self.dt = dt
        self.active_idx = active_idx  # 0, 1, 2, or 3
        self.n_x = 9   # 4 pos, 4 vel, 1 target stiffness
        self.n_z = 8   # 4 pos, 4 vel
        
        self.alpha = 1e-3  
        self.beta = 2.0    
        self.kappa = 0.0   
        self.lambda_ = self.alpha**2 * (self.n_x + self.kappa) - self.n_x
        
        self.x = np.zeros(self.n_x) 
        self.x[8] = 50000.0  # Start assuming nominal health                  
        self.P = np.diag([0.01]*4 + [0.05]*4 + [1000.0])    
        
        self.Q = np.diag([1e-6]*4 + [1e-5]*4 + [500.0])
        self.R = np.diag([0.005**2]*4 + [0.02**2]*4)
        
        self.Wc, self.Wm = self._compute_weights()

    def _compute_weights(self):
        Wm = np.zeros(2 * self.n_x + 1)
        Wc = np.zeros(2 * self.n_x + 1)
        Wm[0] = self.lambda_ / (self.n_x + self.lambda_)
        Wc[0] = (self.lambda_ / (self.n_x + self.lambda_)) + (1 - self.alpha**2 + self.beta)
        weight = 1.0 / (2 * (self.n_x + self.lambda_))
        Wm[1:] = weight
        Wc[1:] = weight
        return Wc, Wm

    def _generate_sigma_points(self, x, P):
        A = scipy.linalg.cholesky((self.n_x + self.lambda_) * P, lower=True)
        Xsig = np.zeros((self.n_x, 2 * self.n_x + 1))
        Xsig[:, 0] = x
        for i in range(self.n_x):
            Xsig[:, i + 1] = x + A[:, i]
            Xsig[:, i + 1 + self.n_x] = x - A[:, i]
        return Xsig 

    def _system_dynamics(self, state, u, Minv, q_nominal):
        q = state[0:4]
        dq = state[4:8]
        kstiff_active = state[8]

        steps = 10
        dt_sub = self.dt / steps
        for _ in range(steps):
            tau_spring = np.zeros(4)
            for i in range(4):
                # Apply the estimated stiffness to the active joint, nominal to the rest
                k = kstiff_active if i == self.active_idx else 50000.0
                tau_spring[i] = -k * (q[i] - q_nominal[i]) - (100.0 * dq[i])
                
            accel_spring = Minv @ tau_spring
            total_accel = u + accel_spring
            
            dq = dq + total_accel * dt_sub
            q = q + dq * dt_sub
            
        return np.concatenate((q, dq, [kstiff_active]))

    def predict(self, u, Minv, q_nominal):
        self.P = 0.5 * (self.P + self.P.T) 
        vals, vecs = np.linalg.eigh(self.P)
        vals = np.maximum(vals, 1e-4)      
        self.P = vecs @ np.diag(vals) @ vecs.T
        self.P = 0.5 * (self.P + self.P.T)
        
        Xsig = self._generate_sigma_points(self.x, self.P)
        self.Xsig_prd = np.zeros((self.n_x, 2 * self.n_x + 1))
        for i in range(2 * self.n_x + 1):
            self.Xsig_prd[:, i] = self._system_dynamics(Xsig[:, i], u, Minv, q_nominal)
            
        self.x = np.zeros(self.n_x)
        for i in range(2 * self.n_x + 1):
            self.x += self.Wm[i] * self.Xsig_prd[:, i]
            
        self.P = np.copy(self.Q)
        for i in range(2 * self.n_x + 1):
            x_diff = self.Xsig_prd[:, i] - self.x
            self.P += self.Wc[i] * np.outer(x_diff, x_diff)

    def update(self, z):
        Zsig = self.Xsig_prd[0:8, :]
        z_prior = np.zeros(self.n_z)
        for i in range(2 * self.n_x + 1):
            z_prior += self.Wm[i] * Zsig[:, i]

        S = np.copy(self.R)
        T = np.zeros((self.n_x, self.n_z))
        for i in range(2 * self.n_x + 1):
            z_diff = Zsig[:, i] - z_prior
            x_diff = self.Xsig_prd[:, i] - self.x
            S += self.Wc[i] * np.outer(z_diff, z_diff)
            T += self.Wc[i] * np.outer(x_diff, z_diff)

        S = 0.5 * (S + S.T) + np.eye(self.n_z) * 1e-6
        K_gain = T @ np.linalg.inv(S)
    
        innovation = z - z_prior
        innovation_norm = np.linalg.norm(innovation)  # Calculate the score!
        
        self.x = self.x + K_gain @ innovation
        P_upd = self.P - K_gain @ S @ K_gain.T
        self.P = 0.5 * (P_upd + P_upd.T)
        
        vals, vecs = np.linalg.eigh(self.P)
        vals = np.maximum(vals, 1e-4)
        self.P = vecs @ np.diag(vals) @ vecs.T
        
        return self.x[0:4], self.x[4:8], self.x[8], innovation_norm