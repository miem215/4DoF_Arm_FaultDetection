import numpy as np
import mujoco
import matplotlib.pyplot as plt
from scipy.linalg import inv

def extract_state_space(xml_path, stiffness_value):
    """
    Uses MuJoCo's finite differencing to linearize the plant dynamics.
    Returns the A and B state-space matrices around the zero-equilibrium.
    """
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # Apply the target stiffness to the flexible joint
    spring_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint2_flex')
    model.jnt_stiffness[spring_joint_id] = stiffness_value
    
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    
    nv = model.nv
    nu = model.nu
    nx = 2 * nv  # State vector is [velocities, accelerations] in MuJoCo's derivative
    
    A = np.zeros((nx, nx))
    B = np.zeros((nx, nu))
    
    # Calculate finite difference Jacobians
    mujoco.mjd_transitionFD(model, data, 1e-5, 1, A, B, None, None)
    
    # Define C matrix to map full state [q, dq] to actuated joint positions
    # Assuming the first 4 states correspond to the 4 actuators for simplicity
    dt = model.opt.timestep
    A_cont = (A - np.eye(nx)) / dt
    B_cont = B / dt
    
    # Define C matrix to map full state [q, dq] to actuated joint positions
    C = np.zeros((nu, nx))
    for i in range(nu):
        C[i, i] = 1.0 
        
    # Return the continuous matrices instead!
    return A_cont, B_cont, C, nu, nv

def calculate_sensitivity():
    xml_file = '3DoFarm.xml'
    
    # Extract linear models for both states
    A_healthy, B_healthy, C_healthy, nu, nv = extract_state_space(xml_file, 50000.0)
    A_degraded, B_degraded, C_degraded, _, _ = extract_state_space(xml_file, 5000.0)
    
    # Frequency range for analysis: 0.1 Hz to 25.0 Hz
    frequencies = np.linspace(0.1, 25.0, 500)
    s_healthy_mag = []
    s_degraded_mag = []
    
    # Controller Approximation K(s)
    # The NMPC acts aggressively on position (cost=500). We approximate this 
    # locally as a PD controller mapping position error to torque.
    Kp = 500.0 * np.eye(nu)
    Kd = 50.0 * np.eye(nu)
    
    print("Computing Frequency Responses...")
    
    for f in frequencies:
        omega = 2.0 * np.pi * f
        s = 1j * omega
        
        # 1. Controller Frequency Response: K(jw) = Kp + jw*Kd
        K_jw = Kp + s * Kd
        
        # 2. Plant Frequency Response: G(jw) = C * (sI - A)^-1 * B
        sI_healthy = s * np.eye(2 * nv) - A_healthy
        G_healthy = C_healthy @ inv(sI_healthy) @ B_healthy
        
        sI_degraded = s * np.eye(2 * nv) - A_degraded
        G_degraded = C_degraded @ inv(sI_degraded) @ B_degraded
        
        # 3. Output Sensitivity: S(jw) = (I + G(jw)K(jw))^-1
        I_nu = np.eye(nu)
        S_healthy = inv(I_nu + G_healthy @ K_jw)
        S_degraded = inv(I_nu + G_degraded @ K_jw)
        
        # 4. Extract the magnitude for Joint 2 (Index 1)
        # Using the absolute value of the diagonal element for Joint 2
        s_healthy_mag.append(np.abs(S_healthy[1, 1]))
        s_degraded_mag.append(np.abs(S_degraded[1, 1]))

    # --- Plotting the Bode Magnitude of Sensitivity ---
    plt.figure(figsize=(10, 6))
    
    # Plotting on a logarithmic scale (Bode plot standard)
    plt.plot(frequencies, 20 * np.log10(s_healthy_mag), label='Healthy (Stiffness 50k)', color='#1f77b4', linewidth=2)
    plt.plot(frequencies, 20 * np.log10(s_degraded_mag), label='Degraded (Stiffness 5k)', color='#d62728', linewidth=2)
    
    plt.axvline(6.0, color='black', linestyle='--', alpha=0.7, label='6.0 Hz Fault Frequency')
    plt.axhline(0.0, color='gray', linestyle=':', alpha=0.5)
    
    plt.title('Closed-Loop Sensitivity Function $|S(j\omega)|$ - Joint 2')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude (dB)')
    plt.xlim(0, 25)
    plt.ylim(-60, 20)
    plt.legend(loc='best')
    plt.grid(True, which="both", ls="--", alpha=0.5)
    
    plt.tight_layout()
    plt.savefig("figure/sensitivity_analysis.png", dpi=150)
    plt.show()

if __name__ == '__main__':
    calculate_sensitivity()