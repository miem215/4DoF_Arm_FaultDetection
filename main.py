import numpy as np
import mujoco
import mujoco.viewer
from controller import NMPCController
from filter import UnscentedKalmanFilter
from scipy.signal import welch, find_peaks
import matplotlib.pyplot as plt

def get_constant_speed_joint_ref(t, amplitude, freq_hz, offset=0.0):
    """
    Generates a mathematically continuous constant-speed triangle wave for joint space.
    """
    omega = 2.0 * np.pi * freq_hz
    q_target = (2.0 * amplitude / np.pi) * np.arcsin(np.sin(omega * t)) + offset
    return q_target

def run_frequency_diagnostics(residual_history, sample_rate, fault_freq_hz, joint_names):
    """
    Processes logged residuals using Welch's Method to isolate the fault frequency.
    """
    residuals = np.array(residual_history)
    num_joints = residuals.shape[1]
    fig, axes = plt.subplots(num_joints, 2, figsize=(12, 2.5 * num_joints))
    
    print("\n--- Running Interconnected System Diagnostics ---")
    for i in range(num_joints):
        res_data = residuals[:, i]
        
        # Time Domain Plot
        axes[i, 0].plot(res_data, color='#555555', alpha=0.8)
        axes[i, 0].set_title(f'{joint_names[i]} - Velocity Residuals')
        axes[i, 0].set_ylabel('Error (rad/s)')
        
        # Frequency Domain Plot (Power Spectral Density)
        freqs, psd = welch(res_data, fs=sample_rate, nperseg=min(1024, len(res_data)))
        axes[i, 1].plot(freqs, psd, color='#1f77b4')
        axes[i, 1].set_title(f'{joint_names[i]} - Power Spectral Density')
        axes[i, 1].set_ylabel('Power')
        axes[i, 1].set_xlim(0, sample_rate / 2)
        
        # Peak Detection/Fault Isolation Logic
        peaks, _ = find_peaks(psd, height=np.max(psd) * 0.25)
        peak_freqs = freqs[peaks]
        fault_detected = any(np.isclose(f, fault_freq_hz, atol=0.4) for f in peak_freqs)
        
        if fault_detected:
            axes[i, 1].axvline(fault_freq_hz, color='red', linestyle='--', label='Fault Frequency')
            axes[i, 1].legend()
            print(f"[ALERT] Mechanical fault isolated in {joint_names[i]} at {fault_freq_hz} Hz!")
            
    plt.tight_layout()
    plt.savefig("figure/fig_analysis.png", dpi=150, bbox_inches='tight')
    plt.show()

def main():
    # 1. Setup
    model = mujoco.MjModel.from_xml_path('3DoFarm.xml') # Holds your 4-DoF actuator setup
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    obs_mocap_id = model.body('obstacle').mocapid[0]

    # MATCH THE TIMESTEPS
    dt = 0.02
    controller = NMPCController(dt=dt, horizon=20)
    ukf = UnscentedKalmanFilter(dt=dt) 
    
    # Initialize control tracking profiles
    u_prev = np.zeros(4)
    
    # Diagnostic Trajectory Parameters (Simulating Hexapod Constant Speed Sweep)
    sweep_amplitude = 0.4  # Radians
    sweep_frequency = 0.2  # Hz (Back and forth cycle speed)
    joint_2_offset = 0.8   # Nominal center position for the sweeping joint
    
    # Fault Injection Parameters (Simulating a 6.0 Hz ballscrew/nut defect on Joint 2)
    fault_frequency = 6.0  # Hz
    fault_amplitude = 1.8  # Torque disturbance magnitude
    
    # Data logging for frequency analysis
    residual_history = []
    joint_names = ['Joint 1 (Base)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)', 'Joint 4 (Wrist)']

    print("Starting Diagnostic Run. Close the viewer window to process frequency analysis.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            sim_time = data.time
            
            # --- 1. SENSOR (Hardware Reality with Noise) ---
            raw_sensor_bus = np.array(data.sensordata)
            noisy_pos = raw_sensor_bus[0:4] + np.random.normal(0, 0.003, 4) 
            noisy_vel = raw_sensor_bus[4:8] + np.random.normal(0, 0.01, 4)
            z_meas = np.concatenate((noisy_pos, noisy_vel))

            # --- 2. STATE ESTIMATION (The UKF Pipeline) ---
            ukf.predict(u_prev)
            q_est, dq_est = ukf.update(z_meas)

            # --- 3. CONSTANT-SPEED REFERENCE GENERATION ---
            # Generate the constant speed profile for Joint 2 (Shoulder)
            q_ref_joints = np.zeros(4)
            q_ref_joints[0] = 0.0
            q_ref_joints[1] = get_constant_speed_joint_ref(sim_time, sweep_amplitude, sweep_frequency, joint_2_offset)
            q_ref_joints[2] = -0.6  # Hold stationary to capture cross-coupled fault propagation
            q_ref_joints[3] = 0.0
            
            # Map the joint space reference trajectory to Cartesian target positions for the NMPC
            dynamic_target_pos = np.array(controller.kin.forward_kinematics_sym(q_ref_joints)).flatten()

            # Keep the background obstacle tracking active
            data.mocap_pos[obs_mocap_id][1] = np.sin(sim_time * 8.0) * 0.25
            obs_pos = data.mocap_pos[obs_mocap_id].copy()

            # --- 4. CONTROL LOOP ---
            try:
                optimal_acc = controller.solve(q_est, dq_est, dynamic_target_pos, obs_pos)
            except Exception as e:
                print(f"Solver failed: {e}")
                optimal_acc = np.zeros(4)

            u_prev = optimal_acc.copy()

            # --- 5. REAL-WORLD FAULT INJECTION & PHYSICS UPDATE ---
            data.qacc[:4] = optimal_acc
            mujoco.mj_inverse(model, data)
            
            # Calculate nominal feedback torque
            applied_torque = data.qfrc_inverse[:4].copy()
            
            # INJECT HARDWARE FAULT: Add periodic torque ripples to Joint 2 (index 1) to simulate physical wear
            torque_ripple = fault_amplitude * np.sin(2.0 * np.pi * fault_frequency * sim_time)
            applied_torque[1] += torque_ripple
            
            data.ctrl[:4] = applied_torque
            
            for _ in range(10):
                mujoco.mj_step(model, data)
                
            # --- 6. LOGGING VELOCITY RESIDUALS ---
            # Quantify the deviation between the noisy sensor reality and what the clean UKF model expected
            current_velocity_residual = noisy_vel - dq_est
            residual_history.append(current_velocity_residual)
            
            viewer.sync()

    # --- 7. POST-RUN DIAGNOSTICS ---
    # Sampling rate calculation: 10 simulation steps per loop step at dt=0.02
    effective_sample_rate = 1.0 / (dt) 
    warmup_samples = int(2.0 / dt)
    if len(residual_history) > warmup_samples + 100:
        run_frequency_diagnostics(residual_history, effective_sample_rate, fault_frequency, joint_names)
    else:
        print("Simulation ended too quickly to generate diagnostic plots.")

if __name__ == '__main__':
    main()     