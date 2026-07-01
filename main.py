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
    Compares absolute power across all joints to find the root cause of the vibration.
    """
    residuals = np.array(residual_history)
    num_joints = residuals.shape[1]
    fig, axes = plt.subplots(num_joints, 2, figsize=(12, 2.5 * num_joints))
    
    print("\n--- Running Interconnected System Diagnostics ---")
    
    fault_powers = [] # Track the strength of the fault in each joint
    
    for i in range(num_joints):
        res_data = residuals[:, i]
        
        # Time Domain Plot
        axes[i, 0].plot(res_data, color='#555555', alpha=0.8, linewidth=1)
        axes[i, 0].set_title(f'{joint_names[i]} - Acceleration Residuals')
        axes[i, 0].set_ylabel('Error (rad/s²)')
        
        # Frequency Domain Plot (Absolute Power Spectrum)
        # Added scaling='spectrum' to calculate true physical magnitude
        freqs, psd = welch(res_data, fs=sample_rate, nperseg=min(1024, len(res_data)), scaling='spectrum')
        axes[i, 1].plot(freqs, psd, color='#1f77b4', linewidth=1.5)
        axes[i, 1].set_title(f'{joint_names[i]} - Power Spectrum')
        axes[i, 1].set_ylabel('Power')
        axes[i, 1].set_xlim(0, 25) 
        
        # Find the specific power at our fault frequency (6.0 Hz)
        fault_bin_index = np.argmin(np.abs(freqs - fault_freq_hz))
        power_at_fault = psd[fault_bin_index]
        fault_powers.append(power_at_fault)
        
        axes[i, 1].axvline(fault_freq_hz, color='red', linestyle='--', alpha=0.5, label='Known Fault Freq')
        axes[i, 1].legend()

    plt.tight_layout()
    plt.savefig("figure/fig_analysis.png", dpi=150, bbox_inches='tight')
    plt.show()

    # --- ROOT CAUSE ISOLATION LOGIC ---
    root_cause_index = np.argmax(fault_powers)
    
    print("\n[DIAGNOSTIC REPORT]")
    for i in range(num_joints):
        power = fault_powers[i]
        # Calculate true physical magnitude (amplitude) in rad/s²
        magnitude = np.sqrt(2 * power) 
        print(f"{joint_names[i]} 6.0Hz Power: {power:.6e} | Accel Ripple Magnitude: {magnitude:.6f} rad/s²")
        
    print(f"\n[ALERT] Root cause physically isolated to: {joint_names[root_cause_index]}")

def main():
    # 1. Setup
    model = mujoco.MjModel.from_xml_path('3DoFarm.xml') # Holds your 4-DoF actuator setup
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

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
    

            # --- 4. CONTROL LOOP ---
            try:
                optimal_acc = controller.solve(q_est, dq_est, dynamic_target_pos)
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
            current_acc_residual = (data.qacc[:4] - optimal_acc).copy()
            residual_history.append(current_acc_residual)

            if len(residual_history) % 50 == 0:
                print(f"Simulation Time: {sim_time:.2f} seconds")
            
            viewer.sync()

    # --- 7. POST-RUN DIAGNOSTICS ---
    # Sampling rate calculation: 10 simulation steps per loop step at dt=0.02
    effective_sample_rate = 1.0 / (dt) 
    warmup_samples = int(15.0 / dt)
    if len(residual_history) > warmup_samples + 100:
        clean_history = residual_history[warmup_samples:]
        run_frequency_diagnostics(clean_history, effective_sample_rate, fault_frequency, joint_names)
    else:
        sim_time_achieved = len(residual_history) * dt
        print(f"\nSimulation ended too quickly! You only generated {sim_time_achieved:.1f}s of data.")
        print(f"You need at least {15.0 + (100*dt)}s of simulation time to run this diagnostic.")

if __name__ == '__main__':
    main()     