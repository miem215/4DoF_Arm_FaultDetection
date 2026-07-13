import numpy as np
import mujoco
import mujoco.viewer
from controller_FTC import NMPCController
from filter_aug_stiffness_iso import UnscentedKalmanFilter  # Must contain the Bank/active_idx logic
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
from diagnostics import run_frequency_diagnostics, plot_inertia_evolution, plot_timeseries
from sensitivity_analysis import calculate_sensitivity

def get_constant_speed_joint_ref(t, amplitude, freq_hz, offset=0.0):
    omega = 2.0 * np.pi * freq_hz
    q_target = amplitude * np.sin(omega * t) + offset
    return q_target

def high_pass_filter(data_list, cutoff_hz=2.0, fs=50.0, order=4):
    data_arr = np.array(data_list)
    nyq = 0.5 * fs
    normal_cutoff = cutoff_hz / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    filtered = filtfilt(b, a, data_arr, axis=0)
    return filtered

def plot_isolation_results(time_history, residual_history, decision_history, stiffness_history):
    """
    Plots the Fault Detection, Isolation, and Identification (FDII) logic.
    """
    res_arr = np.array(residual_history)
    stiff_arr = np.array(stiffness_history)
    dec_arr = np.array(decision_history)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    
    # Plot 1: Observer Residuals (The Detection & Isolation Metric)
    labels = ['Obs 0 (Base)', 'Obs 1 (Shoulder)', 'Obs 2 (Elbow)', 'Obs 3 (Wrist)']
    colors = ['blue', 'orange', 'green', 'red']
    for i in range(4):
        ax1.plot(time_history, res_arr[:, i], label=labels[i], color=colors[i], alpha=0.8)
    ax1.set_title('Observer Bank Residuals (Lowest Error = True Physics)')
    ax1.set_ylabel('Innovation Norm')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Active Fault Isolation Decision
    ax2.scatter(time_history, dec_arr, color='purple', s=10)
    ax2.set_yticks([0, 1, 2, 3])
    ax2.set_yticklabels(['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4'])
    ax2.set_title('Supervisory Logic: Isolated Fault Location')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Estimated Stiffness of the Winning Observer
    winning_stiffness = [stiff_arr[i, dec_arr[i]] for i in range(len(dec_arr))]
    ax3.plot(time_history, winning_stiffness, color='black', label='Identified Degradation')
    ax3.axhline(50000.0, color='gray', linestyle='--', label='Nominal Baseline (50k)')
    ax3.axhline(5000.0, color='red', linestyle=':', label='True Fault Severity (5k)')
    ax3.set_title('Identified Degradation Magnitude (FTC Input)')
    ax3.set_ylabel('Stiffness (N/m)')
    ax3.set_xlabel('Time (s)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig("figure/fig_isolation_results.png", dpi=150, bbox_inches='tight')
    plt.show()

def main():
    model = mujoco.MjModel.from_xml_path('3DoFarm.xml')
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    dt = 0.02
    model.opt.timestep = 0.0002
    
    controller = NMPCController(dt=dt, horizon=20)
    
    # --- MULTIPLE MODEL OBSERVER BANK ---
    ukf_bank = [UnscentedKalmanFilter(active_idx=i, dt=dt) for i in range(4)]
    
    u_prev = np.zeros(4)
    sweep_amplitude = 0.4
    sweep_frequency = 0.2
    joint_2_offset = 0.8
    
    fault_frequency = 6.0
    fault_amplitude = 1.8
    
    acc_history = []
    torque_history = []
    torque_command = []
    M_inv_history = []
    joint_names = ['Joint 1 (Base)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)', 'Joint 4 (Wrist)']

    # --- FDI Loggers ---
    time_hist = []
    residual_hist = []
    decision_hist = []
    stiffness_hist = []
    
    smoothed_residuals = np.ones(4) * 0.5  # High initial value to force convergence
    alpha_res = 0.05  # EMA smoothing factor for residuals

    print("Starting Diagnostic Run with Observer Bank. Close the viewer window to process analysis.")

    spring_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint2_flex')

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            sim_time = data.time

            nv = model.nv 
            M_dense = np.zeros((nv, nv))
            mujoco.mj_fullM(model, M_dense, data.qM)

            actuated_indices = [0, 1, 3, 4]
            M_arm = M_dense[np.ix_(actuated_indices, actuated_indices)]
            M_inv = np.linalg.inv(M_arm)

            q_ref_joints = np.zeros(4)
            q_ref_joints[0] = 0.0
            q_ref_joints[1] = get_constant_speed_joint_ref(sim_time, sweep_amplitude, sweep_frequency, joint_2_offset)
            q_ref_joints[2] = -0.6  
            q_ref_joints[3] = 0.0

            dynamic_target_pos = np.array(controller.kin.forward_kinematics_sym(q_ref_joints)).flatten()
            
            # --- INJECT FAULT: Softened stiffness level ---
            model.jnt_stiffness[spring_joint_id] = 5000.0  
            
            raw_sensor_bus = np.array(data.sensordata)
            noisy_pos = raw_sensor_bus[0:4] + np.random.normal(0, 0.003, 4)
            noisy_vel = raw_sensor_bus[4:8] + np.random.normal(0, 0.01, 4)
            z_meas = np.concatenate((noisy_pos, noisy_vel))

            # --- 1. RUN THE OBSERVER BANK ---
            current_residuals = np.zeros(4)
            current_stiffness = np.zeros(4)
            state_estimates = []
            
            for i in range(4):
                ukf_bank[i].predict(u_prev, M_inv, q_ref_joints)
                # Unpack the 4 returns from the updated filter
                q_est_i, dq_est_i, kstiff_i, res_norm_i = ukf_bank[i].update(z_meas)
                
                current_residuals[i] = res_norm_i
                current_stiffness[i] = kstiff_i
                state_estimates.append((q_est_i, dq_est_i))

            # --- 2. SUPERVISORY LOGIC: ISOLATE FAULT ---
            smoothed_residuals = (1 - alpha_res) * smoothed_residuals + alpha_res * current_residuals
            isolated_idx = int(np.argmin(smoothed_residuals))
            
            best_q, best_dq = state_estimates[isolated_idx]
            best_k = current_stiffness[isolated_idx]

            # --- 3. ACTIVE FAULT-TOLERANT CONTROL ---
            try:
                optimal_acc = controller.solve(best_q, best_dq, dynamic_target_pos, best_k, M_inv, q_ref_joints)
            except Exception as e:
                print(f"Solver failed: {e}")
                optimal_acc = np.zeros(4)

            u_prev = optimal_acc.copy()

            data.qacc[0] = optimal_acc[0]  
            data.qacc[1] = optimal_acc[1]  
            data.qacc[3] = optimal_acc[2]  
            data.qacc[4] = optimal_acc[3]  

            mujoco.mj_inverse(model, data)

            applied_torque = np.zeros(4)
            applied_torque[0] = data.qfrc_inverse[0]
            applied_torque[1] = data.qfrc_inverse[1]
            applied_torque[2] = data.qfrc_inverse[3] 
            applied_torque[3] = data.qfrc_inverse[4] 

            pos_error = q_ref_joints - noisy_pos
            vel_error = -noisy_vel
            pd_stabilization = (200.0 * pos_error) + (20.0 * vel_error)
            applied_torque += pd_stabilization

            # --- INJECT FAULT: 6Hz Ripple ---
            torque_ripple = fault_amplitude * np.sin(2.0 * np.pi * fault_frequency * sim_time)
            applied_torque[1] += torque_ripple

            data.ctrl[:4] = applied_torque

            for _ in range(100):
                mujoco.mj_step(model, data)
                
            current_acc_residual = (data.qacc[[0, 1, 3, 4]] - optimal_acc).copy()
            M_inv_history.append([M_inv[1, 1], M_inv[2, 1]])
            
            # --- LOGGING ---
            current_torque_residual = M_arm @ current_acc_residual
            acc_history.append(current_acc_residual)
            torque_history.append(current_torque_residual)
            torque_command.append(optimal_acc)
            
            if sim_time > 0.5:  # Skip startup transients
                time_hist.append(sim_time)
                residual_hist.append(smoothed_residuals.copy())
                decision_hist.append(isolated_idx)
                stiffness_hist.append(current_stiffness.copy())

            if len(acc_history) % 50 == 0:
                print(f"Time: {sim_time:.2f}s | Isolated Joint: {isolated_idx+1} | Tracking Stiffness: {best_k:.0f}")
            
            viewer.sync()

    # --- FINAL POST-PROCESSING ---
    effective_sample_rate = 1.0 / dt
    warmup_samples = int(3 / dt)
    
    if len(acc_history) > warmup_samples + 100:
        raw_acc = acc_history[warmup_samples:]
        raw_torque = torque_history[warmup_samples:]
        
        clean_acc = high_pass_filter(raw_acc, cutoff_hz=2.0, fs=effective_sample_rate)
        clean_torque = high_pass_filter(raw_torque, cutoff_hz=2.0, fs=effective_sample_rate)
        
        clean_M_inv = M_inv_history[warmup_samples:]
        clean_torque_com = torque_command[warmup_samples:]
        
        run_frequency_diagnostics(clean_acc, clean_torque, effective_sample_rate, fault_frequency, joint_names)
        plot_inertia_evolution(clean_M_inv, effective_sample_rate)
        plot_timeseries(clean_torque_com, effective_sample_rate)
        
        # Plot the FDI Bank logic
        plot_isolation_results(time_hist, residual_hist, decision_hist, stiffness_hist)
        
        calculate_sensitivity()
    else:
        sim_time_achieved = len(acc_history) * dt
        print(f"\nSimulation ended too quickly! You generated {sim_time_achieved:.1f}s of data.")

if __name__ == '__main__':
    main()