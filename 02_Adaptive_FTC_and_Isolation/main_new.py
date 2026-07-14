import numpy as np
import mujoco
import mujoco.viewer
from controller_FTC import NMPCController
from filter_aug_stiffness import UnscentedKalmanFilter
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

def plot_stiffness_evolution(kstiff_list, dt):
    """
    Plots the estimated stiffness evolution over time for all 4 joints.
    """
    time_axis = np.arange(len(kstiff_list)) * dt
    kstiff_arr = np.array(kstiff_list)
    
    plt.figure(figsize=(10, 5))
    joint_labels = ['Joint 1 (Base)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)', 'Joint 4 (Wrist)']
    for i in range(4):
        plt.plot(time_axis, kstiff_arr[:, i], label=joint_labels[i])
        
    plt.axhline(y=50000.0, color='k', linestyle='--', alpha=0.5, label='Nominal Stiffness')
    plt.title('Estimated Joint Stiffness Evolution Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Stiffness (N/m or Nm/rad)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig("figure/fig_stiffness_estimation.png", dpi=150, bbox_inches='tight')
    plt.show()

def main():
    model = mujoco.MjModel.from_xml_path('3DoFarm.xml')
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    dt = 0.02
    model.opt.timestep = 0.0002
    controller = NMPCController(dt=dt, horizon=20)
    ukf = UnscentedKalmanFilter(dt=dt) 
    
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
    kstiff_history = []  # --- NEW: Stiffness log history ---
    joint_names = ['Joint 1 (Base)', 'Joint 2 (Shoulder)', 'Joint 3 (Elbow)', 'Joint 4 (Wrist)']

    print("Starting Diagnostic Run. Close the viewer window to process frequency analysis.")

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
            model.jnt_stiffness[spring_joint_id] = 5000.0  # Softened stiffness level
            
            raw_sensor_bus = np.array(data.sensordata)
            noisy_pos = raw_sensor_bus[0:4] + np.random.normal(0, 0.003, 4)
            noisy_vel = raw_sensor_bus[4:8] + np.random.normal(0, 0.01, 4)
            z_meas = np.concatenate((noisy_pos, noisy_vel))

            ukf.predict(u_prev, M_inv, q_ref_joints)
            q_est, dq_est, kstiff_est = ukf.update(z_meas)  #

            try:
                optimal_acc = controller.solve(q_est, dq_est, dynamic_target_pos, kstiff_est, M_inv, q_ref_joints)
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

            torque_ripple = fault_amplitude * np.sin(2.0 * np.pi * fault_frequency * sim_time)
            applied_torque[1] += torque_ripple

            data.ctrl[:4] = applied_torque

            for _ in range(100):
                mujoco.mj_step(model, data)
                
            current_acc_residual = (data.qacc[[0, 1, 3, 4]] - optimal_acc).copy()
            M_inv_history.append([M_inv[1, 1], M_inv[2, 1]])
            kstiff_history.append(kstiff_est.copy())  # --- NEW: Log live stiffness ---
            
            current_torque_residual = M_arm @ current_acc_residual

            acc_history.append(current_acc_residual)
            torque_history.append(current_torque_residual)
            torque_command.append(optimal_acc)

            if len(acc_history) % 50 == 0:
                print(f"Simulation Time: {sim_time:.2f} seconds")
            
            viewer.sync()

    effective_sample_rate = 1.0 / (dt)
    warmup_samples = int(3 / dt)
    if len(acc_history) > warmup_samples + 100:
        raw_acc = acc_history[warmup_samples:]
        raw_torque = torque_history[warmup_samples:]
        
        clean_acc = high_pass_filter(raw_acc, cutoff_hz=2.0, fs=effective_sample_rate)
        clean_torque = high_pass_filter(raw_torque, cutoff_hz=2.0, fs=effective_sample_rate)
        
        clean_M_inv = M_inv_history[warmup_samples:]
        clean_kstiff = kstiff_history[warmup_samples:]  # --- NEW: Warmup slice for stiffness ---
        clean_torque_com = torque_command[warmup_samples:]
        
        run_frequency_diagnostics(clean_acc, clean_torque, effective_sample_rate, fault_frequency, joint_names)
        plot_inertia_evolution(clean_M_inv, effective_sample_rate)
        plot_timeseries(clean_torque_com, effective_sample_rate)
        plot_stiffness_evolution(clean_kstiff, dt)  # --- NEW: Plot stiffness ---

        calculate_sensitivity()
    else:
        sim_time_achieved = len(acc_history) * dt
        print(f"\nSimulation ended too quickly! You only generated {sim_time_achieved:.1f}s of data.")
        print(f"You need at least {3+ (100*dt)}s of simulation time to run this diagnostic.")

if __name__ == '__main__':
    main()