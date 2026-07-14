import numpy as np
import mujoco
from controller import NMPCController
from filter import UnscentedKalmanFilter
from scipy.signal import welch
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

def get_constant_speed_joint_ref(t, amplitude, freq_hz, offset=0.0):
    omega = 2.0 * np.pi * freq_hz
    return amplitude * np.sin(omega * t) + offset

def main():
    model = mujoco.MjModel.from_xml_path('3DoFarm.xml')
    spring_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint2_flex')

    dt = 0.02
    model.opt.timestep = 0.0002
    
    sweep_amplitude = 0.4
    sweep_frequency = 0.2
    joint_2_offset = 0.8
    fault_frequency = 6.0
    fault_amplitude = 1.8
    
    # 1. Define the Degradation Sweep
    stiffness_vals = np.linspace(50000, 5000, 10)
    psd_list = []
    freqs = None
    
    print("Starting Headless Batch Sweep...")

    # 2. Iterate through each stiffness state
    for K in stiffness_vals:
        print(f"Testing Stiffness: {K:.0f} Nm/rad")
        
        # Reset physics and controllers for each run
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        
        model.jnt_stiffness[spring_joint_id] = K
        
        controller = NMPCController(dt=dt, horizon=20)
        ukf = UnscentedKalmanFilter(dt=dt) 
        u_prev = np.zeros(4)
        
        joint2_torque_residual_history = []
        
        # 3. Headless Simulation Loop (e.g., 11.5 seconds)
        while data.time < 11.5:
            sim_time = data.time
            
            # --- SENSOR & ESTIMATION ---
            raw_sensor_bus = np.array(data.sensordata)
            noisy_pos = raw_sensor_bus[0:4] + np.random.normal(0, 0.003, 4) 
            noisy_vel = raw_sensor_bus[4:8] + np.random.normal(0, 0.01, 4)
            z_meas = np.concatenate((noisy_pos, noisy_vel))

            ukf.predict(u_prev)
            q_est, dq_est = ukf.update(z_meas)

            # --- REFERENCE ---
            q_ref_joints = np.zeros(4)
            q_ref_joints[1] = get_constant_speed_joint_ref(sim_time, sweep_amplitude, sweep_frequency, joint_2_offset)
            q_ref_joints[2] = -0.6
            
            dynamic_target_pos = np.array(controller.kin.forward_kinematics_sym(q_ref_joints)).flatten()

            # --- CONTROL ---
            try:
                optimal_acc = controller.solve(q_est, dq_est, dynamic_target_pos)
            except Exception:
                optimal_acc = np.zeros(4)
            u_prev = optimal_acc.copy()

            # --- PHYSICS & FAULT INJECTION ---
            data.qacc[0] = optimal_acc[0]
            data.qacc[1] = optimal_acc[1]
            data.qacc[2] = 0.0
            data.qacc[3] = optimal_acc[2]
            data.qacc[4] = optimal_acc[3]

            mujoco.mj_inverse(model, data)

            applied_torque = np.zeros(4)
            applied_torque[0] = data.qfrc_inverse[0]
            applied_torque[1] = data.qfrc_inverse[1] + (fault_amplitude * np.sin(2.0 * np.pi * fault_frequency * sim_time))
            applied_torque[2] = data.qfrc_inverse[3]
            applied_torque[3] = data.qfrc_inverse[4]

            data.ctrl[:4] = applied_torque

            for _ in range(100):
                mujoco.mj_step(model, data)
                
            # --- LOGGING ---
            current_acc_residual = (data.qacc[[0, 1, 3, 4]] - optimal_acc).copy()
            
            nv = model.nv 
            M_dense = np.zeros((nv, nv))
            mujoco.mj_fullM(model, M_dense, data.qM)
            M_arm = M_dense[np.ix_([0, 1, 3, 4], [0, 1, 3, 4])]
            
            current_torque_residual = M_arm @ current_acc_residual
            
            # We only care about saving Joint 2 (Index 1) for the waterfall plot
            joint2_torque_residual_history.append(current_torque_residual[1])

        # 4. Post-Process the Run (Welch PSD)
        warmup_samples = int(3 / dt)
        clean_torque = joint2_torque_residual_history[warmup_samples:]
        
        f, Pxx = welch(clean_torque, fs=(1/dt), nperseg=256)
        freqs = f
        psd_list.append(Pxx)

    print("Sweep Complete. Generating Plot...")

    # --- 5. RENDER WATERFALL PLOT ---
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    psd_list_db = []
    for Pxx in psd_list:
        # Add 1e-10 to prevent math errors if a value is exactly 0
        Pxx_db = 10 * np.log10(Pxx + 1e-10)
        psd_list_db.append(Pxx_db)
        
    # Find the global minimum and maximum for setting the Z-axis boundaries
    z_min = np.min(psd_list_db)
    z_max = np.max(psd_list_db)

    verts = []
    for i, Pxx_db in enumerate(psd_list_db):
        # 2. Anchor the edges of the polygons to the z_min floor, not 0
        xs = np.concatenate([[freqs[0]], freqs, [freqs[-1]]])
        ys = np.concatenate([[z_min], Pxx_db, [z_min]])
        verts.append(list(zip(xs, ys)))

    poly = PolyCollection(verts, cmap='coolwarm', edgecolors='k', linewidths=0.5, alpha=0.8)
    poly.set_array(stiffness_vals)
    ax.add_collection3d(poly, zs=stiffness_vals, zdir='y')

    ax.set_xlim(0, 26)
    ax.set_ylim(5000, 50000)
    
    # 3. Update Z-axis limits and labels for the dB scale
    ax.set_zlim(z_min, z_max * 1.1) 

    ax.set_xlabel('Frequency (Hz)', fontsize=12, labelpad=10)
    ax.set_ylabel('Joint 2 Stiffness (Nm/rad)', fontsize=12, labelpad=10)
    ax.set_zlabel('PSD Magnitude (dB)', fontsize=12, labelpad=10)
    ax.set_title('Spectral Masking Boundary: Stiffness Degradation', fontsize=14, pad=20)

    ax.invert_yaxis()
    ax.view_init(elev=25, azim=-55)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()