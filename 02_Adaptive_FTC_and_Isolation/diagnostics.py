import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import welch


def run_frequency_diagnostics(acc_history, torque_history, sample_rate, fault_freq_hz, joint_names):
    """
    Processes both acceleration and torque residuals to prove the necessity of 
    inverse inertia mapping for isolating faults in MIMO systems.
    """
    acc_res = np.array(acc_history)
    torque_res = np.array(torque_history)
    num_joints = acc_res.shape[1]
    
    # Create a 4x4 grid: Acc Time | Acc PSD | Torque Time | Torque PSD
    fig, axes = plt.subplots(num_joints, 4, figsize=(24, 2.5 * num_joints))
    
    print("\n--- Running Interconnected System Diagnostics ---")
    
    acc_powers = []
    torque_powers = []
    
    for i in range(num_joints):
        a_data = acc_res[:, i]
        t_data = torque_res[:, i]
        
        # --- 1. Acceleration Time Domain ---
        axes[i, 0].plot(a_data, color='#555555', alpha=0.8, linewidth=1)
        axes[i, 0].set_title(f'{joint_names[i]} - Accel Residuals')
        axes[i, 0].set_ylabel('Error (rad/s²)')
        
        # --- 2. Acceleration Frequency Domain ---
        freqs_a, psd_a = welch(a_data, fs=sample_rate, nperseg=min(1024, len(a_data)), scaling='spectrum')
        axes[i, 1].plot(freqs_a, psd_a, color='#d62728', linewidth=1.5) 
        axes[i, 1].set_title(f'{joint_names[i]} - Accel Spectrum')
        axes[i, 1].set_ylabel('Power')
        axes[i, 1].set_xlim(0, 25)
        
        a_fault_idx = np.argmin(np.abs(freqs_a - fault_freq_hz))
        acc_powers.append(psd_a[a_fault_idx])
        axes[i, 1].axvline(fault_freq_hz, color='black', linestyle='--', alpha=0.5)

        # --- 3. Torque Time Domain ---
        axes[i, 2].plot(t_data, color='#555555', alpha=0.8, linewidth=1)
        axes[i, 2].set_title(f'{joint_names[i]} - Torque Residuals')
        axes[i, 2].set_ylabel('Error (Nm)')
        
        # --- 4. Torque Frequency Domain ---
        freqs_t, psd_t = welch(t_data, fs=sample_rate, nperseg=min(1024, len(t_data)), scaling='spectrum')
        axes[i, 3].plot(freqs_t, psd_t, color='#1f77b4', linewidth=1.5) 
        axes[i, 3].set_title(f'{joint_names[i]} - Torque Spectrum')
        axes[i, 3].set_ylabel('Power')
        axes[i, 3].set_xlim(0, 25)
        
        t_fault_idx = np.argmin(np.abs(freqs_t - fault_freq_hz))
        torque_powers.append(psd_t[t_fault_idx])
        axes[i, 3].axvline(fault_freq_hz, color='black', linestyle='--', alpha=0.5)

    plt.tight_layout()
    plt.savefig("figure/fig_combined_analysis.png", dpi=150, bbox_inches='tight')
    plt.show()

    # --- ROOT CAUSE ISOLATION LOGIC ---
    root_acc_idx = np.argmax(acc_powers)
    root_torque_idx = np.argmax(torque_powers)
    
    print("\n[DIAGNOSTIC REPORT: ACCELERATION vs. TORQUE]")
    print(f"{'Joint':<20} | {'Accel Ripple (rad/s²)':<25} | {'Torque Ripple (Nm)':<20}")
    print("-" * 70)
    
    for i in range(num_joints):
        acc_mag = np.sqrt(2 * acc_powers[i])
        torque_mag = np.sqrt(2 * torque_powers[i])
        print(f"{joint_names[i]:<20} | {acc_mag:<25.6f} | {torque_mag:<20.6f}")
        
    print("\n[ISOLATION RESULTS]")
    print(f"Algorithm via Acceleration  -> Flagged: {joint_names[root_acc_idx]} (Kinematic Amplification Trap)")
    print(f"Algorithm via Torque        -> Flagged: {joint_names[root_torque_idx]} (True Root Cause)")


def plot_inertia_evolution(M_inv_history, sample_rate):
    """
    Visualizes the mathematical proof of the 'whip effect' by tracking the 
    diagonal and cross-coupled terms of the inverse inertia matrix.
    """
    data = np.array(M_inv_history)
    time_axis = np.arange(len(data)) / sample_rate
    
    plt.figure(figsize=(10, 5))
    
    # Plot (M^-1)_22 : The diagonal term (Shoulder driving Shoulder)
    plt.plot(time_axis, data[:, 0], label=r'$(M^{-1})_{22}$ (Shoulder Diagonal)', 
             color='#1f77b4', linewidth=2)
    
    # Plot (M^-1)_32 : The cross-coupled term (Shoulder driving Elbow)
    plt.plot(time_axis, data[:, 1], label=r'$(M^{-1})_{32}$ (Shoulder-Elbow Cross-Coupling)', 
             color='#d62728', linewidth=2)
    
    plt.title('Evolution of Inverse Inertia Terms During Constant-Velocity Sweep')
    plt.xlabel('Time (s)')
    plt.ylabel(r'Inverse Inertia ($kg^{-1} m^{-2}$)')
    plt.legend(loc='best')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig("figure/fig_inertia_evolution.png", dpi=150, bbox_inches='tight')
    plt.show()
    
    print("\n[INERTIA ANALYSIS COMPLETE]")
    print(f"Average (M^-1)_22 Magnitude: {np.mean(data[:, 0]):.4f}")
    print(f"Average (M^-1)_32 Magnitude: {np.mean(data[:, 1]):.4f}")

def plot_timeseries(time_series, sample_rate):
        data = np.array(time_series)
        time_axis = np.arange(len(data)) / sample_rate
        
        plt.figure(figsize=(10, 5))
        
        plt.plot(time_axis, data[:,1], label=r'$(M^{-1})_{22}$ (Shoulder Diagonal)', 
                color='#1f77b4', linewidth=2)
        
        plt.plot(time_axis, data[:,2], label=r'$(M^{-1})_{22}$ (Shoulder Diagonal)', 
                color='#d62728', linewidth=2)
        
        plt.title('time series')
        plt.xlabel('Time (s)')
        
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig("figure/time_series.png", dpi=150, bbox_inches='tight')
        plt.show()
        
        