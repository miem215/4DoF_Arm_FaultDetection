# 4-DoF Robotic Arm: MIMO Diagnostics & Fault Isolation

A high-fidelity simulation of a 4 Degree-of-Freedom (4-DOF) interconnected robotic arm. This project serves as a testbed for advanced nonlinear control (NMPC), state estimation (UKF), and frequency-domain fault diagnostics in complex, multi-variable dynamic systems.

The primary research objective of this repository is to successfully isolate localized mechanical degradation (such as actuator wear) from cross-coupled structural vibrations across interconnected joints.

---

## System Architecture

The simulation bridges realistic sensor noise, state estimation, and whole-body nonlinear optimal control.

* **Physics Engine:** MuJoCo (Python bindings) executing forward dynamics at 500 Hz.
* **Optimal Control (NMPC):** Powered by CasADi. The controller embeds the full nonlinear rigid body dynamics ($M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau$) within a 20-step prediction horizon. This allows the arm to track trajectories while natively compensating for shifting gravity vectors and Coriolis forces that linear models cannot handle.
* **State Estimation (UKF):** An Unscented Kalman Filter runs at 50 Hz, fusing noisy joint position and velocity sensor data to generate clean, nonlinearly-derived state estimates ($\hat{q}, \hat{\dot{q}}$) for the control loop.

---

## Fault Injection & Diagnostic Pipeline

To replicate industrial constant-speed diagnostic tests, the arm is commanded to execute a continuous triangle-wave trajectory in joint space (sweeping Joint 2 at a constant velocity while holding adjacent joints stationary).

1. **Hardware Fault Simulation:** A localized mechanical anomaly (mimicking a degrading ballscrew or nut) is modeled as a high-frequency sinusoidal torque disturbance ($\tau_{fault} = A \sin(\omega t)$). This is injected directly into the **Joint 2 (Shoulder)** control loop at 6.0 Hz.
2. **Residual Generation:** The pipeline calculates a velocity residual vector $r(t) = y(t) - \hat{y}(t)$, comparing the raw, noisy sensor velocity to the UKF's expected healthy velocity. 
3. **Frequency Isolation:** Welch’s method is applied to the time-domain residuals to estimate the Power Spectral Density (PSD), transforming the noise into a clear frequency spectrum to flag anomalous harmonics.

---

## Analytical Findings: The Pitfalls of MIMO Fault Isolation

Detecting a fault is straightforward; isolating its root cause in an interconnected Multi-Input Multi-Output (MIMO) system presents complex mathematical and physical challenges. This testbed successfully demonstrated two classic diagnostic traps when analyzing open-chain kinematics.

![System Diagnostics Plot](figure/fig_analysis.png)

### Trap 1: Velocity Space & The Inertial Anchor
In initial tests, the diagnostic algorithm analyzed **velocity residuals**, comparing signal peaks against each joint's individual noise floor. The algorithm correctly detected the 6.0 Hz fault but falsely flagged **Joint 1 (Base)** as the root cause. 
* **The Physics:** Joint 1 has a massive effective inertia ($M_{11}$), acting as the system's anchor. It absorbs the cross-coupled reaction forces of the shaking shoulder with minimal physical displacement. 
* **The Algorithmic Failure:** Because Joint 1 was commanded to remain stationary, its baseline noise floor was practically zero. The tiny cross-coupled vibration easily triggered its relative peak threshold, while the true fault in Joint 2 was buried in the broadband noise of its macro-movement. 
* **The Fix:** The pipeline was updated to compare absolute spectral power across the entire system, correctly isolating Joint 2 in velocity space.

### Trap 2: Acceleration Space & Kinematic Amplification (The Whip Effect)
To achieve a cleaner dynamic response, the pipeline was shifted to analyze **acceleration residuals** ($\Delta \ddot{q} = \ddot{q}_{actual} - \ddot{q}_{commanded}$). While the frequency peaks became incredibly sharp, the absolute power comparison flagged **Joint 3 (Elbow)** as the root cause, registering exactly double the acceleration magnitude of the broken Joint 2.
* **The Physics:** Joint 3 sits at the end of the shaking 1.0-meter proximal link. Because the distal links (3 and 4) possess significantly lower rotational inertia than the heavy shoulder, they act as kinematic amplifiers. The vibration of the base whips the lightweight distal links, forcing them to undergo massive angular acceleration to maintain their posture.
* **The Mathematical Proof:** The acceleration error is defined by the inverse inertia matrix: $\Delta \ddot{q} = M^{-1}(q) \tau_{fault}$. In robotic arms with heavy bases and light tips, the off-diagonal cross-coupled terms (e.g., $(M^{-1})_{32}$) are often significantly larger than the diagonal driving terms ($(M^{-1})_{22}$), mathematically guaranteeing that the healthy distal joint will accelerate faster than the broken proximal joint.
* **The Conclusion:** Raw acceleration magnitude cannot be used to isolate faults in open-chain robotics. To truly isolate the root cause, acceleration residuals must be mapped back through the inertia matrix to generate **Torque Residuals** ($\tau_{res} = M(q)\Delta\ddot{q}$).

---
