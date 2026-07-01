# 4-DoF Robotic Arm: MIMO Diagnostics & Fault Isolation

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0%2B-lightgrey)
![CasADi](https://img.shields.io/badge/CasADi-Optimal_Control-orange)

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

## Case Study: Cross-Coupled False Positives

Detecting a fault is straightforward; isolating it in an interconnected MIMO system is mathematically complex. 

During initial diagnostic development, a naive relative-thresholding algorithm successfully detected the 6.0 Hz anomaly. However, **it falsely flagged Joint 1 (Base)** as the source of the mechanical failure, despite the fault being physically injected into Joint 2.

![System Diagnostics Plot](figure/fig_analysis.png)

This false positive perfectly illustrates the challenges of Multi-Input Multi-Output (MIMO) system diagnostics:

* **Physical Cross-Coupling:** Because the arm is a rigidly coupled multibody system, the violent 6.0 Hz torque ripple in the shoulder physically translates through the inertia matrix $M(q)$. Joint 1, acting as the anchored base, absorbs the reaction forces, causing the 6.0 Hz vibration to physically propagate into Joint 1's sensor data.
* **Algorithmic Vulnerability:** The initial pipeline evaluated fault peaks relative to each joint's *individual* noise floor. Because Joint 2 was executing a massive kinematic sweep, its baseline broadband noise was high, visually masking the absolute power of the 6.0 Hz peak. Conversely, because Joint 1 was commanded to remain perfectly stationary, its baseline noise floor was zero. The cross-coupled vibration easily triggered Joint 1's relative threshold, leading to a misclassification of the root cause.

**The Solution:** The pipeline was rewritten to abandon isolated relative thresholds. It now performs a system-wide absolute power comparison at the targeted harmonic. By measuring where the structural vibration is mathematically most powerful, the algorithm correctly ignores the cross-coupled noise and isolates Joint 2 as the true root cause.

---
