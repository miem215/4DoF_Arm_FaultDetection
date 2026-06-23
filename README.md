## Project Overview

This project simulates a 4 Degree-of-Freedom (4-DOF) robotic arm in the MuJoCo physics engine. It utilizes a custom-built **Nonlinear Model Predictive Controller (NMPC)** powered by CasADi to calculate optimal trajectories, coupled with an **Unscented Kalman Filter (UKF)** to handle noisy real-world sensor data.

The system is designed to track a target coordinate while proactively dodging a dynamic, swinging obstacle, demonstrating advanced optimal control and state estimation in a simulated hardware environment.

## File Structure

* `main.py` - The core simulation loop. Initializes MuJoCo, injects Gaussian noise into the sensor bus, and ties the UKF and NMPC pipelines together.

* `controller.py` - Contains the CasADi optimization logic. Defines the explicit dynamics, cost functions, and non-linear collision constraints over a finite prediction horizon.

* `filter.py` - Contains the Unscented Kalman Filter (UKF) implementation. Uses deterministic sigma points to filter noisy joint positions and velocities.

* `Kinematic.py` - The kinematic engine handling symbolic forward kinematics for the CasADi solver.

* `3DoFarm.xml` - The MuJoCo environment specification, defining the physical attributes of the arm, the dynamic obstacle, and the target.

## Installation & Usage

**Prerequisites:**
You will need Python 3.8+ and the following libraries:

```bash
pip install mujoco casadi numpy scipy stable-baselines3 gymnasium
```

**Running the Simulation:**
To launch the simulation with the passive MuJoCo viewer:

```bash
python main.py
```

## Mathematical Formulation

### 1. System Dynamics

The NMPC predicts the future states of the arm over a horizon $N$ using Explicit Euler integration. Let the state vector be $x = [q, \dot{q}]^T \\in \\mathbb{R}^8$ and the control input be joint accelerations $u = \ddot{q} \\in \\mathbb{R}^4$. The system dynamics are defined as:

$$
x_{k+1} = \\begin{bmatrix} q_{k+1} \\\\ \dot{q}_{k+1} \\end{bmatrix} = \\begin{bmatrix} q_k + \dot{q}_k \Delta t \\\\ \dot{q}_k + u_k \Delta t \\end{bmatrix}
$$

### 2. NMPC Cost Function

The CasADi solver minimizes a highly tuned cost function $J$ across the prediction horizon. The cost function balances aggressive target tracking with energy efficiency and postural stability:

$$
J = \sum_{k=0}^{N-1} \left( J_{track, k} + J_{effort, k} + J_{posture, k} + J_{slack, k} \right) + J_{terminal}
$$

Where the individual running costs are defined as:

* **Target Tracking:** $J_{track, k} = 500 \\| \\text{FK}(q_k) - p_{target} \\|_2^2$

* **Control & Velocity Effort:** $J_{effort, k} = 0.2 \\| u_k \\|_2^2 + 0.2 \\| \dot{q}_k \\|_2^2$

* **Postural Alignment:** $J_{posture, k} = (q_k - q_{home})^T W_{posture} (q_k - q_{home})$

* **Obstacle Slack Penalty:** $J_{slack, k} = W_{obs} \cdot s_k$ (where $W_{obs} = 100,000$)

### 3. Whole-Body Collision Avoidance (Virtual Nodes)

To prevent the intermediate links from clipping through the dynamic obstacle, the arm calculates fast 2D planar kinematics (treating the obstacle as an infinite pillar along the Z-axis). For each joint/node, the radial distance $r$ in the X-Y plane is derived:

$$
r_{elbow} = L_2 \sin(q_2)
$$

$$
r_{wrist} = r_{elbow} + L_3 \sin(q_2 + q_3)
$$

A soft constraint is applied to the End-Effector, Wrist, Elbow, and interpolated Link Midpoints. A slack variable $s_k \geq 0$ allows the solver to find mathematically feasible routes if trapped:

$$
(x_{node} - x_{obs})^2 + (y_{node} - y_{obs})^2 + s_k \geq r_{safe}^2
$$

### 4. Unscented Kalman Filter (UKF)

To simulate hardware reality, Gaussian noise is injected into the MuJoCo sensor bus. The measurement vector $z_t \\in \\mathbb{R}^8$ is formulated as:

$$
z_t = x_{true, t} + \mathcal{N}(0, R)
$$

The UKF utilizes deterministic sigma points to predict the non-linear state propagation, filtering the noisy $z_t$ into clean state estimates $\hat{x}_t = [\hat{q}, \hat{\dot{q}}]^T$ before they are passed into the NMPC solver.

## Current State & Features

* **Advanced State Estimation:** Filters Gaussian noise from joint sensors before feeding states into the controller.

* **Dynamic Postural Costs (NMPC):** State-dependent cost weights. Distal joints stiffen when reaching from afar to act like a spear, and loosen dynamically as the end-effector enters the target zone.

* **Dynamic Obstacle Avoidance:** Environment features a moving dynamic obstacle. The NMPC recalculates on the fly to dodge it.

* **Target Tracking & Hold:** The arm aggressively pursues the target coordinate and switches to a stable hold/hover state upon breaching the tolerance threshold.

## Known Issues

**Whole-Body vs. Tip Collision:** Currently, the end-effector dodges the dynamic obstacle perfectly using a 2D planar force field constraint. However, the system struggles with strict *Whole-Body Collision Avoidance*. While intermediate virtual nodes have been drafted, the intermediate links can still occasionally clip the obstacle. The discrete virtual node approach requires further tuning to create a truly impenetrable force field along the 1-meter link lengths.

## Future Roadmap

1. **Continuous Collision Avoidance:** Replace the discrete "Virtual Node" point-mass constraints with true **Line-Segment to Point** distance formulas.

2. **Computer Vision Integration:** Replace the raw MuJoCo `mocap_pos` data with a simulated RGB camera pipeline to estimate the obstacle's state dynamically.

3. **Dynamic Target Interception:** Feed an estimated target velocity vector into the CasADi prediction horizon to intercept moving targets.

4. **Reinforcement Learning Benchmarking:** Wrap the environment in a Gymnasium interface to benchmark this NMPC's performance against PPO/SAC deep learning agents.
"""
