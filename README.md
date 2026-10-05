# 🤖 Real-Time AMR Non-Linear Model Predictive Control (NMPC) & Obstacle Avoidance Suite (C++20 & Python)

A high-performance trajectory tracking and dynamic obstacle avoidance control framework engineered for **Autonomous Mobile Robots (AMRs)** and **Automated Guided Vehicles (AGVs)**. Implemented in **Modern C++20** with **Eigen3**, featuring Runge-Kutta 4th-order (RK4) non-linear kinematic propagation, barrier-function obstacle constraints, and sub-millisecond real-time execution (< 300 µs per solve cycle).

---

## 📌 Problem & Engineering Motivation
In automated smart warehouses (e.g., Amazon Robotics, KUKA, Tesla Gigafactories), mobile robots must track complex spline trajectories while dynamically evading unexpected obstacles, humans, and other moving AGVs.
Classical linear controllers (like PID or pure pursuit) fail because:
1. Differential drive kinematics are inherently non-linear ($\dot{x} = v \cos\theta, \dot{y} = v \sin\theta$).
2. They cannot anticipate upcoming curvature changes or enforce hard actuator velocity and acceleration limits.
3. Obstacle avoidance requires reactive swerving without destabilizing high-speed transport.

**Non-Linear Model Predictive Control (NMPC)** solves a receding-horizon constrained optimization problem at every control tick ($50-100\text{ Hz}$), guaranteeing optimal trajectory tracking, smooth acceleration, and collision-free navigation.

---

## 🌟 Architecture & Data Flow

```text
┌───────────────────────────────┐
│ Reference Trajectory Planner  │ (Lemniscate / Spline Waypoints)
└───────────────┬───────────────┘
                │ Horizon States x_ref[k], k=0..N
                ▼
┌───────────────────────────────┐
│     NMPC Optimal Controller   │ <─── Static & Dynamic Obstacle Coordinates
│     (Modern C++20 + Eigen3)   │ <─── Current Odometry State (x, y, theta)
└───────────────┬───────────────┘
                │ Solved Control Vector u*(t) = [v*, omega*]^T
                ▼
┌───────────────────────────────┐
│ Non-Linear Kinematics Engine  │ ───> Motor Wheel Speeds (Diff-Drive / AGV)
│ (4th-Order Runge-Kutta RK4)   │
└───────────────────────────────┘
```

---

## 📐 Mathematical Formulation

### 1. Robot Kinematics (Unicycle / Differential Drive)
$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} v \cos\theta \\ v \sin\theta \\ \omega \end{bmatrix}$$
Integrated numerically using 4th-Order Runge-Kutta (RK4) over discretization interval $\Delta t = 0.1\text{ s}$.

### 2. Optimal Control Problem (Receding Horizon)
$$\min_{\mathbf{u}_0, \dots, \mathbf{u}_{N-1}} \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{x}_{ref,k}\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k\|_{\mathbf{R}}^2 \right) + \|\mathbf{x}_N - \mathbf{x}_{ref,N}\|_{\mathbf{P}}^2 + \sum_{\text{obs}} \mathcal{B}(\mathbf{x}_k, \text{obs})$$

Subject to actuator saturation bounds:
$$0 \le v_k \le v_{max}, \quad |\omega_k| \le \omega_{max}$$

### 3. Collision Barrier Penalty
$$\mathcal{B}(\mathbf{x}, \text{obs}) = \begin{cases} w_{obs} \left( (R_{robot} + R_{obs} + d_{buffer}) - d_{actual} \right)^2 & \text{if } d_{actual} < R_{robot} + R_{obs} + d_{buffer} \\ 0 & \text{otherwise} \end{cases}$$

---

## 🎯 Real-World Applications & Cross-Industry Impact

### ⚙️ Robotics & Autonomous Systems
- **Intralogistics & Warehouse AGVs:** Safe high-speed navigation of shelving transport robots in tight industrial aisles.
- **Autonomous Delivery & Security Rovers:** Pedestrian avoidance and curbside trajectory tracking in GPS-denied environments.
- **Agricultural Robotics:** Precision field navigation along crop rows with active implement control.

### 🌐 Cross-Industry & Software Applications
- **Surgical Robotics:** Constrained needle and laparoscope trajectory planning avoiding critical anatomical nerves and vessels.
- **Autonomous Driving (ADAS):** Highway lane-change trajectory generation and collision-mitigation steering.

---

## 🚀 Installation & Build Instructions

### 1. Build & Run C++ NMPC Solver
```bash
mkdir build && cd build
cmake ..
cmake --build .
./amr_nmpc_solver
```

### 2. Run Python Performance & Trajectory Visualizer
```bash
cd python_visualizer
pip install -r requirements.txt
python plot_nmpc_trajectory.py
```

---

## 🛠️ Tech Stack
- **Core Optimization Engine:** Modern C++20, `Eigen3`
- **Numerical Integration:** Runge-Kutta 4th-Order (RK4)
- **Visualization & Analytics:** Python 3.10+, `numpy`, `matplotlib`
- **Build System:** `CMake` 3.16+

---

## 👨‍💻 Author
**Ardavan Ghal-Eh**  
Mechanical Engineering Student, Sharif University of Technology  
*Focus: Robotics, Autonomous Mobile Navigation & Model Predictive Control (MPC)*
