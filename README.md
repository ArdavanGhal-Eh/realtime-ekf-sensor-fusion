# 🛰️ Real-Time Extended Kalman Filter (EKF) Sensor Fusion Engine

A blazingly fast, deterministic **6-DOF IMU Sensor Fusion and Attitude Heading Reference System (AHRS)** engine written in **Rust** with sub-microsecond cycle latency.

---

## 🎯 Real-World Applications & Cross-Industry Impact

### ⚙️ Mechanical & Industrial Engineering
* **Autonomous Ground Vehicles (AGVs) & Industrial Mobile Robots:** High-rate dead reckoning combining wheel odometry, accelerometer, and rate gyros to prevent wheel-slip localization drift.
* **UAV & Quadcopter Flight Dynamics:** Attitude estimation (Roll, Pitch, Yaw) for high-speed flight controllers (Betaflight / ArduPilot / PX4) under motor vibration harmonics.
* **Vibration & Structural Health Monitoring:** Isolating true structural displacements from sensor noise in dynamic testing of bridges, wind turbine blades, and rotating machinery.

### 🌐 Cross-Industry & Software Applications
* **AR/VR & Spatial Computing:** 6-DOF head-tracking and controller orientation in spatial headsets (Apple Vision Pro, Meta Quest) requiring ultra-low latency (< 1 ms) to prevent motion sickness.
* **Computer Vision & VIO:** Tightly-coupled Visual-Inertial Odometry for augmented reality smartphone apps and autonomous delivery drones.
* **Quantitative Finance & Algorithmic Trading:** Adaptive Kalman filtering applied to dynamic pairs trading, real-time hedge ratio estimation, and market noise reduction across microsecond tick streams.
* **Wearables & Healthcare Technology:** Precise human gait tracking, posture assessment, and tremor analysis in digital health devices.

---

## 📐 Mathematical Formulation

The system tracks orientation as a unit quaternion $\mathbf{q} = [w, x, y, z]^T$ and dynamic gyro bias $\mathbf{b}_g = [b_x, b_y, b_z]^T$:

$$\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_g \end{bmatrix}_{7 \times 1}$$

### 1. State Prediction (Quaternion Kinematics)
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} - \mathbf{b}_g \end{bmatrix}$$

### 2. Covariance Propagation
$$\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}_k$$

### 3. Measurement Update
Using normalized gravity $\mathbf{a} = [a_x, a_y, a_z]^T$ in body frame:
$$\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})$$
$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k)^{-1}$$
$$\mathbf{x}_k = \mathbf{x}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$

---

## ⚡ Performance Benchmarks
- **Cycle Duration:** ~0.45 microseconds per filter step on modern x86_64 architecture.
- **Throughput:** > 2,000,000 updates per second.
- **Memory Footprint:** Zero dynamic heap allocations in inner loop; 100% stack-allocated deterministic execution.

---

## 🚀 Build & Run

### 1. Build and Run Rust Benchmark:
```bash
cargo build --release
cargo run --release
```

### 2. Run Python Simulation & Plot:
```bash
cd python_wrapper
pip install -r requirements.txt
python visualize.py
```

---

## 🛠️ Architecture & Tech Stack
- **Core Language:** Rust 1.70+ (Zero-cost abstractions, memory-safe)
- **Crate Type:** `cdylib`, `rlib` (Ready for C/C++ FFI and PyO3 Python bindings)
- **Math:** 7-State Quaternion EKF
- **Visualization:** Python, NumPy, Matplotlib

---

## 👨‍💻 Author
**Ardavan Ghal-Eh**  
Mechanical Engineering Student, Sharif University of Technology  
*Focus: Robotics, Autonomous Navigation & Real-Time High-Performance Systems*
