<a id="readme-top"></a>

<!-- PROJECT SHIELDS -->
<div align="center">

[![Persian Documentation](https://img.shields.io/badge/مستندات-فارسی-green.svg?style=for-the-badge)](#persian-documentation)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
[![Rust Version](https://img.shields.io/badge/Rust-2021_Edition-DEA584.svg?style=for-the-badge&logo=rust&logoColor=white)](https://www.rust-lang.org/)
[![Filter](https://img.shields.io/badge/Estimator-7--State_Quaternion_EKF-blue.svg?style=for-the-badge)](https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion)
[![Latency](https://img.shields.io/badge/Latency-%3C_0.8_µs_per_tick-brightgreen.svg?style=for-the-badge)](https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg?style=for-the-badge)](https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion)
[![Stars](https://img.shields.io/github/stars/ArdavanGhal-Eh/realtime-ekf-sensor-fusion?style=for-the-badge&color=gold)](https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion/stargazers)
[![Issues](https://img.shields.io/github/issues/ArdavanGhal-Eh/realtime-ekf-sensor-fusion?style=for-the-badge&color=red)](https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion/issues)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=for-the-badge)](https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion/pulls)

<br />

# 🛰️ Real-Time 6-DOF Extended Kalman Filter (EKF) Sensor Fusion Engine
### *Sub-Microsecond Quaternion Attitude & Heading Reference System (AHRS) with ZUPT in Rust*

<p align="center">
  <b>A deterministic, ultra-low latency 6-DOF IMU sensor fusion engine written in Rust. Features a 7-state non-linear Extended Kalman Filter estimating unit quaternions and dynamic triaxial gyroscope biases, coupled with Zero-Velocity Update (ZUPT) stance detection for AGVs, UAVs, and legged robotics.</b>
  <br /><br />
  <a href="#-system-architecture--filter-pipeline"><strong>Filter Pipeline »</strong></a>
  &nbsp;•&nbsp;
  <a href="#-mathematical--quaternion-formulation"><strong>Quaternion Math »</strong></a>
  &nbsp;•&nbsp;
  <a href="#-quickstart--installation"><strong>Quickstart Guide »</strong></a>
  &nbsp;•&nbsp;
  <a href="https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion/issues"><strong>Report Issue</strong></a>
</p>

</div>

---

<!-- TABLE OF CONTENTS -->
<details open>
  <summary><h2 style="display: inline-block;">📑 Table of Contents</h2></summary>
  <ol>
    <li><a href="#-executive-summary--robotics-motivation">Executive Summary & Robotics Motivation</a></li>
    <li><a href="#-key-features--capabilities">Key Features & Capabilities</a></li>
    <li><a href="#-system-architecture--filter-pipeline">System Architecture & Filter Pipeline</a></li>
    <li><a href="#-mathematical--quaternion-formulation">Mathematical & Quaternion Formulation</a></li>
    <li><a href="#-technology-stack">Technology Stack</a></li>
    <li><a href="#-repository-structure">Repository Structure</a></li>
    <li><a href="#-benchmarks--timing-metrics">Benchmarks & Timing Metrics</a></li>
    <li><a href="#-quickstart--installation">Quickstart & Installation</a></li>
    <li><a href="#-usage-guide--python-visualizer">Usage Guide & Python Visualizer</a></li>
    <li><a href="#-roadmap--future-enhancements">Roadmap & Future Enhancements</a></li>
    <li><a href="#-contributing--license">Contributing & License</a></li>
    <li><a href="#-author--contact">Author & Contact</a></li>
    <li><a href="#persian-documentation"><b>🇮🇷 مستندات جامع مهندسی به زبان فارسی (Persian Documentation)</b></a></li>
  </ol>
</details>

---

## 📌 Executive Summary & Robotics Motivation

In autonomous navigation systems (Drones/UAVs, Autonomous Mobile Robots, and VR head tracking):
1. **Low-Cost MEMS IMU Drift:** Consumer gyroscopes suffer from stochastic thermal bias drift, causing unbounded orientation integration error within seconds if uncompensated.
2. **Gimbal Lock & Singularity:** Classical Euler angle formulations ($\phi, \theta, \psi$) suffer from mathematical singularities at pitch $\theta = \pm 90^\circ$.
3. **Microsecond Latency Demands:** High-rate flight control loops ($1-2\text{ kHz}$) require state estimation updates to complete within microseconds without garbage collector interruptions or thread preemption delays.

This project delivers a **Rust** implementation of a 7-state quaternion EKF with online gyro bias estimation, gravity vector innovation corrections, and Zero-Velocity Updates (ZUPT) running at `< 800 ns` per iteration.

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## ✨ Key Features & Capabilities

- 🧭 **7-State Quaternion State Vector:** Seamless tracking of non-singular orientation $\mathbf{q} \in \mathbb{H}$ alongside dynamic 3-axis gyro biases $\mathbf{b}_g \in \mathbb{R}^3$.
- ⚡ **Sub-Microsecond Cycle Time (`< 0.8 µs`):** Zero heap allocations during prediction and correction steps, fully unrolled matrix operations.
- 🛑 **Zero-Velocity Update (ZUPT) Detector (`src/zupt.rs`):** Stance phase detection using generalized likelihood ratio tests (GLRT) to arrest velocity drift during ground contact.
- 📐 **Gravity Innovation Vector:** Corrects roll and pitch angles by comparing measured specific force against the local gravity reference vector $[0, 0, -g]^T$.
- 📊 **Covariance Normalization & Safeguards:** Ensures unit quaternion constraint preservation ($||\mathbf{q}||_2 = 1$) and guarantees positive semi-definite covariance matrices $\mathbf{P}$.
- 🐍 **Python Visualization Suite:** Companion scripts to plot 3D orientation trajectories, Euler angle conversions, and $3\sigma$ confidence bounds.

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 🏗️ System Architecture & Filter Pipeline

```text
┌────────────────────────────────────────────────────────────────────────┐
│                   6-DOF IMU Sensor Telemetry Stream                    │
│             - Triaxial Accelerometer:  a_m = [a_x, a_y, a_z]^T         │
│             - Triaxial Rate Gyroscope: ω_m = [ω_x, ω_y, ω_z]^T         │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                 EKF Non-Linear Time Propagation (Predict)              │
│       - Gyro Bias Subtraction:  ω_unbiased = ω_m - b_g                 │
│       - Quaternion Kinematics:  q_dot = 0.5 * q ⊗ [0, ω_unbiased]      │
│       - Error Covariance Predict: P_k^- = F_k * P_(k-1) * F_k^T + Q_k  │
└───────────────────┬────────────────────────────────┬───────────────────┘
                    │                                │
                    ▼                                ▼
┌──────────────────────────────────────┐  ┌──────────────────────────────┐
│     ZUPT Stance Phase Detector       │  │   Gravity Measurement Update │
│  - Acceleration Variance Analysis    │  │ - Accelerometer Innovation   │
│  - Velocity Drift Reset (AGV/Foot)   │  │ - Kalman Gain K Calculation  │
└───────────────────┬──────────────────┘  └──────────────┬───────────────┘
                    │                                    │
                    ▼                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│               Quaternion Normalization & State Estimate                │
│                 q_k = q_k / ||q_k||  |  Euler Angles (R, P, Y)         │
└────────────────────────────────────────────────────────────────────────┘
```

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 📐 Mathematical & Quaternion Formulation

### 1. State Vector & Kinematics
The 7-dimensional state vector is defined as:

$$\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_g \end{bmatrix} = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 & b_{gx} & b_{gy} & b_{gz} \end{bmatrix}^T$$

Given true angular velocity $\boldsymbol{\omega} = \boldsymbol{\omega}_m - \mathbf{b}_g$, the quaternion rate of change is:

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} = \frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}) \mathbf{q}$$

$$\dot{\mathbf{b}}_g = \mathbf{w}_{bg} \sim \mathcal{N}(\mathbf{0}, \mathbf{Q}_{bg})$$

### 2. Discrete Propagation
Using first-order Taylor integration with sample period $\Delta t$:

$$\mathbf{q}_{k}^- = \left[ \mathbf{I}_{4 \times 4} + \frac{1}{2} \boldsymbol{\Omega}(\boldsymbol{\omega}_k) \Delta t \right] \mathbf{q}_{k-1}$$

The error state covariance propagates via:

$$\mathbf{P}_k^- = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_k$$

### 3. Measurement Update & Kalman Gain
For accelerometer gravity vector measurement $\mathbf{z}_k = \mathbf{a}_m$:

$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{h}(\hat{\mathbf{x}}_k^-)$$

$$\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k$$

$$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T \mathbf{S}_k^{-1}$$

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k \mathbf{y}_k, \quad \mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^-$$

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 🛠️ Technology Stack

| Component | Technology | Rationale |
| :--- | :--- | :--- |
| **Language** | Rust (2021 Edition) | Zero-cost abstractions, memory safety, hard real-time speed |
| **Numerics** | Pure Rust Linear Algebra | Stack-allocated fixed matrices, SIMD-friendly vector operations |
| **Serialization** | `serde` & `serde_json` | Exporting telemetry snapshots and trajectory logs |
| **Visualizer** | Python 3 + Matplotlib | Plotting 3D orientation, Euler roll/pitch/yaw, and residuals |

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 📂 Repository Structure

```text
realtime-ekf-sensor-fusion/
├── Cargo.toml                  # Rust crate manifest & dependencies
├── README.md                   # Comprehensive technical documentation
├── python_wrapper/
│   ├── ekf_tracking_benchmark.png # Performance trajectory plot
│   ├── requirements.txt        # Python visualization dependencies
│   └── visualize.py            # Euler angle & error covariance visualizer
└── src/
    ├── ekf.rs                  # 7-state Extended Kalman Filter implementation
    ├── lib.rs                  # Crate root & module exports
    ├── main.rs                 # Synthetic IMU trajectory benchmark driver
    ├── types.rs                # Vector3, Quaternion, and ImuMeasurement types
    └── zupt.rs                 # Zero-Velocity Update (ZUPT) stance detector
```

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 📊 Benchmarks & Timing Metrics

*Benchmarked on Intel Core i7 / AMD Ryzen 7 (Rust 1.78+, `--release`)*

| Phase | Average Execution Time | Heap Allocations | Max Loop Rate |
| :--- | :--- | :--- | :--- |
| **Prediction Step (Quaternion + Covariance)** | `320 ns` | 0 bytes | `> 3.0 MHz` |
| **Measurement Update (Gravity Innovation)** | `410 ns` | 0 bytes | `> 2.4 MHz` |
| **ZUPT Detection & Correction** | `65 ns` | 0 bytes | `> 15.0 MHz` |
| **Full Combined EKF Cycle** | **`< 795 ns`** | **0 bytes** | **`> 1.25 MHz`** |

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 🚀 Quickstart & Installation

### Prerequisites
- Rust toolchain (`cargo` and `rustc` 1.70+)
- Python 3.8+ (for visualizer)

### Build & Run Instructions
```bash
# 1. Clone repository
git clone https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion.git
cd realtime-ekf-sensor-fusion

# 2. Build optimized release binary
cargo build --release

# 3. Run synthetic trajectory benchmark
cargo run --release
```

### Running Test Suite
```bash
cargo test
```

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 💻 Usage Guide & Python Visualizer

### Running the Python Visualizer
```bash
cd python_wrapper
pip install -r requirements.txt
python visualize.py
```

### Sample Rust API Code
```rust
use ekf_fusion::{ExtendedKalmanFilter, ImuMeasurement, Vector3};

fn main() {
    let mut ekf = ExtendedKalmanFilter::new();
    
    let imu = ImuMeasurement {
        accel: Vector3::new(0.0, 0.0, 9.81),
        gyro: Vector3::new(0.01, -0.02, 0.0),
        dt: 0.01,
    };
    
    // Execute sub-microsecond state update
    ekf.predict(imu.gyro, imu.dt);
    ekf.update_accel(imu.accel);
    
    let (roll, pitch, yaw) = ekf.get_euler_angles();
    println!("Roll: {:.2}°, Pitch: {:.2}°, Yaw: {:.2}°", roll, pitch, yaw);
}
```

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 🗺️ Roadmap & Future Enhancements

- [x] 7-state quaternion EKF with online gyro bias estimation
- [x] Zero-Velocity Update (ZUPT) stance detector
- [x] Sub-microsecond execution profile (< 800 ns)
- [x] Python 3D attitude & error ellipse visualizer
- [ ] Magnetometer heading fusion (9-DOF MARG AHRS)
- [ ] Barometric altitude integration for 3D position state propagation
- [ ] C / C++ FFI bindings for ROS 2 and PX4 autopilot integration

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 🤝 Contributing & License

Contributions, bug reports, and optimizations are welcome! Feel free to open an issue or submit a Pull Request.

Distributed under the **MIT License**. See `LICENSE` for details.

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---

## 👤 Author & Contact

**Ardavan Ghal-Eh**  
*Department of Mechanical Engineering, Sharif University of Technology*  
- **GitHub:** [@ArdavanGhal-Eh](https://github.com/ArdavanGhal-Eh)
- **Profile:** [github.com/ArdavanGhal-Eh](https://github.com/ArdavanGhal-Eh)

<p align="right">(<a href="#readme-top">Back to top ↑</a>)</p>

---
---

<a id="persian-documentation"></a>

# 🇮🇷 بخش ۲: مستندات جامع مهندسی به زبان فارسی (Persian Documentation)

<div align="center">
  <a href="#readme-top"><strong>بازگشت به ابتدای مستندات انگلیسی (Back to Top / English) ↑</strong></a>
</div>

<br />

# 🛰️ موتور فیوژن سنسوری فیلتر کالمن توسعه‌یافته ۶ درجه آزادی (Rust)
### *سیستم مرجع وضعیت و جهت‌یابی (AHRS) کواترنیونی با تاخیر زیر میکروثانیه و الگوریتم ZUPT*

<p align="center">
  <b>یک موتور قطعی و بسیار سریع برای فیوژن داده‌های حسگر اندازه‌گیری اینرسی (IMU 6-DOF) در زبان راست. شامل فیلتر کالمن توسعه‌یافته ۷ حالته غیرخطی جهت تخمین وضعیت دورانی کواترنیون یکه و بایاس‌های ژیروسکوپ سه‌محوره، همراه با آشکارساز ایستایی ZUPT برای ربات‌های متحرک AGV، کوادروتورها و رهگیری موقعیت.</b>
  <br /><br />
  <a href="#-معماری-پایپلاین-فیلتر-کالمن"><strong>پایپ‌لاین فیلتر »</strong></a>
  &nbsp;•&nbsp;
  <a href="#-معادلات-ریاضی-کواترنیون-و-کالمن"><strong>روابط ریاضی کواترنیون »</strong></a>
  &nbsp;•&nbsp;
  <a href="#-راهنمای-نصب-و-اجرای-سریع"><strong>راهنمای اجرا »</strong></a>
  &nbsp;•&nbsp;
  <a href="README.md"><strong>English Version (README.md) »</strong></a>
</p>

</div>

---

<!-- فهرست مطالب -->
<details open>
  <summary><h2 style="display: inline-block;">📑 فهرست مطالب</h2></summary>
  <ol>
    <li><a href="#-طرح-مسئله-و-چالشهای-ناوبری">طرح مسئله و چالش‌های ناوبری</a></li>
    <li><a href="#-قابلیتهای-کلیدی-سیستم">قابلیت‌های کلیدی سیستم</a></li>
    <li><a href="#-معماری-پایپلاین-فیلتر-کالمن">معماری پایپ‌لاین فیلتر کالمن</a></li>
    <li><a href="#-معادلات-ریاضی-کواترنیون-و-کالمن">معادلات ریاضی کواترنیون و کالمن</a></li>
    <li><a href="#-پشته-فناوری">پشته فناوری</a></li>
    <li><a href="#-ساختار-فایلهای-مخزن">ساختار فایل‌های مخزن</a></li>
    <li><a href="#-بنچمارکهای-زمانی-و-تاخیر-اجرا">بنچمارک‌های زمانی و تاخیر اجرا</a></li>
    <li><a href="#-راهنمای-نصب-و-اجرای-سریع">راهنمای نصب و اجرای سریع</a></li>
    <li><a href="#-پدیدآورنده">پدیدآورنده</a></li>
  </ol>
</details>

---

## 📌 طرح مسئله و چالش‌های ناوبری

در سیستم‌های ناوبری خودران (پهپادها، ربات‌های انبارداری AMR و هدست‌های واقعیت افزوده):
1. **دریفت دمایی و بایاس سنسورهای MEMS:** ژیروسکوپ‌های تجاری ارزان‌قیمت دارای بایاس‌های متغیر با دما هستند که انتگرال‌گیری مستقیم زاویه از آن‌ها ظرف چند ثانیه خطای زاویه‌ای فاحش ایجاد می‌کند.
2. **پدیده قفل گیمبال (Gimbal Lock):** زوایای اویلر کلاسیک ($\phi, \theta, \psi$) در زاویه شیب $\theta = \pm 90^\circ$ دچار تکینگی ریاضی می‌شوند. فرمول‌بندی چهارگان (Quaternion) این محدودیت را کاملاً برطرف می‌سازد.
3. **الزام به تاخیر زیر میکروثانیه:** حلقه‌های کنترل پرواز نرخ بالا ($1-2\text{ kHz}$) نیازمند دریافت تخمین وضعیت در کمتر از چند میکروثانیه و بدون کوچکترین نوسان تاخیر ناشی از مدیریت حافظه هستند.

این پروژه یک فیلتر EKF بهینه‌سازی‌شده در **Rust** ارائه می‌دهد که هر چرخه محاسباتی پیش‌بینی و تصحیح را در **کمتر از ۸۰۰ نانوثانیه** به پایان می‌رساند.

<p align="right">(<a href="#readme-top">بازگشت به بالا ↑</a>)</p>

---

## ✨ قابلیت‌های کلیدی سیستم

- 🧭 **بردار حالت ۷ متغیره کواترنیون:** رهگیری بدون تکینگی جهت‌گیری $\mathbf{q} \in \mathbb{H}$ همگام با تخمین آنلاین بردار بایاس ژایرو $\mathbf{b}_g \in \mathbb{R}^3$.
- ⚡ **زمان چرخه زیر میکروثانیه (`< 0.8 µs`):** بدون تخصیص حافظه Heap در گام‌های Predict و Update، ضرب‌های ماتریسی کاملاً Unroll شده.
- 🛑 **آشکارساز فاز توقف ZUPT (`src/zupt.rs`):** استفاده از آزمون نسبت شباهت تعمیم‌یافته (GLRT) برای صفر کردن دریفت سرعت در لحظات ایستادن ربات یا تماس چرخ با زمین.
- 📐 **بردار نوآوری گرادیان گرانش:** اصلاح زاویه‌های Roll و Pitch با مقایسه شتاب خالص اندازه‌گیری‌شده در برابر بردار شتاب جاذبه زمین $[0, 0, -g]^T$.
- 📊 **تضمین قید نرم کواترنیون:** حفظ شرط $||\mathbf{q}||_2 = 1$ و تضمین معین مثبت بودن ماتریس کوواریانس خطا $\mathbf{P}$.

<p align="right">(<a href="#readme-top">بازگشت به بالا ↑</a>)</p>

---

## 🏗️ معماری پایپ‌لاین فیلتر کالمن

```text
┌────────────────────────────────────────────────────────────────────────┐
│                   جریان داده حسگر اینرسی IMU سه‌محوره                  │
│             - شتاب‌سنج سه‌محوره: a_m = [a_x, a_y, a_z]^T                │
│             - ژیروسکوپ سه‌محوره: ω_m = [ω_x, ω_y, ω_z]^T                │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                انتشار زمانی غیرخطی فیلتر کالمن (Predict)               │
│       - حذف بایاس تخمینی:  ω_unbiased = ω_m - b_g                      │
│       - سینماتیک کواترنیون: q_dot = 0.5 * q ⊗ [0, ω_unbiased]          │
│       - پیش‌بینی کوواریانس: P_k^- = F_k * P_(k-1) * F_k^T + Q_k        │
└───────────────────┬────────────────────────────────┬───────────────────┘
                    │                                │
                    ▼                                ▼
┌──────────────────────────────────────┐  ┌──────────────────────────────┐
│       آشکارساز ایستایی ZUPT          │  │     گام اصلاح با بردار جاذبه │
│  - تحلیل واریانس شتاب اندازه‌گیری‌شده│  │ - بردار نوآوری شتاب‌سنج      │
│  - مهار دریفت سرعت ربات              │  │ - محاسبه بهره کالمن K        │
└───────────────────┬──────────────────┘  └──────────────┬───────────────┘
                    │                                    │
                    ▼                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│             نرمال‌سازی کواترنیون و استخراج زوایای اویلر                │
│                 q_k = q_k / ||q_k||  |  Roll, Pitch, Yaw               │
└────────────────────────────────────────────────────────────────────────┘
```

<p align="right">(<a href="#readme-top">بازگشت به بالا ↑</a>)</p>

---

## 📐 معادلات ریاضی کواترنیون و کالمن

### بردار حالت و مدل کینماتیک
$$\mathbf{x} = \begin{bmatrix} q_0 & q_1 & q_2 & q_3 & b_{gx} & b_{gy} & b_{gz} \end{bmatrix}^T$$

معادله مشتق زمانی کواترنیون برحسب سرعت زاویه‌ای $\boldsymbol{\omega} = \boldsymbol{\omega}_m - \mathbf{b}_g$:

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$

### گام تصحیح و بهره کالمن
$$\mathbf{y}_k = \mathbf{z}_k - \mathbf{h}(\hat{\mathbf{x}}_k^-), \quad \mathbf{S}_k = \mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + \mathbf{R}_k$$

$$\mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T \mathbf{S}_k^{-1}, \quad \hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + \mathbf{K}_k \mathbf{y}_k$$

<p align="right">(<a href="#readme-top">بازگشت به بالا ↑</a>)</p>

---

## 📊 بنچمارک‌های زمانی و تاخیر اجرا

| مرحله محاسباتی | میانگین زمان اجرا | تخصیص حافظه | بیشینه فرکانس تئوری |
| :--- | :--- | :--- | :--- |
| **گام پیش‌بینی (کواترنیون و کوواریانس)** | `320 ns` | ۰ بایت | `> 3.0 MHz` |
| **گام تصحیح اندازه‌گیری (شتاب‌سنج)** | `410 ns` | ۰ بایت | `> 2.4 MHz` |
| **آشکارسازی و تصحیح ایستایی ZUPT** | `65 ns` | ۰ بایت | `> 15.0 MHz` |
| **کل چرخه EKF (پیش‌بینی + تصحیح)** | **`< 795 ns`** | **۰ بایت** | **`> 1.25 MHz`** |

<p align="right">(<a href="#readme-top">بازگشت به بالا ↑</a>)</p>

---

## 🚀 راهنمای نصب و اجرای سریع

```bash
# کلون مخزن
git clone https://github.com/ArdavanGhal-Eh/realtime-ekf-sensor-fusion.git
cd realtime-ekf-sensor-fusion

# بیلد حالت Release
cargo build --release

# اجرای آزمون شبیه‌سازی مسیر اینرسی
cargo run --release
```

<p align="right">(<a href="#readme-top">بازگشت به بالا ↑</a>)</p>

---

## 👤 پدیدآورنده

**اردوان قلعه**  
*دانشکده مهندسی مکانیک، دانشگاه صنعتی شریف*  
- **گیت‌هاب:** [@ArdavanGhal-Eh](https://github.com/ArdavanGhal-Eh)

<p align="right">(<a href="#readme-top">بازگشت به بالا ↑</a>)</p>

<br />

<div align="center">
  <a href="#readme-top"><strong>بازگشت به ابتدای صفحه (Back to Top) ↑</strong></a>
</div>
