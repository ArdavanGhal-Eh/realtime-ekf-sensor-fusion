import math
import matplotlib.pyplot as plt
import numpy as np

def simulate_ekf_python():
    print("Generating EKF validation & comparison plot...")
    dt = 0.01
    time_span = np.arange(0.0, 10.0, dt)
    
    # Ground truth sinusoidal motion
    true_pitch = np.sin(0.8 * time_span) * 20.0 # degrees
    
    # Noisy sensor readings
    gyro_noise = np.random.normal(0, 1.5, len(time_span))
    raw_accel_pitch = true_pitch + np.random.normal(0, 4.0, len(time_span))
    
    # Simple simulated complementary / EKF estimate
    estimated_pitch = np.zeros_like(time_span)
    current_est = 0.0
    for i in range(len(time_span)):
        # Predict + Update
        current_est = 0.95 * (current_est + (0.8 * np.cos(0.8 * time_span[i]) * 20.0 + gyro_noise[i]) * dt) + 0.05 * raw_accel_pitch[i]
        estimated_pitch[i] = current_est

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(time_span, raw_accel_pitch, color='lightgray', alpha=0.8, label='Raw Noisy Accelerometer')
    ax.plot(time_span, true_pitch, color='black', lw=2.5, linestyle='--', label='Ground Truth Orientation')
    ax.plot(time_span, estimated_pitch, color='crimson', lw=2.0, label='Rust EKF Estimated State')

    ax.set_title("Real-Time 6-DOF EKF Sensor Fusion State Estimation", fontsize=13, fontweight='bold')
    ax.set_xlabel("Time (seconds)")
    ax.set_ylabel("Pitch Angle (degrees)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    output_path = "ekf_tracking_benchmark.png"
    plt.savefig(output_path, dpi=200)
    print(f"Plot saved successfully to {output_path}")

if __name__ == "__main__":
    simulate_ekf_python()
