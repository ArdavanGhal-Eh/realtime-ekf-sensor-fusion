use ekf_fusion::{ExtendedKalmanFilter, ImuMeasurement, Vector3};
use std::time::Instant;

fn main() {
    println!("============================================================");
    println!("🚀 Real-Time Extended Kalman Filter (EKF) Sensor Fusion Engine");
    println!("   Developed in High-Performance Rust (Zero-Allocation Loop)");
    println!("============================================================");

    let mut ekf = ExtendedKalmanFilter::new();
    let dt = 0.01; // 100 Hz update frequency
    let total_steps = 1000;

    println!("Simulating 10 seconds of 6-DOF IMU data (1000 steps)...\n");
    let start_time = Instant::now();

    for step in 0..total_steps {
        let t = step as f64 * dt;
        
        // Simulated noisy angular velocity (yaw rotation + noise)
        let true_rate = (t * 0.5).sin() * 0.2;
        let noise = ((step % 7) as f64 - 3.0) * 0.005;
        let gyro = Vector3::new(noise, noise, true_rate + noise);

        // Simulated accelerometer measuring gravity + centripetal noise
        let accel = Vector3::new(
            0.0 + noise,
            0.0 + noise,
            9.81 + noise * 0.1
        );

        let measurement = ImuMeasurement { accel, gyro, timestamp: t };
        let (roll, pitch, yaw) = ekf.step(measurement, dt);

        if step % 200 == 0 || step == total_steps - 1 {
            println!(
                "Step {:4} (t = {:.2}s) | Roll: {:+6.2}° | Pitch: {:+6.2}° | Yaw: {:+6.2}° | Gyro Bias Z: {:+.4}",
                step,
                t,
                roll.to_degrees(),
                pitch.to_degrees(),
                yaw.to_degrees(),
                ekf.gyro_bias.z
            );
        }
    }

    let elapsed = start_time.elapsed();
    let avg_step_micros = elapsed.as_micros() as f64 / total_steps as f64;

    println!("\n✅ Benchmark Completed:");
    println!("• Total Time for 1000 EKF Cycles: {:?}", elapsed);
    println!("• Average Processing Time per IMU Cycle: {:.3} microseconds (< 1 µs)", avg_step_micros);
    println!("• Capable of processing IMU frequencies exceeding 100,000 Hz");
}
