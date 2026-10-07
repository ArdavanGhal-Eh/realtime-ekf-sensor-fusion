use ekf_fusion::{ExtendedKalmanFilter, ImuMeasurement, Quaternion, Vector3, ZuptDetector};

#[test]
fn test_quaternion_identity_and_euler() {
    let q = Quaternion::identity();
    let norm = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    assert!((norm - 1.0).abs() < 1e-6);

    let (roll, pitch, yaw) = q.to_euler_angles();
    assert!(roll.abs() < 1e-6);
    assert!(pitch.abs() < 1e-6);
    assert!(yaw.abs() < 1e-6);
}

#[test]
fn test_ekf_static_gravity_alignment() {
    let mut ekf = ExtendedKalmanFilter::new();
    let dt = 0.01;

    // Static horizontal IMU: accel = [0, 0, 9.81], gyro = [0, 0, 0]
    for _ in 0..100 {
        let m = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0.0,
        };
        let (roll, pitch, _) = ekf.step(m, dt);
        assert!(roll.to_degrees().abs() < 5.0);
        assert!(pitch.to_degrees().abs() < 5.0);
    }
}

#[test]
fn test_ekf_yaw_rotation() {
    let mut ekf = ExtendedKalmanFilter::new();
    let dt = 0.01;
    let yaw_rate = 0.5; // rad/s
    let steps = 200; // 2 seconds

    let mut final_yaw = 0.0;
    for step in 0..steps {
        let m = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.0, 0.0, yaw_rate),
            timestamp: step as f64 * dt,
        };
        let (_, _, yaw) = ekf.step(m, dt);
        final_yaw = yaw;
    }

    // After 2.0s at 0.5 rad/s, expected yaw angle ~ 1.0 rad (~57.3 deg)
    let expected_yaw = yaw_rate * (steps as f64 * dt);
    assert!((final_yaw - expected_yaw).abs() < 0.15);
}

#[test]
fn test_zupt_detector_stationary_vs_dynamic() {
    let mut zupt = ZuptDetector::new(10, 0.05, 0.02);

    // Feed 10 stationary samples
    for _ in 0..10 {
        zupt.push_sample(9.81, 0.001);
    }
    assert!(zupt.is_stationary());

    // Inject violent movement
    zupt.push_sample(15.2, 1.4);
    assert!(!zupt.is_stationary());
}
