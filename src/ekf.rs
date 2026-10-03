use crate::types::{ImuMeasurement, Quaternion, Vector3};

/// Extended Kalman Filter state estimator for 6-DOF IMU.
/// State: [q0, q1, q2, q3, bg_x, bg_y, bg_z]
/// q: Orientation quaternion (4x1)
/// bg: Gyroscope bias drift (3x1)
pub struct ExtendedKalmanFilter {
    pub q: Quaternion,
    pub gyro_bias: Vector3,
    pub p: [[f64; 7]; 7], // 7x7 error covariance matrix
    pub q_noise: f64,     // Process noise covariance
    pub r_noise: f64,     // Measurement noise covariance
}

impl ExtendedKalmanFilter {
    pub fn new() -> Self {
        let mut p = [[0.0; 7]; 7];
        for i in 0..7 {
            p[i][i] = 0.01;
        }
        Self {
            q: Quaternion::identity(),
            gyro_bias: Vector3::new(0.0, 0.0, 0.0),
            p,
            q_noise: 0.001,
            r_noise: 0.05,
        }
    }

    /// Prediction step: propagates orientation quaternion using unbiased angular rates.
    pub fn predict(&mut self, gyro: Vector3, dt: f64) {
        // Remove estimated bias
        let wx = gyro.x - self.gyro_bias.x;
        let wy = gyro.y - self.gyro_bias.y;
        let wz = gyro.z - self.gyro_bias.z;

        // Quaternion kinematics: dq/dt = 0.5 * q * [0, w]
        let q = self.q;
        let dq_w = 0.5 * (-q.x * wx - q.y * wy - q.z * wz);
        let dq_x = 0.5 * ( q.w * wx + q.y * wz - q.z * wy);
        let dq_y = 0.5 * ( q.w * wy - q.x * wz + q.z * wx);
        let dq_z = 0.5 * ( q.w * wz + q.x * wy - q.y * wx);

        self.q.w += dq_w * dt;
        self.q.x += dq_x * dt;
        self.q.y += dq_y * dt;
        self.q.z += dq_z * dt;
        self.q = self.q.normalize();

        // Propagate state covariance: P = F * P * F^T + Q
        for i in 0..7 {
            self.p[i][i] += self.q_noise * dt;
        }
    }

    /// Measurement update step using gravitational acceleration vector.
    pub fn update_accel(&mut self, accel: Vector3) {
        let a = accel.normalize();
        let q = self.q;

        // Estimated gravity direction in body frame: g_b = R(q)^T * [0, 0, 1]
        let g_x = 2.0 * (q.x * q.z - q.w * q.y);
        let g_y = 2.0 * (q.w * q.x + q.y * q.z);
        let g_z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

        // Innovation error residual
        let err_x = a.x - g_x;
        let err_y = a.y - g_y;
        let err_z = a.z - g_z;

        // Adaptive Kalman gain calculation (scalar approximation for real-time safety)
        let s = self.p[0][0] + self.r_noise;
        let k = (self.p[0][0] / s).clamp(0.01, 0.5);

        // State correction
        self.q.w += 0.5 * k * (-q.x * err_x - q.y * err_y - q.z * err_z);
        self.q.x += 0.5 * k * ( q.w * err_x + q.y * err_z - q.z * err_y);
        self.q.y += 0.5 * k * ( q.w * err_y - q.x * err_z + q.z * err_x);
        self.q.z += 0.5 * k * ( q.w * err_z + q.x * err_y - q.y * err_x);
        self.q = self.q.normalize();

        // Gyro bias adaptation
        self.gyro_bias.x += 0.1 * k * err_x;
        self.gyro_bias.y += 0.1 * k * err_y;
        self.gyro_bias.z += 0.1 * k * err_z;

        // Covariance update: P = (I - K*H) * P
        for i in 0..7 {
            self.p[i][i] *= (1.0 - 0.5 * k);
        }
    }

    /// Executes full filter step on IMU frame.
    pub fn step(&mut self, measurement: ImuMeasurement, dt: f64) -> (f64, f64, f64) {
        self.predict(measurement.gyro, dt);
        self.update_accel(measurement.accel);
        self.q.to_euler_angles()
    }
}
