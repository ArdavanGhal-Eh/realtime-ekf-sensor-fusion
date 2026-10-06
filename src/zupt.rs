// Zero-Velocity Update (ZUPT) & Stance Phase Detector
// Part of 6-DOF Extended Kalman Filter Suite in Rust
// Author: Ardavan Ghal-Eh | Sharif University of Technology

pub struct ZuptDetector {
    pub accel_window: Vec<f64>,
    pub gyro_window: Vec<f64>,
    pub window_size: usize,
    pub accel_threshold: f64, // (m/s^2)^2 variance threshold
    pub gyro_threshold: f64,  // (rad/s)^2 threshold
}

impl ZuptDetector {
    pub fn new(window_size: usize, accel_threshold: f64, gyro_threshold: f64) -> Self {
        Self {
            accel_window: Vec::with_capacity(window_size),
            gyro_window: Vec::with_capacity(window_size),
            window_size,
            accel_threshold,
            gyro_threshold,
        }
    }

    pub fn push_sample(&mut self, accel_mag: f64, gyro_mag: f64) {
        if self.accel_window.len() >= self.window_size {
            self.accel_window.remove(0);
            self.gyro_window.remove(0);
        }
        self.accel_window.push(accel_mag);
        self.gyro_window.push(gyro_mag);
    }

    // Returns true if vehicle/robot is in stationary stance phase
    pub fn is_stationary(&self) -> bool {
        if self.accel_window.len() < self.window_size {
            return false;
        }

        // Calculate accelerometer variance
        let n = self.accel_window.len() as f64;
        let mean_a = self.accel_window.iter().sum::<f64>() / n;
        let var_a = self.accel_window.iter().map(|&a| (a - mean_a).powi(2)).sum::<f64>() / n;

        // Calculate gyro energy
        let mean_g = self.gyro_window.iter().sum::<f64>() / n;

        var_a < self.accel_threshold && mean_g < self.gyro_threshold
    }
}
