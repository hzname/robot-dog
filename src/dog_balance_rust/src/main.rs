use std::f64::consts::PI;
use std::io::{self, Write};
use std::time::Duration;

struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    prev_error: f64,
    integral_limit: f64,
}

impl PidController {
    fn new(kp: f64, ki: f64, kd: f64) -> Self {
        PidController {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            integral_limit: 0.5,
        }
    }
    
    fn update(&mut self, error: f64, dt: f64) -> f64 {
        self.integral += error * dt;
        self.integral = self.integral.min(self.integral_limit).max(-self.integral_limit);
        
        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;
        
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }
    
    fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

struct BalanceController {
    roll_pid: PidController,
    pitch_pid: PidController,
    target_roll: f64,
    target_pitch: f64,
    foot_offsets: [f64; 12],
}

impl BalanceController {
    fn new() -> Self {
        BalanceController {
            roll_pid: PidController::new(0.02, 0.001, 0.005),
            pitch_pid: PidController::new(0.02, 0.001, 0.005),
            target_roll: 0.0,
            target_pitch: 0.0,
            foot_offsets: [0.0; 12],
        }
    }
    
    fn update(&mut self, roll: f64, pitch: f64, dt: f64) -> [f64; 12] {
        let roll_error = self.target_roll - roll;
        let pitch_error = self.target_pitch - pitch;
        
        let roll_correction = self.roll_pid.update(roll_error, dt);
        let pitch_correction = self.pitch_pid.update(pitch_error, dt);
        
        // Calculate foot position adjustments
        // FL, FR, BL, BR - each leg has hip, thigh, calf
        let adjustments = [
            // Front Left
            -roll_correction * 0.5,   // hip
            pitch_correction * 0.5,   // thigh  
            pitch_correction * 0.3,   // calf
            // Front Right
            roll_correction * 0.5,    // hip
            pitch_correction * 0.5,   // thigh
            pitch_correction * 0.3,   // calf
            // Back Left
            -roll_correction * 0.5,   // hip
            -pitch_correction * 0.5,  // thigh
            -pitch_correction * 0.3,  // calf
            // Back Right
            roll_correction * 0.5,    // hip
            -pitch_correction * 0.5,  // thigh
            -pitch_correction * 0.3,  // calf
        ];
        
        for i in 0..12 {
            self.foot_offsets[i] = adjustments[i];
        }
        
        self.foot_offsets
    }
    
    fn set_target(&mut self, roll: f64, pitch: f64) {
        self.target_roll = roll;
        self.target_pitch = pitch;
    }
}

fn main() {
    println!("Balance Controller Rust started");
    println!("PID control for roll/pitch stabilization");
    
    let mut controller = BalanceController::new();
    let mut simulated_roll = 0.0;
    let mut simulated_pitch = 0.0;
    
    loop {
        let dt = 0.02; // 50 Hz
        
        // Simulate IMU data (would come from real IMU)
        simulated_roll = (simulated_roll + 0.01f64).sin() * 0.1;
        simulated_pitch = (simulated_pitch + 0.015f64).cos() * 0.08;
        
        let offsets = controller.update(simulated_roll, simulated_pitch, dt);
        
        print!("\rRoll: {:+.3} | Pitch: {:+.3} | Offsets: [", simulated_roll, simulated_pitch);
        for i in 0..4 {
            if i > 0 { print!(", "); }
            print!("{:+.3}", offsets[i]);
        }
        print!("...]");
        io::stdout().flush().unwrap();
        
        std::thread::sleep(Duration::from_millis(20));
    }
}
