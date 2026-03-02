use std::io::{self, Write};
use std::time::Duration;

struct ServoDriver {
    positions: [f64; 12],
    use_mock: bool,
}

impl ServoDriver {
    fn new(use_mock: bool) -> Self {
        println!("Servo Driver Rust started (mock: {})", use_mock);
        ServoDriver {
            positions: [0.0; 12],
            use_mock,
        }
    }
    
    fn angle_to_pwm(angle: f64) -> u16 {
        // Convert radians to servo PWM (1000-2000 μs)
        let normalized = angle.clamp(-1.57, 1.57) / 1.57;
        (1500.0 + normalized * 500.0) as u16
    }
    
    fn update(&mut self) {
        // Simulate servo movement
        for i in 0..12 {
            self.positions[i] = (self.positions[i] * 0.9) + (0.0 * 0.1);
        }
        
        if !self.use_mock {
            // Here would be actual PCA9685 I2C communication
            // For now, just print
        }
        
        // Print PWM values
        print!("\rPWM: [");
        for i in 0..4 {
            if i > 0 { print!(", "); }
            print!("{}", Self::angle_to_pwm(self.positions[i]));
        }
        print!("...]");
        let _ = io::stdout().flush();
    }
    
    fn set_positions(&mut self, positions: [f64; 12]) {
        self.positions = positions;
    }
}

fn main() {
    let use_mock = std::env::var("USE_MOCK").unwrap_or_else(|_| "true".to_string()) == "true";
    let mut driver = ServoDriver::new(use_mock);
    
    println!("\nServo Driver running... (Ctrl+C to stop)");
    
    loop {
        driver.update();
        std::thread::sleep(Duration::from_millis(50));
    }
}
