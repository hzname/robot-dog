use std::f64::consts::PI;
use std::io::{self, Write};
use std::time::Duration;

struct ImuData {
    ax: f64,
    ay: f64,
    az: f64,
    gx: f64,
    gy: f64,
    gz: f64,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

struct ImuNode {
    use_mock: bool,
    roll: f64,
    pitch: f64,
    yaw: f64,
}

impl ImuNode {
    fn new(use_mock: bool) -> Self {
        println!("IMU Node Rust started (mock: {})", use_mock);
        ImuNode {
            use_mock,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
    
    fn read_mpu6050(&self) -> (f64, f64, f64, f64, f64, f64) {
        if self.use_mock {
            // Simulate movement
            let t = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or(std::time::Duration::from_secs(0))
                .as_secs_f64();
            (
                0.0,
                0.0,
                9.81,
                (t * 2.0).sin() * 0.1,
                (t * 1.5).cos() * 0.1,
                0.0,
            )
        } else {
            // Here would be actual MPU6050 I2C reading
            (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
        }
    }
    
    fn complementary_filter(&mut self, ax: f64, ay: f64, az: f64, gx: f64, gy: f64, gz: f64, dt: f64) {
        let alpha = 0.02;
        
        // Accelerometer angles
        let accel_roll = ay.atan2(az);
        let accel_pitch = ax.atan2(az);
        
        // Complementary filter
        self.roll = alpha * (self.roll + gx * dt) + (1.0 - alpha) * accel_roll;
        self.pitch = alpha * (self.pitch + gy * dt) + (1.0 - alpha) * accel_pitch;
        self.yaw += gz * dt;
    }
    
    fn update(&mut self, dt: f64) -> ImuData {
        let (ax, ay, az, gx, gy, gz) = self.read_mpu6050();
        self.complementary_filter(ax, ay, az, gx, gy, gz, dt);
        
        ImuData {
            ax, ay, az, gx, gy, gz,
            roll: self.roll,
            pitch: self.pitch,
            yaw: self.yaw,
        }
    }
}

fn main() {
    let use_mock = std::env::var("USE_MOCK").unwrap_or_else(|_| "true".to_string()) == "true";
    let mut imu = ImuNode::new(use_mock);
    
    println!("\nIMU running... (Ctrl+C to stop)");
    
    let mut last_time = std::time::Instant::now();
    
    loop {
        let now = std::time::Instant::now();
        let dt = now.duration_since(last_time).as_secs_f64();
        last_time = now;
        
        let data = imu.update(dt);
        
        print!("\rRoll: {:+.3} | Pitch: {:+.3} | Yaw: {:+.3}", 
               data.roll, data.pitch, data.yaw);
        let _ = io::stdout().flush();
        
        std::thread::sleep(Duration::from_millis(10));
    }
}
