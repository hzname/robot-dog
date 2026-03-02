use std::f64::consts::PI;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use rclrs::{Context, Node, Publisher, QosProfile, RclrsError, Subscription};
use sensor_msgs::msg::JointState;
use geometry_msgs::msg::Twist;

/// Gait controller node for quadruped robot
/// Publishes JointState messages to /joint_states at 50Hz

struct GaitController {
    joint_names: Vec<String>,
}

impl GaitController {
    fn new() -> Self {
        let legs = ["FL", "FR", "BL", "BR"];
        let mut joint_names = Vec::with_capacity(12);
        
        for leg in &legs {
            joint_names.push(format!("{}_hip_joint", leg));
            joint_names.push(format!("{}_thigh_joint", leg));
            joint_names.push(format!("{}_calf_joint", leg));
        }
        
        GaitController { joint_names }
    }
    
    /// Inverse kinematics: foot position → joint angles
    /// Simplified 2D IK for leg in sagittal plane
    fn inverse_kinematics(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
        let l1 = 0.08; // hip to thigh
        let l2 = 0.12; // thigh to calf
        let l3 = 0.12; // calf to foot
        
        // Hip abduction (y-axis movement)
        let hip = (y / l1).atan();
        
        // 2D IK in leg plane
        let d = (x * x + z * z).sqrt();
        let d2 = d - l1 * hip.cos();
        
        let cos_knee = (d2 * d2 + l2 * l2 - l3 * l3) / (2.0 * d2 * l2);
        let knee = PI - cos_knee.clamp(-1.0, 1.0).acos();
        
        let cos_thigh = (d2 * d2 + l2 * l2 - l3 * l3) / (2.0 * d2 * l2);
        let alpha = (z / d2).atan2(x / d2);
        let beta = (l3 * knee.sin() / d2).asin();
        let thigh = alpha + beta;
        
        (hip, thigh, -knee)
    }
    
    /// Get foot position for leg based on gait phase
    fn get_foot_position(leg: &str, phase: f64, velocity_x: f64) -> (f64, f64, f64) {
        let stride_x = velocity_x * 0.06; // stride length based on velocity
        let stride_z = 0.04; // lift height
        
        // Trot gait: diagonal legs move together
        let phase_offset = match leg {
            "FL" | "BR" => 0.0,
            "FR" | "BL" => PI,
            _ => 0.0,
        };
        
        let local_phase = (phase + phase_offset) % (2.0 * PI);
        
        if local_phase < PI {
            // Swing phase (leg in air)
            let t = local_phase / PI;
            let x = stride_x * (t - 0.5) * 2.0;
            let z = stride_z * (t * PI).sin();
            (x, 0.0, -0.18 + z) // z relative to hip
        } else {
            // Stance phase (leg on ground)
            let t = (local_phase - PI) / PI;
            let x = -stride_x * (t - 0.5) * 2.0;
            (x, 0.0, -0.18)
        }
    }
    
    /// Calculate joint states for all 12 joints
    fn calculate_joint_states(&self, phase: f64, velocity_x: f64) -> JointState {
        let legs = ["FL", "FR", "BL", "BR"];
        let mut positions = Vec::with_capacity(12);
        
        for leg in &legs {
            let (x, y, z) = Self::get_foot_position(leg, phase, velocity_x);
            let (hip, thigh, calf) = Self::inverse_kinematics(x, y, z);
            
            // Mirror right legs
            let hip = if *leg == "FR" || *leg == "BR" { -hip } else { hip };
            
            positions.push(hip);
            positions.push(thigh);
            positions.push(calf);
        }
        
        JointState {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time {
                    sec: 0,
                    nanosec: 0,
                }, // TODO: Use node.get_clock().now() when API available
                frame_id: "base_link".to_string(),
            },
            name: self.joint_names.clone(),
            position: positions,
            velocity: vec![0.0; 12],
            effort: vec![0.0; 12],
        }
    }
}

fn main() -> Result<(), RclrsError> {
    // Initialize ROS2 context
    let context = Context::new(std::env::args())?;
    
    // Create node
    let node = context.create_node("gait_controller_rust")?;
    
    // Shared velocity state between subscription and main loop
    let velocity_x = Arc::new(Mutex::new(0.1_f64)); // Default 0.1 m/s
    let velocity_x_clone = Arc::clone(&velocity_x);
    
    // Create publisher for joint states
    let publisher: Publisher<JointState> = node.create_publisher(
        "/joint_states",
        QosProfile::default(),
    )?;
    
    // Subscribe to cmd_vel for velocity commands
    let _cmd_vel_sub: Subscription<Twist> = node.create_subscription(
        "/cmd_vel",
        QosProfile::default(),
        move |msg: Twist| {
            let mut vel = velocity_x_clone.lock().unwrap();
            *vel = msg.linear.x as f64;
        },
    )?;
    
    println!("Gait Controller ROS2 (Rust) started");
    println!("Publishing to /joint_states at 50Hz");
    
    let controller = GaitController::new();
    let mut gait_phase = 0.0_f64;
    let frequency = 1.0_f64; // 1 Hz gait cycle
    
    // Main loop at 50Hz
    let period = Duration::from_millis(20);
    
    while context.ok() {
        let start = std::time::Instant::now();
        
        // Get current velocity
        let vel_x = *velocity_x.lock().unwrap();
        
        // Update gait phase
        let dt = 0.02_f64; // 20ms
        gait_phase += 2.0 * PI * frequency * dt;
        gait_phase %= 2.0 * PI;
        
        // Calculate joint states
        let joint_state = controller.calculate_joint_states(gait_phase, vel_x);
        
        // Publish
        publisher.publish(&joint_state)?;
        
        // Sleep to maintain 50Hz
        let elapsed = start.elapsed();
        if elapsed < period {
            std::thread::sleep(period - elapsed);
        }
    }
    
    println!("Shutting down gait controller");
    Ok(())
}
