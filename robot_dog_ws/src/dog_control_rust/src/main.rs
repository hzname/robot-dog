use std::f64::consts::PI;
use std::io::{self, Write};
use std::time::Duration;

// ROS2 types
mod ros2_types {
    use std::time::{SystemTime, UNIX_EPOCH};
    
    #[derive(Clone, Debug)]
    pub struct Time {
        pub sec: i32,
        pub nanosec: u32,
    }
    
    impl Time {
        pub fn now() -> Self {
            let duration = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or(Duration::from_secs(0));
            Time {
                sec: duration.as_secs() as i32,
                nanosec: duration.subsec_nanos(),
            }
        }
    }
    
    #[derive(Clone, Debug)]
    pub struct Header {
        pub stamp: Time,
        pub frame_id: String,
    }
    
    #[derive(Clone, Debug)]
    pub struct JointState {
        pub header: Header,
        pub name: Vec<String>,
        pub position: Vec<f64>,
        pub velocity: Vec<f64>,
        pub effort: Vec<f64>,
    }
    
    impl JointState {
        pub fn new() -> Self {
            JointState {
                header: Header {
                    stamp: Time::now(),
                    frame_id: "".to_string(),
                },
                name: Vec::new(),
                position: Vec::new(),
                velocity: Vec::new(),
                effort: Vec::new(),
            }
        }
    }
}

use ros2_types::*;

struct GaitController {
    joint_names: Vec<String>,
}

impl GaitController {
    fn new() -> Self {
        let joint_names = vec![
            "FL_hip_joint".to_string(), "FL_thigh_joint".to_string(), "FL_calf_joint".to_string(),
            "FR_hip_joint".to_string(), "FR_thigh_joint".to_string(), "FR_calf_joint".to_string(),
            "BL_hip_joint".to_string(), "BL_thigh_joint".to_string(), "BL_calf_joint".to_string(),
            "BR_hip_joint".to_string(), "BR_thigh_joint".to_string(), "BR_calf_joint".to_string(),
        ];
        
        GaitController { joint_names }
    }
    
    fn inverse_kinematics(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
        let l1 = 0.04;
        let l2 = 0.10;
        let l3 = 0.10;
        
        let hip = y.atan2(l1);
        let d = (x * x + z * z).sqrt().min(l2 + l3 - 0.01);
        
        let cos_calf = (l2 * l2 + l3 * l3 - d * d) / (2.0 * l2 * l3);
        let calf = PI - cos_calf.clamp(-1.0, 1.0).acos();
        
        let cos_thigh = (d * d + l2 * l2 - l3 * l3) / (2.0 * d * l2);
        let alpha = cos_thigh.clamp(-1.0, 1.0).acos();
        let beta = (-x).atan2(-z);
        let thigh = alpha + beta;
        
        (hip, thigh, calf)
    }
    
    fn get_foot_position(leg: &str, phase: f64, velocity_x: f64) -> (f64, f64, f64) {
        let stride_x = velocity_x * 0.06;
        let stride_z = 0.04;
        
        let phase_offset = match leg {
            "FL" | "BR" => 0.0,
            "FR" | "BL" => PI,
            _ => 0.0,
        };
        
        let local_phase = (phase + phase_offset) % (2.0 * PI);
        
        if local_phase < PI {
            let t = local_phase / PI;
            let x = stride_x * (t - 0.5) * 2.0;
            let z = stride_z * (t * PI).sin();
            (x, 0.0, -0.18 + z)
        } else {
            let t = (local_phase - PI) / PI;
            let x = -stride_x * (t - 0.5) * 2.0;
            (x, 0.0, -0.18)
        }
    }
    
    fn calculate_joint_states(&self, phase: f64, velocity_x: f64) -> JointState {
        let legs = ["FL", "FR", "BL", "BR"];
        let mut positions = Vec::with_capacity(12);
        
        for leg in &legs {
            let (x, y, z) = Self::get_foot_position(leg, phase, velocity_x);
            let (hip, thigh, calf) = Self::inverse_kinematics(x, y, z);
            let hip = if *leg == "FR" || *leg == "BR" { -hip } else { hip };
            
            positions.push(hip);
            positions.push(thigh);
            positions.push(calf);
        }
        
        JointState {
            header: Header {
                stamp: Time::now(),
                frame_id: "base_link".to_string(),
            },
            name: self.joint_names.clone(),
            position: positions,
            velocity: vec![0.0; 12],
            effort: vec![0.0; 12],
        }
    }
    
    fn format_joint_state(&self, js: &JointState) -> String {
        let mut result = format!("header:\n  stamp:\n    sec: {}\n    nanosec: {}\n  frame_id: \"{}\"\nname:",
            js.header.stamp.sec, js.header.stamp.nanosec, js.header.frame_id);
        
        for name in &js.name {
            result.push_str(&format!("\n  - \"{}\"", name));
        }
        
        result.push_str("\nposition:");
        for pos in &js.position {
            result.push_str(&format!("\n  - {:.6}", pos));
        }
        
        result.push_str("\nvelocity:");
        for _ in &js.velocity {
            result.push_str("\n  - 0.0");
        }
        
        result.push_str("\neffort:");
        for _ in &js.effort {
            result.push_str("\n  - 0.0");
        }
        
        result
    }
}

fn main() {
    println!("Gait Controller ROS2 started");
    println!("Publishing to /joint_states");
    println!("Format: YAML (for ros2 topic echo compatibility)");
    
    let controller = GaitController::new();
    let mut gait_phase = 0.0;
    let frequency = 1.0;
    
    loop {
        let dt = 0.02;
        gait_phase += 2.0 * PI * frequency * dt;
        gait_phase %= 2.0 * PI;
        
        let joint_state = controller.calculate_joint_states(gait_phase, 0.1);
        
        // Print as YAML format that can be piped to ros2 topic pub
        println!("---");
        println!("{}", controller.format_joint_state(&joint_state));
        
        std::thread::sleep(Duration::from_millis(20));
    }
}
