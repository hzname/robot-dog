use crossterm::event::{self, Event, KeyCode, KeyEvent};
use std::io::{self, Write};
use std::time::Duration;

#[derive(Clone, Debug)]
pub struct Twist {
    pub linear_x: f64,
    pub linear_y: f64,
    pub angular_z: f64,
}

impl Twist {
    fn new() -> Self {
        Twist {
            linear_x: 0.0,
            linear_y: 0.0,
            angular_z: 0.0,
        }
    }
    
    fn stop(&mut self) {
        self.linear_x = 0.0;
        self.linear_y = 0.0;
        self.angular_z = 0.0;
    }
}

pub struct TeleopNode {
    cmd_vel: Twist,
    speed: f64,
    turn_speed: f64,
    gait_enabled: bool,
    current_pose: String,
}

impl TeleopNode {
    pub fn new() -> Self {
        println!("Teleop Rust started");
        println!("Controls:");
        println!("  w/s - Forward/Backward");
        println!("  a/d - Left/Right");
        println!("  q/e - Rotate Left/Right");
        println!("  g - Toggle gait");
        println!("  1/2/3 - Stand/Sit/Lie poses");
        println!("  +/- - Speed up/down");
        println!("  Space - Emergency stop");
        println!("  Ctrl+C - Quit");
        println!();
        
        TeleopNode {
            cmd_vel: Twist::new(),
            speed: 0.1,
            turn_speed: 0.5,
            gait_enabled: false,
            current_pose: "stand".to_string(),
        }
    }
    
    pub fn process_key(&mut self, key: KeyEvent) {
        match key.code {
            KeyCode::Char('w') | KeyCode::Char('W') => {
                self.cmd_vel.linear_x = self.speed;
            }
            KeyCode::Char('s') | KeyCode::Char('S') => {
                self.cmd_vel.linear_x = -self.speed;
            }
            KeyCode::Char('a') | KeyCode::Char('A') => {
                self.cmd_vel.linear_y = self.speed;
            }
            KeyCode::Char('d') | KeyCode::Char('D') => {
                self.cmd_vel.linear_y = -self.speed;
            }
            KeyCode::Char('q') | KeyCode::Char('Q') => {
                self.cmd_vel.angular_z = self.turn_speed;
            }
            KeyCode::Char('e') | KeyCode::Char('E') => {
                self.cmd_vel.angular_z = -self.turn_speed;
            }
            KeyCode::Char(' ') => {
                self.cmd_vel.stop();
                println!("\n🛑 Emergency STOP!");
            }
            KeyCode::Char('g') | KeyCode::Char('G') => {
                self.gait_enabled = !self.gait_enabled;
                println!("\n🐕 Gait: {}", if self.gait_enabled { "ON" } else { "OFF" });
            }
            KeyCode::Char('1') => {
                self.current_pose = "stand".to_string();
                println!("\n🦮 Pose: STAND");
            }
            KeyCode::Char('2') => {
                self.current_pose = "sit".to_string();
                println!("\n🪑 Pose: SIT");
            }
            KeyCode::Char('3') => {
                self.current_pose = "lie".to_string();
                println!("\n🛌 Pose: LIE");
            }
            KeyCode::Char('+') | KeyCode::Char('=') => {
                self.speed = (self.speed + 0.05).min(0.5);
                println!("\n⚡ Speed: {:.2}", self.speed);
            }
            KeyCode::Char('-') => {
                self.speed = (self.speed - 0.05).max(0.05);
                println!("\n⚡ Speed: {:.2}", self.speed);
            }
            _ => {}
        }
    }
    
    pub fn publish_cmd_vel(&self) {
        print!("\rCmdVel: x={:+.2} y={:+.2} rot={:+.2} | Gait: {} | Pose: {} | Speed: {:.2}  ",
               self.cmd_vel.linear_x,
               self.cmd_vel.linear_y,
               self.cmd_vel.angular_z,
               if self.gait_enabled { "ON " } else { "OFF" },
               self.current_pose,
               self.speed);
        io::stdout().flush().unwrap();
    }
    
    pub fn get_cmd_vel(&self) -> Twist {
        self.cmd_vel.clone()
    }
}

fn main() -> io::Result<()> {
    let mut teleop = TeleopNode::new();
    
    loop {
        // Check for keypress (non-blocking)
        if event::poll(Duration::from_millis(50))? {
            if let Event::Key(key_event) = event::read()? {
                match key_event.code {
                    KeyCode::Char('c') if key_event.modifiers.contains(event::KeyModifiers::CONTROL) => {
                        println!("\n👋 Goodbye!");
                        return Ok(());
                    }
                    _ => teleop.process_key(key_event),
                }
            }
        }
        
        teleop.publish_cmd_vel();
    }
}
