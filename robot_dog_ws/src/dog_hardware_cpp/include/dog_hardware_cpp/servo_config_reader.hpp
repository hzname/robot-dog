#pragma once

#include <string>
#include <vector>
#include <optional>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>

namespace dog_hardware_cpp {

struct JointConfig {
    std::string name;
    std::string leg;       // "LF", "RF", "LR", "RR"
    std::string type;      // "hip", "thigh", "calf"
    int channel;           // servo_id = PCA9685 channel
    bool inverted;
    double min_rad;
    double max_rad;
    double neutral_rad;
};

struct BalancePID {
    double kp, ki, kd;
};

class ServoConfigReader {
public:
    explicit ServoConfigReader(const std::string& config_path = "/home/sg/robot-dog/servo_config.json");
    
    // Reload config from file
    void reload();
    
    // Check if file was modified since last load
    bool is_changed() const;
    
    // Check if config was successfully loaded at least once
    bool is_loaded() const;
    
    // Joint queries
    std::vector<JointConfig> get_all_joints() const;
    std::optional<JointConfig> get_joint(const std::string& name) const;
    
    // Body/geometry
    double get_hip_length() const;
    double get_thigh_length() const;
    double get_calf_length() const;
    double get_body_length() const;
    double get_body_width() const;
    double get_standing_height() const;
    double get_min_height() const;
    double get_max_height() const;
    double get_foot_x_scale() const;
    double get_foot_y_scale() const;
    
    // Gait params
    double get_gait_period() const;
    double get_step_height() const;
    double get_turn_gain() const;
    double get_cog_pitch_gain() const;
    double get_cog_roll_gain() const;
    
    // Balance PID
    BalancePID get_pid_roll() const;
    BalancePID get_pid_pitch() const;
    BalancePID get_pid_height() const;
    double get_max_roll_correction() const;
    double get_max_pitch_correction() const;
    double get_max_height_correction() const;
    double get_max_integral_roll() const;
    double get_max_integral_pitch() const;
    double get_max_integral_height() const;
    double get_max_correction_rate() const;
    
    // Calibration paths & flags
    std::string get_calibration_dir() const;
    std::string get_zero_offsets_file() const;
    std::string get_joint_limits_file() const;
    std::vector<bool> get_inverted_flags() const;

private:
    std::string config_path_;
    nlohmann::json config_;
    std::filesystem::file_time_type last_mtime_;
    bool loaded_ = false;
    
    void load_from_json();
};

} // namespace dog_hardware_cpp
