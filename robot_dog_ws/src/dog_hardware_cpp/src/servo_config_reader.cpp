#include "dog_hardware_cpp/servo_config_reader.hpp"
#include <fstream>
#include <stdexcept>
#include <cstdio>

// Logging — works with or without rclcpp
#ifndef RCLCPP_WARN
#define RCLCPP_WARN(ctx, fmt, ...) fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__)
#endif

// Suppress unused variable warnings for config_
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-private-field"
#endif

namespace dog_hardware_cpp {

ServoConfigReader::ServoConfigReader(const std::string& config_path)
    : config_path_(config_path) {}

void ServoConfigReader::reload() {
    load_from_json();
}

bool ServoConfigReader::is_changed() const {
    if (!loaded_) return true;
    auto current = std::filesystem::last_write_time(config_path_);
    return current != last_mtime_;
}

bool ServoConfigReader::is_loaded() const {
    return loaded_;
}

void ServoConfigReader::load_from_json() {
    std::ifstream file(config_path_);
    if (!file.is_open()) {
        RCLCPP_WARN(nullptr, "ServoConfigReader: Cannot open %s, using defaults", config_path_.c_str());
        config_ = nlohmann::json::object();
        loaded_ = false;
        return;
    }
    try {
        file >> config_;
        last_mtime_ = std::filesystem::last_write_time(config_path_);
        loaded_ = true;
    } catch (const nlohmann::json::parse_error& e) {
        RCLCPP_WARN(nullptr, "ServoConfigReader: JSON parse error: %s", e.what());
        loaded_ = false;
    }
}

// Helper: get nested json object by path - returns a copy to avoid dangling references
static nlohmann::json get_nested(const nlohmann::json& obj, const std::vector<std::string>& keys, const nlohmann::json& def = nlohmann::json::object()) {
    const nlohmann::json* current = &obj;
    for (const auto& k : keys) {
        if (current->contains(k) && (*current)[k].is_object()) {
            current = &(*current)[k];
        } else {
            return def;
        }
    }
    return *current;  // Return by value to avoid dangling reference
}

// Helper: get value from json, with double default
static double get_value(const nlohmann::json& j, const std::string& key, double def) {
    if (j.contains(key) && j[key].is_number()) {
        return j[key].get<double>();
    }
    return def;
}
static int get_int(const nlohmann::json& j, const std::string& key, int def) {
    if (j.contains(key) && j[key].is_number()) {
        return j[key].get<int>();
    }
    return def;
}
static bool get_bool(const nlohmann::json& j, const std::string& key, bool def) {
    if (j.contains(key)) {
        return j[key].get<bool>();
    }
    return def;
}
static std::string get_str(const nlohmann::json& j, const std::string& key, const std::string& def) {
    if (j.contains(key) && j[key].is_string()) {
        return j[key].get<std::string>();
    }
    return def;
}

std::vector<JointConfig> ServoConfigReader::get_all_joints() const {
    std::vector<JointConfig> result;
    nlohmann::json joints = config_.value("joints", nlohmann::json::object());
    nlohmann::json list = joints.value("list", nlohmann::json::array());

    // Get type-level configs
    nlohmann::json hip_cfg = joints.value("hip", nlohmann::json::object());
    nlohmann::json thigh_cfg = joints.value("thigh", nlohmann::json::object());
    nlohmann::json calf_cfg = joints.value("calf", nlohmann::json::object());

    std::map<std::string, nlohmann::json> type_map = {
        {"hip", hip_cfg},
        {"thigh", thigh_cfg},
        {"calf", calf_cfg},
    };

    for (const auto& j : list) {
        std::string type = j.value("type", "hip");
        const nlohmann::json& type_cfg = type_map.count(type) ? type_map.at(type) : hip_cfg;

        JointConfig jc;
        jc.name = get_str(j, "name", "");
        jc.leg = get_str(j, "leg", "");
        jc.type = type;
        jc.channel = get_int(j, "servo_id", 0);
        jc.inverted = get_bool(j, "inverted", false);
        jc.min_rad = get_value(type_cfg, "min_rad", -0.8);
        jc.max_rad = get_value(type_cfg, "max_rad", 0.8);
        jc.neutral_rad = get_value(type_cfg, "neutral_rad", 0.0);
        result.push_back(jc);
    }
    return result;
}

std::optional<JointConfig> ServoConfigReader::get_joint(const std::string& name) const {
    auto joints = get_all_joints();
    for (const auto& j : joints) {
        if (j.name == name) return j;
    }
    return std::nullopt;
}

double ServoConfigReader::get_hip_length() const {
    auto& body = get_nested(config_, {"body"});
    auto& links = get_nested(body, {"link_lengths_m"});
    return get_value(links, "hip", 0.045);
}

double ServoConfigReader::get_thigh_length() const {
    auto& body = get_nested(config_, {"body"});
    auto& links = get_nested(body, {"link_lengths_m"});
    return get_value(links, "thigh", 0.11);
}

double ServoConfigReader::get_calf_length() const {
    auto& body = get_nested(config_, {"body"});
    auto& links = get_nested(body, {"link_lengths_m"});
    return get_value(links, "calf", 0.11);
}

double ServoConfigReader::get_body_length() const {
    auto& body = get_nested(config_, {"body"});
    return get_value(body, "length_m", 0.35);
}

double ServoConfigReader::get_body_width() const {
    auto& body = get_nested(config_, {"body"});
    return get_value(body, "width_m", 0.2);
}

double ServoConfigReader::get_standing_height() const {
    auto& standing = get_nested(config_, {"body", "standing"});
    auto& computed = get_nested(standing, {"_computed"});
    return get_value(computed, "standing_height_m", 0.25);
}

double ServoConfigReader::get_min_height() const {
    auto& standing = get_nested(config_, {"body", "standing"});
    auto& computed = get_nested(standing, {"_computed"});
    return get_value(computed, "min_height_m", 0.15);
}

double ServoConfigReader::get_max_height() const {
    auto& standing = get_nested(config_, {"body", "standing"});
    auto& computed = get_nested(standing, {"_computed"});
    return get_value(computed, "max_height_m", 0.35);
}

double ServoConfigReader::get_foot_x_scale() const {
    auto& body = get_nested(config_, {"body"});
    auto& standing = get_nested(body, {"standing"});
    return get_value(standing, "foot_x_scale", 0.5);
}

double ServoConfigReader::get_foot_y_scale() const {
    auto& body = get_nested(config_, {"body"});
    auto& standing = get_nested(body, {"standing"});
    return get_value(standing, "foot_y_scale", 0.8);
}

double ServoConfigReader::get_gait_period() const {
    auto& gait = get_nested(config_, {"gait"});
    auto& gaits = get_nested(gait, {"gaits"});
    auto& trot = get_nested(gaits, {"trot"});
    return get_value(trot, "period_s", 0.5);
}

double ServoConfigReader::get_step_height() const {
    auto& gait = get_nested(config_, {"gait"});
    auto& gaits = get_nested(gait, {"gaits"});
    auto& trot = get_nested(gaits, {"trot"});
    return get_value(trot, "step_height_m", 0.08);
}

double ServoConfigReader::get_turn_gain() const {
    auto& gait = get_nested(config_, {"gait"});
    auto& gaits = get_nested(gait, {"gaits"});
    auto& trot = get_nested(gaits, {"trot"});
    return get_value(trot, "turn_gain", 0.05);
}

double ServoConfigReader::get_cog_pitch_gain() const {
    auto& gait = get_nested(config_, {"gait"});
    auto& gaits = get_nested(gait, {"gaits"});
    auto& trot = get_nested(gaits, {"trot"});
    return get_value(trot, "cog_pitch_gain_m_per_rad", 0.02);
}

double ServoConfigReader::get_cog_roll_gain() const {
    auto& gait = get_nested(config_, {"gait"});
    auto& gaits = get_nested(gait, {"gaits"});
    auto& trot = get_nested(gaits, {"trot"});
    return get_value(trot, "cog_roll_gain_m_per_rad", 0.02);
}

BalancePID ServoConfigReader::get_pid_roll() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& pid = get_nested(balance, {"pid"});
    auto& roll = get_nested(pid, {"roll"});
    return {get_value(roll, "kp", 0.15), get_value(roll, "ki", 0.01), get_value(roll, "kd", 0.05)};
}

BalancePID ServoConfigReader::get_pid_pitch() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& pid = get_nested(balance, {"pid"});
    auto& pitch = get_nested(pid, {"pitch"});
    return {get_value(pitch, "kp", 0.15), get_value(pitch, "ki", 0.01), get_value(pitch, "kd", 0.05)};
}

BalancePID ServoConfigReader::get_pid_height() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& pid = get_nested(balance, {"pid"});
    auto& height = get_nested(pid, {"height"});
    return {get_value(height, "kp", 1.0), get_value(height, "ki", 0.1), get_value(height, "kd", 0.2)};
}

double ServoConfigReader::get_max_roll_correction() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& mc = get_nested(balance, {"max_correction"});
    return get_value(mc, "roll_rad", 0.08);
}

double ServoConfigReader::get_max_pitch_correction() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& mc = get_nested(balance, {"max_correction"});
    return get_value(mc, "pitch_rad", 0.08);
}

double ServoConfigReader::get_max_height_correction() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& mc = get_nested(balance, {"max_correction"});
    return get_value(mc, "height_m", 0.05);
}

double ServoConfigReader::get_max_integral_roll() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& mi = get_nested(balance, {"max_integral"});
    return get_value(mi, "roll", 0.5);
}

double ServoConfigReader::get_max_integral_pitch() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& mi = get_nested(balance, {"max_integral"});
    return get_value(mi, "pitch", 0.5);
}

double ServoConfigReader::get_max_integral_height() const {
    auto& balance = get_nested(config_, {"balance"});
    auto& mi = get_nested(balance, {"max_integral"});
    return get_value(mi, "height", 0.2);
}

double ServoConfigReader::get_max_correction_rate() const {
    auto& balance = get_nested(config_, {"balance"});
    return get_value(balance, "max_correction_rate", 0.5);
}

std::string ServoConfigReader::get_calibration_dir() const {
    auto& calib = get_nested(config_, {"calibration"});
    return get_str(calib, "calibration_dir", "/home/sg/robot-dog");
}

std::string ServoConfigReader::get_zero_offsets_file() const {
    auto& calib = get_nested(config_, {"calibration"});
    return get_str(calib, "zero_offsets_file", "calibration.json");
}

std::string ServoConfigReader::get_joint_limits_file() const {
    auto& calib = get_nested(config_, {"calibration"});
    return get_str(calib, "joint_limits_file", "joint_limits.json");
}

std::vector<bool> ServoConfigReader::get_inverted_flags() const {
    nlohmann::json joints = config_.value("joints", nlohmann::json::object());
    nlohmann::json list = joints.value("list", nlohmann::json::array());
    std::vector<bool> flags;
    for (const auto& j : list) {
        flags.push_back(get_bool(j, "inverted", false));
    }
    return flags;
}

} // namespace dog_hardware_cpp
