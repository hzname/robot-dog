// locomotion_node: operator commands -> joint targets.
//
// Subscribes (relative names, launched in the /dog namespace):
//   cmd_vel        geometry_msgs/Twist    linear.x/y [m/s], angular.z [rad/s]
//   command        std_msgs/String        "stand" | "lie" | "greet" | "survey" | "crawl" | "trot"
//   body_pose      geometry_msgs/Vector3  x=roll [rad], y=pitch [rad], z=height offset [m]
//   estop          std_msgs/Bool          true = limp, requires "stand" after release
//   imu/data       sensor_msgs/Imu        optional: slope compensation and heading hold
//   terrain/profile std_msgs/Float32MultiArray  optional, for the crawl gait: [x0, dx, n,
//                  n heights on the left foot line, n on the right] (body frame x, odom z)
//   guard          std_msgs/Float64MultiArray  optional, from perception: [max forward
//                  speed m/s (inf = none), swing height m per leg LF, RF, LR, RR
//                  (NaN = default), gait (0 trot, 1 crawl), sideways velocity m/s
//                  to go round an obstacle]; dropped
//                  after guard_timeout without messages
// Gait period: from gait.period, or with gait.auto_period computed from
// servo.max_speed [rad/s], servo.margin and servo.knee_ratio at start and on
// live changes (D-12). Changes of gait.period, gait.min_period [s], servo.*
// and gait.auto_period via `ros2 param set` are accepted only in PASSIVE,
// STAND and LYING - WALK and transitions reject them; when no period up to
// 1.5 s fits, the node exits with code 1.
// Publishes:
//   joint_commands sensor_msgs/JointState 12 joint positions [rad]
//   state          std_msgs/String        current mode, or "estop" (latched)
//   odom           nav_msgs/Odometry      only with odom.publish (the real robot): pose
//                  dead-reckoned from the walked twist and the IMU heading, 25 Hz.
//                  In simulation Gazebo publishes the true pose on it instead
//                  (odom.topic moves dead reckoning elsewhere to compare).
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dog_control/locomotion.hpp"
#include "dog_diagnostics/reporter.hpp"
#include "dog_control/odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

namespace dog_control
{

namespace
{
// Copy of the helper in servo_driver_node.cpp (dog_hardware is Phase 3
// territory): numbers from YAML or `ros2 param set` may arrive as integer or
// double.
double asNumber(const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  }
  return p.as_double();  // throws InvalidParameterTypeException otherwise
}
}  // namespace

const std::vector<std::string> kJointNames = {
  "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint",
  "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
  "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint",
  "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint",
};

class LocomotionNode : public rclcpp::Node
{
public:
  explicit LocomotionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("locomotion", options)
  {
    const LocomotionParams p = loadParams();
    controller_ = std::make_unique<LocomotionController>(p);
    gait_period_ = p.gait.period;
    gait_auto_ = p.auto_period;
    gait_min_period_ = p.min_period;
    servo_ = p.servo;
    heading_debug_ = p.heading_debug;
    if (gait_auto_) {
      RCLCPP_INFO(get_logger(),
        "gait period %.4f s computed from servo.max_speed %.2f, margin %.2f, knee_ratio %.3f",
        controller_->gaitPeriod(), servo_.max_speed, servo_.margin, servo_.knee_ratio);
    } else {
      RCLCPP_INFO(get_logger(), "gait period %.4f s (manual gait.period)", controller_->gaitPeriod());
    }
    rate_ = declare_parameter("control_rate", 50.0);
    cmd_timeout_ = declare_parameter("cmd_vel_timeout", 0.5);
    guard_timeout_ = declare_parameter("guard_timeout", 1.0);
    odom_publish_ = declare_parameter("odom.publish", false);
    // the simulation publishes the true pose on "odom": dead reckoning goes
    // elsewhere there (localization tests compare the two)
    const auto odom_topic = declare_parameter("odom.topic", std::string("odom"));
    // heading: "imu" = the IMU orientation's yaw (on the robot imu_node
    // integrates the gyro itself), "gyro" = integrate angular_velocity.z here,
    // plus gyro_bias_dps - a drifting gyro for the simulation, whose IMU yaw is true
    odom_gyro_ = declare_parameter("odom.yaw_source", std::string("imu")) == "gyro";
    odom_gyro_bias_ = declare_parameter("odom.gyro_bias_dps", 0.0) * M_PI / 180.0;
    // the gait walks a little less than it steps for (slip): measured once by
    // walking a known distance (DEPLOYMENT.md), the walked twist is scaled by it
    odom_scale_ = declare_parameter("odom.scale", 1.0);
    odom_height_ = p.stand_height;

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
    if (odom_publish_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    }
    state_pub_ = create_publisher<std_msgs::msg::String>("state", latched);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        controller_->setVelocity({msg->linear.x, msg->linear.y, msg->angular.z});
        last_cmd_vel_ = now();
        cmd_vel_active_ = true;
      });
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (controller_->request(msg->data)) {
          RCLCPP_INFO(get_logger(), "command '%s' accepted", msg->data.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "command '%s' rejected (mode %s%s): %s", msg->data.c_str(),
            modeName(controller_->mode()), controller_->estopActive() ? ", e-stop active" : "",
            controller_->lastRefusal().c_str());
        }
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "body_pose", 10, [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
        controller_->setBodyPose({msg->x, msg->y, msg->z});
      });
    // E-stop is volatile on purpose: with several latched publishers a late
    // subscriber would get their last values in undefined order. Nodes start
    // in a safe state (PASSIVE / servos off) instead.
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "estop", rclcpp::QoS(10).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data != controller_->estopActive()) {
          RCLCPP_WARN(get_logger(), "E-STOP %s", msg->data ? "ENGAGED" : "released");
        }
        controller_->setEstop(msg->data);
      });

    // imu_node tells the world that its gyro bias is known; latched, so a node
    // that starts later still learns it (SAF-06).
    const auto latched_bool = rclcpp::QoS(1).transient_local().reliable();
    imu_calibrated_sub_ = create_subscription<std_msgs::msg::Bool>(
      "imu/calibrated", latched_bool, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data != controller_->imuCalibrated()) {
          RCLCPP_INFO(get_logger(), "imu calibrated: %s", msg->data ? "yes" : "no");
        }
        controller_->setImuCalibrated(msg->data);
      });

    estop_pub_ = create_publisher<std_msgs::msg::Bool>("estop", rclcpp::QoS(10).reliable());
    // Latched so a tool started later (keyboard_teleop, a script) sees the profile
    // this robot was launched with. profile:=floor makes that tool stand down.
    profile_pub_ = create_publisher<std_msgs::msg::String>(
      "profile", rclcpp::QoS(1).reliable().transient_local());
    {
      std_msgs::msg::String msg;
      msg.data = risk_profile_;
      profile_pub_->publish(msg);   // loadParams() ran before this line
      RCLCPP_INFO(get_logger(), "profile: %s", risk_profile_.c_str());
    }
    diag_ = std::make_unique<dog_diagnostics::Reporter>(this, "locomotion");
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {report();});
    release_client_ = create_client<std_srvs::srv::Trigger>("servo_driver/release_fault");
    arm_client_ = create_client<std_srvs::srv::Trigger>("servo_driver/arm");
    recover_srv_ = create_service<std_srvs::srv::Trigger>("~/recover",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          // One operator gesture (D-12): check the robot is level, clear the fall,
          // clear the driver's latch and e-stop, arm it, then stand up.
          const std::string why = controller_->recover();
          if (!why.empty()) {
            res->success = false;
            res->message = why;
            RCLCPP_WARN(get_logger(), "recover refused: %s", why.c_str());
            return;
          }
          publishEstop(false);
          requestDriverRelease();
          res->success = true;
          res->message = "recovering: standing up";
          RCLCPP_WARN(get_logger(), "recover: %s", res->message.c_str());
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        const double stamp = rclcpp::Time(msg->header.stamp).seconds();
        const double gdt = gyro_stamp_ > 0.0 ? std::clamp(stamp - gyro_stamp_, 0.0, 0.1) : 0.0;
        gyro_stamp_ = stamp;
        controller_->addYawRate(msg->angular_velocity.z, gdt);
        // the heading for dead reckoning ("gyro"): the rate minus its bias,
        // learnt whenever the robot is still - lying at start (as imu_node
        // does once) and standing on its feet later (the survey and the
        // greeting turn the body: not then), as the bias wanders with temperature
        // over the whole gap since the last message (messages get lost when the
        // computer is busy): the IMU's own yaw change where it gives one (on the
        // robot imu_node integrates the gyro at its full rate), else rate * dt
        const double rdt = prev_stamp_ > 0.0 ? std::clamp(stamp - prev_stamp_, 0.0, 2.0) : 0.0;
        prev_stamp_ = stamp;
        double dyaw = msg->angular_velocity.z * gdt;
        if (msg->orientation_covariance[0] >= 0.0) {
          const auto & o = msg->orientation;
          const double y = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
          dyaw = have_prev_yaw_ ? std::remainder(y - prev_yaw_, 2.0 * M_PI) : 0.0;
          prev_yaw_ = y;
          have_prev_yaw_ = true;
        }
        dyaw += odom_gyro_bias_ * rdt;  // a drifting gyro (simulation)
        const double wz = rdt > 0.0 ? dyaw / rdt : 0.0;
        const Mode md = controller_->mode();
        const bool resting = md == Mode::PASSIVE || md == Mode::LYING ||
          (md == Mode::STAND && !controller_->gait().stepping() && !controller_->crawl().stepping());
        const bool still = resting && std::abs(wz - gyro_bias_est_) < 0.05;
        still_time_ = still ? still_time_ + rdt : 0.0;
        if (still_time_ > 0.5) {gyro_bias_est_ += (wz - gyro_bias_est_) * std::min(1.0, rdt / 2.0);}
        gyro_yaw_ += dyaw - gyro_bias_est_ * rdt;
        last_gyro_ = now();
        const auto & q = msg->orientation;
        if (msg->orientation_covariance[0] < 0.0) {return;}  // no orientation in this message
        const double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        const double pitch = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
        imu_rpy_ = {roll, pitch, std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
        const auto t = now();
        const double dt = imu_seen_ ? std::clamp((t - last_imu_).seconds(), 0.0, 0.2) : 0.0;
        if (!imu_seen_) {
          RCLCPP_INFO(get_logger(), "IMU data received - slope compensation and heading hold active");
        }
        imu_seen_ = true;
        last_imu_ = t;
        controller_->setImuAttitude(roll, pitch, dt);
        // The fall guard reads the accelerometer's own direction: the filter's tau
        // is a second, and a robot tipping over reaches 30 degrees in a fraction of
        // one. The plausibility window keeps an impact out of the estimate.
        // The sign of gravity is not the same everywhere: our MPU driver and the mock
        // report +g on z at rest, the Gazebo plugin reports -g, and the first version
        // read that as upside down and latched a fall at spawn. Point the vector "up"
        // before reading the angle out of it.
        auto a = msg->linear_acceleration;
        if (a.z < 0.0) {
          a.x = -a.x;
          a.y = -a.y;
          a.z = -a.z;
        }
        const double norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        // Only a plausible "at rest" read may feed the guard: a landing spike or a
        // bounce distorts the vector (the simulator spawns the robot from 0.25 m and
        // the impact read as 46 degrees for a fifth of a second), while a robot
        // actually toppling still reads close to 1 g.
        if (norm > 0.8 * 9.80665 && norm < 1.2 * 9.80665) {
          const double roll_acc = std::atan2(a.y, a.z);
          const double pitch_acc = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
          controller_->setFallTilt(
            std::max(std::abs(roll_acc), std::abs(roll)) * (roll_acc < 0 ? -1.0 : 1.0),
            std::max(std::abs(pitch_acc), std::abs(pitch)) * (pitch_acc < 0 ? -1.0 : 1.0));
        } else {
          // Implausible sample: hold the guard to the filter, which also resets its
          // debounce instead of counting the spike towards a fall.
          controller_->setFallTilt(roll, pitch);
        }
      });

    guard_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "guard", 10, [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        // [max_vx, step] (all legs) or [max_vx, step LF, RF, LR, RR]
        if (msg->data.size() < 2) {return;}
        const double vmax = msg->data[0];
        std::array<double, kNumLegs> steps;
        for (int leg = 0; leg < kNumLegs; ++leg) {
          steps[leg] = msg->data.size() >= 1 + kNumLegs ? msg->data[1 + leg] : msg->data[1];
        }
        if (vmax != last_guard_vx_) {
          if (vmax == 0.0) {
            RCLCPP_WARN(get_logger(), "guard: hazard ahead - forward motion stopped");
          } else if (std::isfinite(vmax)) {
            RCLCPP_INFO(get_logger(), "guard: forward speed limited to %.2f m/s", vmax);
          } else {
            RCLCPP_INFO(get_logger(), "guard: clear");
          }
          last_guard_vx_ = vmax;
        }
        controller_->setGuard(vmax, steps);
        if (msg->data.size() >= 3 + kNumLegs) {
          controller_->setGuardGait(msg->data[1 + kNumLegs] > 0.5 ? GaitType::CRAWL : GaitType::TROT,
            msg->data[2 + kNumLegs]);
        }
        last_guard_ = now();
        guard_active_ = true;
      });

    terrain_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "terrain/profile", 10, [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        const auto & d = msg->data;
        if (d.size() < 3) {return;}
        const size_t n = static_cast<size_t>(d[2]);
        if (d.size() < 3 + 2 * n || n == 0) {return;}
        TerrainProfile t;
        t.x0 = d[0];
        t.dx = d[1];
        t.left.assign(d.begin() + 3, d.begin() + 3 + n);
        t.right.assign(d.begin() + 3 + n, d.begin() + 3 + 2 * n);
        t.fillGaps();  // the shadow behind a bar or beyond a drop
        controller_->setTerrain(t);
        last_terrain_ = now();
        terrain_active_ = true;
      });

    joint_msg_.name = kJointNames;
    joint_msg_.position.resize(kNumJoints);
    last_tick_ = now();
    last_cmd_vel_ = now();
    // Node clock: follows /clock when use_sim_time is set (Gazebo).
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this]() {tick();});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return onParams(params);});
    publishState();
    RCLCPP_INFO(get_logger(), "locomotion ready at %.0f Hz (send \"stand\" on ~/command)", rate_);
  }

private:
  LocomotionParams loadParams()
  {
    LocomotionParams p;
    p.leg.hip = declare_parameter("geometry.hip_offset", p.leg.hip);
    p.leg.thigh = declare_parameter("geometry.thigh", p.leg.thigh);
    p.leg.calf = declare_parameter("geometry.calf", p.leg.calf);
    p.hip_x = declare_parameter("geometry.hip_x", p.hip_x);
    p.hip_y = declare_parameter("geometry.hip_y", p.hip_y);
    p.knee_direction = static_cast<int>(declare_parameter("geometry.knee_direction", -1));

    p.foot_offset_x = declare_parameter("stance.foot_offset_x", p.foot_offset_x);
    p.foot_offset_y = declare_parameter("stance.foot_offset_y", p.foot_offset_y);
    p.stand_height = declare_parameter("stance.stand_height", p.stand_height);
    p.lie_height = declare_parameter("stance.lie_height", p.lie_height);
    p.min_height = declare_parameter("stance.min_height", p.min_height);
    p.max_height = declare_parameter("stance.max_height", p.max_height);
    p.transition_time = declare_parameter("stance.transition_time", p.transition_time);
    p.max_roll = declare_parameter("stance.max_roll", p.max_roll);
    p.max_pitch = declare_parameter("stance.max_pitch", p.max_pitch);

    p.gait.period = declare_parameter("gait.period", p.gait.period);
    p.gait.duty = declare_parameter("gait.duty", p.gait.duty);
    p.gait.step_height = declare_parameter("gait.step_height", p.gait.step_height);
    p.gait.max_step = declare_parameter("gait.max_step", p.gait.max_step);
    p.auto_period = declare_parameter("gait.auto_period", p.auto_period);
    p.min_period = declareNumber("gait.min_period", p.min_period);
    p.servo.max_speed = declareNumber("servo.max_speed", p.servo.max_speed);
    p.servo.margin = declareNumber("servo.margin", p.servo.margin);
    p.servo.knee_ratio = declareNumber("servo.knee_ratio", p.servo.knee_ratio);
    p.crawl.shift_time = declare_parameter("crawl.shift_time", p.crawl.shift_time);
    p.crawl.swing_time = declare_parameter("crawl.swing_time", p.crawl.swing_time);
    p.crawl.max_stride = declare_parameter("crawl.max_stride", p.crawl.max_stride);
    p.crawl.clearance = declare_parameter("crawl.clearance", p.crawl.clearance);
    p.crawl.max_lift = declare_parameter("crawl.max_lift", p.crawl.max_lift);
    p.crawl.shift_rate = declare_parameter("crawl.shift_rate", p.crawl.shift_rate);
    p.crawl.max_pitch = declare_parameter("crawl.max_pitch", p.crawl.max_pitch);
    p.crawl.shift_accel = declare_parameter("crawl.shift_accel", p.crawl.shift_accel);
    p.greet.rear_x = declare_parameter("greet.rear_x", p.greet.rear_x);
    p.greet.sit_deg = declare_parameter("greet.sit_deg", p.greet.sit_deg);
    p.greet.beg_deg = declare_parameter("greet.beg_deg", p.greet.beg_deg);
    p.greet.margin = declare_parameter("greet.margin", p.greet.margin);
    p.greet.waves = static_cast<int>(declare_parameter("greet.waves", static_cast<int64_t>(p.greet.waves)));
    p.greet.speed = declare_parameter("greet.speed", p.greet.speed);
    p.survey.pitch_up_deg = declare_parameter("survey.pitch_up_deg", p.survey.pitch_up_deg);
    p.survey.pitch_down_deg = declare_parameter("survey.pitch_down_deg", p.survey.pitch_down_deg);
    p.survey.yaw_deg = declare_parameter("survey.yaw_deg", p.survey.yaw_deg);
    p.survey.cycles = static_cast<int>(declare_parameter("survey.cycles", static_cast<int64_t>(p.survey.cycles)));
    p.survey.segment_time = declare_parameter("survey.segment_time", p.survey.segment_time);
    // it kneels on the rear knees: knee and foot contacts of the description
    p.greet.contact_r = declare_parameter("description.foot_radius", p.greet.contact_r);

    p.slope_compensation = declare_parameter("slope.compensation", p.slope_compensation);
    p.slope_gain = declare_parameter("slope.gain", p.slope_gain);
    p.slope_filter_tau = declare_parameter("slope.filter_tau", p.slope_filter_tau);
    p.slope_max_shift = declare_parameter("slope.max_shift", p.slope_max_shift);
    p.slope_max_deg = declare_parameter("slope.max_deg", p.slope_max_deg);
    p.heading_hold = declare_parameter("heading.hold", p.heading_hold);
    p.heading_kp = declare_parameter("heading.kp", p.heading_kp);
    p.heading_ki = declare_parameter("heading.ki", p.heading_ki);
    p.heading_max_rate = declare_parameter("heading.max_rate", p.heading_max_rate);
    p.heading_max_error = declare_parameter("heading.max_error", p.heading_max_error);
    p.heading_crawl_kp = declare_parameter("heading.crawl_kp", p.heading_crawl_kp);
    p.heading_crawl_ki = declare_parameter("heading.crawl_ki", p.heading_crawl_ki);
    p.heading_crawl_max_rate = declare_parameter("heading.crawl_max_rate", p.heading_crawl_max_rate);
    p.heading_debug = declare_parameter("heading.debug", p.heading_debug);
    p.tilt_soft_deg = declare_parameter("tilt.soft_deg", p.tilt_soft_deg);
    p.tilt_stop_deg = declare_parameter("tilt.stop_deg", p.tilt_stop_deg);
    // Fall protection (phase 4). The profile says whether the robot works on the
    // floor, where a missing IMU must stop it, or on the bench, where the IMU may
    // be absent altogether (SAF-06).
    p.safety_soft_deg = declare_parameter("safety.soft_deg", p.safety_soft_deg);
    p.safety_hard_deg = declare_parameter("safety.hard_deg", p.safety_hard_deg);
    p.safety_debounce_s = declare_parameter("safety.debounce_s", p.safety_debounce_s);
    p.safety_hysteresis_deg = declare_parameter("safety.hysteresis_deg", p.safety_hysteresis_deg);
    p.safety_imu_timeout_s = declare_parameter("safety.imu_timeout_s", p.safety_imu_timeout_s);
    p.safety_idle_lie_s = declare_parameter("safety.idle_lie_s", p.safety_idle_lie_s);
    const auto profile = declare_parameter("safety.profile", std::string("bench"));
    if (profile != "floor" && profile != "bench") {
      throw std::invalid_argument("safety.profile must be 'floor' or 'bench', got '" + profile + "'");
    }
    p.safety_require_imu = profile == "floor";
    risk_profile_ = profile;

    p.max_velocity.vx = declare_parameter("limits.max_vx", p.max_velocity.vx);
    p.max_velocity.vy = declare_parameter("limits.max_vy", p.max_velocity.vy);
    p.max_velocity.wz = declare_parameter("limits.max_wz", p.max_velocity.wz);
    p.max_accel.vx = declare_parameter("limits.accel_vx", p.max_accel.vx);
    p.max_accel.vy = declare_parameter("limits.accel_vy", p.max_accel.vy);
    p.max_accel.wz = declare_parameter("limits.accel_wz", p.max_accel.wz);
    return p;
  }

  double declareNumber(const std::string & name, double default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    const auto v = declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
    return asNumber(rclcpp::Parameter(name, v));
  }

  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // The whole set applies atomically: one reconfigureGait call with the
    // merged values (T-01-08-03).
    double period = gait_period_;
    bool auto_period = gait_auto_;
    double min_period = gait_min_period_;
    ServoSpeedModel servo = servo_;
    bool seen = false;
    for (const auto & param : params) {
      const auto & name = param.get_name();
      if (name == "gait.auto_period") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          result.successful = false;
          result.reason = "gait.auto_period: expected a boolean";
          return result;
        }
        auto_period = param.as_bool();
        seen = true;
      } else if (name == "gait.period" || name == "gait.min_period" || name == "servo.max_speed" ||
        name == "servo.margin" || name == "servo.knee_ratio")
      {
        double v = 0.0;
        try {
          v = asNumber(param);
        } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
          result.successful = false;
          result.reason = name + ": " + e.what();
          return result;
        }
        if (name == "gait.period") {period = v;}
        else if (name == "gait.min_period") {min_period = v;}
        else if (name == "servo.max_speed") {servo.max_speed = v;}
        else if (name == "servo.margin") {servo.margin = v;}
        else {servo.knee_ratio = v;}
        seen = true;
      }
    }
    if (!seen || (period == gait_period_ && auto_period == gait_auto_ && min_period == gait_min_period_ &&
      servo.max_speed == servo_.max_speed && servo.margin == servo_.margin &&
      servo.knee_ratio == servo_.knee_ratio))
    {
      return result;  // nothing of ours changed: leave the controller alone
    }
    if (controller_->reconfigureGait(period, auto_period, min_period, servo)) {
      gait_period_ = period;
      gait_auto_ = auto_period;
      gait_min_period_ = min_period;
      servo_ = servo;
      RCLCPP_INFO(get_logger(), "gait period %.4f s after parameter change", controller_->gaitPeriod());
      return result;
    }
    result.successful = false;
    if (!controller_->gaitReconfigurable()) {
      result.reason = "period change rejected in mode " + std::string(modeName(controller_->mode()));
    } else {
      std::ostringstream msg;
      msg << "no gait period up to " << kMaxAutoPeriod << " s fits servo.margin * servo.max_speed"
          << " (or the manual period is not a finite number above 0)";
      result.reason = msg.str();
    }
    RCLCPP_WARN(get_logger(), "gait period change rejected: %s", result.reason.c_str());
    return result;
  }

  void tick()
  {
    const auto t = now();
    const double dt = std::clamp((t - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = t;

    if (cmd_vel_active_ && (t - last_cmd_vel_).seconds() > cmd_timeout_) {
      controller_->setVelocity({});
      cmd_vel_active_ = false;
      RCLCPP_WARN(get_logger(), "cmd_vel timeout (%.2fs) - stopping", cmd_timeout_);
    }

    if (guard_active_ && (t - last_guard_).seconds() > guard_timeout_) {
      controller_->clearGuard();
      guard_active_ = false;
      last_guard_vx_ = std::numeric_limits<double>::infinity();
      RCLCPP_WARN(get_logger(), "guard silent for %.1fs - hazard limits dropped", guard_timeout_);
    }

    if (terrain_active_ && (t - last_terrain_).seconds() > 0.5) {
      controller_->clearTerrain();  // stale: the crawl walks as on flat ground
      terrain_active_ = false;
    }

    if (last_gyro_.nanoseconds() != 0 && (t - last_gyro_).seconds() > 0.2) {
      controller_->clearYawRate();  // IMU silent: no heading hold on stale data
    }
    const bool out = controller_->update(dt);
    if (heading_debug_ && (++heading_debug_tick_ % 10) == 0) {
      // 5 Hz at the 50 Hz loop: err, integral, the applied correction and the
      // gyro rate, so the acceptance trace can tell a steady drift from a step.
      // tilt is the governor's own input (the worse of pitch and roll, degrees) and
      // scale what it did with it: the trace then answers whether the governor
      // interfered with a cell, not just whether the walk was weak (02-03).
      const double tilt_deg = 180.0 / M_PI * std::max(
        std::abs(controller_->slopePitch()), std::abs(controller_->slopeRoll()));
      RCLCPP_INFO(get_logger(),
        "heading_debug err=%.3f int=%.3f corr=%.3f wz_meas=%.3f tilt=%.1f scale=%.2f",
        controller_->headingError(), controller_->headingIntegral(),
        controller_->headingCorrection(), controller_->yawRateMeasured(),
        tilt_deg, controller_->tiltScale());
    }
    if (controller_->tiltScale() < 1.0) {
      // The governor speaks up once a second, not every tick: a slowed walk must
      // be explainable from the log, and a stand on a slope must not look like a
      // stuck robot (GAIT-04).
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "tilt_guard scale=%.2f pitch=%.1f roll=%.1f",
        controller_->tiltScale(), controller_->slopePitch() * 180.0 / M_PI,
        controller_->slopeRoll() * 180.0 / M_PI);
    }
    if (controller_->fallen() && !fall_announced_) {
      fall_announced_ = true;
      RCLCPP_ERROR(get_logger(), "FALLEN: %s - call ~/recover to raise the robot",
        controller_->fallReason().c_str());
    }
    if (controller_->takeRelaxRequest()) {
      // A fall or the thermal timeout: the operator must not have to know the
      // driver's protocol to make the servos let go.
      publishEstop(true);
      RCLCPP_WARN(get_logger(), "outputs released: %s",
        controller_->fallen() ? controller_->fallReason().c_str() : "thermal timeout, robot is lying");
    }
    if (odom_publish_) {publishOdom(t, dt);}
    if (controller_->mode() != last_mode_ || controller_->estopActive() != last_estop_) {
      publishState();
    }
    if (controller_->unreachableCount() > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "%d foot target(s) outside the leg workspace - clamped", controller_->unreachableCount());
    }
    if (!out) {
      return;
    }
    joint_msg_.header.stamp = t;
    const auto & q = controller_->joints();
    std::copy(q.begin(), q.end(), joint_msg_.position.begin());
    joint_pub_->publish(joint_msg_);
  }

  /// Mode, tilt and the reason the robot stopped - the page reads the same words.
  void report()
  {
    using dog_diagnostics::Level;
    Level level = Level::OK;
    std::string reason = modeName(controller_->mode());
    if (controller_->fallen()) {
      level = Level::ERROR;
      reason = "fallen: " + controller_->fallReason();
    } else if (controller_->estopActive()) {
      level = Level::WARN;
      reason = "e-stop engaged";
    } else if (controller_->tiltScale() < 1.0) {
      level = Level::WARN;
      reason = "tilt " + std::to_string(static_cast<int>(controller_->slopePitch() * 180.0 / M_PI)) +
        " deg: the walk slows down";
    }
    char tilt[32];
    std::snprintf(tilt, sizeof(tilt), "%.1f", controller_->tiltDeg());
    diag_->publish(level, reason, {
      {"mode", modeName(controller_->mode())},
      {"tilt_deg", tilt},
      {"profile", risk_profile_},
      {"imu_calibrated", controller_->imuCalibrated() ? "true" : "false"}});
  }

  void publishEstop(bool active)
  {
    estop_pub_->publish(std_msgs::msg::Bool().set__data(active));
  }

  /// Ask the driver to drop its latch, then to arm: without the arm the driver
  /// ignores the joint commands the stand-up is about to publish.
  void requestDriverRelease()
  {
    recover_attempts_ = 0;
    releaseAndArm();
  }

  /// One pass of "drop the latch, then arm". A fallen robot stops sending commands,
  /// so the driver's own timeout can latch a fault in the same moment the operator
  /// asks to recover; a single retry covers that race.
  void releaseAndArm()
  {
    if (!release_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "recover: servo driver release_fault is not reachable");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    release_client_->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto res = future.get();
        RCLCPP_WARN(get_logger(), "recover: release_fault -> %s", res->message.c_str());
        requestDriverArm();
      });
  }

  void requestDriverArm()
  {
    if (!arm_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "recover: servo driver arm is not reachable");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    arm_client_->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto res = future.get();
        if (res->success) {
          controller_->request("stand");
          RCLCPP_WARN(get_logger(), "recover: driver %s, standing up", res->message.c_str());
        } else if (++recover_attempts_ < 2) {
          RCLCPP_WARN(get_logger(), "recover: driver refused to arm (%s) - retrying the release",
            res->message.c_str());
          releaseAndArm();
        } else {
          RCLCPP_ERROR(get_logger(), "recover: driver refused to arm: %s", res->message.c_str());
        }
      });
  }

  void publishOdom(const rclcpp::Time & t, double dt)
  {
    const bool imu = imu_seen_ && (t - last_imu_).seconds() < 0.2;
    const auto & v = controller_->gaitVelocity();
    // trot or crawl: gaitVelocity is what the stepping gait walks
    const bool moving = controller_->gait().stepping() || controller_->crawl().stepping();
    const double yaw_in = !imu ? std::nan("") : odom_gyro_ ? gyro_yaw_ : imu_rpy_[2];
    dead_reckoning_.update(dt, moving ? odom_scale_ * v.vx : 0.0, moving ? odom_scale_ * v.vy : 0.0,
      moving ? v.wz : 0.0, yaw_in);
    if (++odom_div_ % 2) {return;}  // 25 Hz at the 50 Hz control rate
    nav_msgs::msg::Odometry m;
    m.header.stamp = t;
    m.header.frame_id = "odom";
    m.child_frame_id = "base_link";
    m.pose.pose.position.x = dead_reckoning_.x();
    m.pose.pose.position.y = dead_reckoning_.y();
    m.pose.pose.position.z = odom_height_ + controller_->baseHeight();  // climbs with the crawl
    const double r = imu ? imu_rpy_[0] : 0.0, p = imu ? imu_rpy_[1] : 0.0, y = dead_reckoning_.yaw();
    const double cr = std::cos(r / 2), sr = std::sin(r / 2), cp = std::cos(p / 2), sp = std::sin(p / 2);
    const double cy = std::cos(y / 2), sy = std::sin(y / 2);
    m.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    m.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    m.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    m.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    m.twist.twist.linear.x = moving ? v.vx : 0.0;
    m.twist.twist.linear.y = moving ? v.vy : 0.0;
    m.twist.twist.angular.z = moving ? v.wz : 0.0;
    odom_pub_->publish(m);
  }

  void publishState()
  {
    last_mode_ = controller_->mode();
    last_estop_ = controller_->estopActive();
    std_msgs::msg::String msg;
    // A fall outranks the e-stop banner it raised: "estop" would hide the reason.
    msg.data = (last_estop_ && last_mode_ != Mode::FALLEN) ? "estop" : modeName(last_mode_);
    state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "mode -> %s", msg.data.c_str());
  }

  std::unique_ptr<LocomotionController> controller_;
  double gait_period_{0.0};
  bool gait_auto_{false};
  double gait_min_period_{0.0};
  ServoSpeedModel servo_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  double rate_{50.0};
  double cmd_timeout_{0.5};
  bool cmd_vel_active_{false};
  double guard_timeout_{1.0};
  bool guard_active_{false};
  double last_guard_vx_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_guard_;
  Mode last_mode_{Mode::PASSIVE};
  bool last_estop_{false};
  bool imu_seen_{false};
  bool odom_publish_{false};
  bool heading_debug_{false};
  int heading_debug_tick_{0};
  bool odom_gyro_{false};
  double odom_gyro_bias_{0.0}, odom_scale_{1.0};
  double gyro_yaw_{0.0}, gyro_bias_est_{0.0}, still_time_{0.0}, prev_stamp_{0.0}, prev_yaw_{0.0};
  bool have_prev_yaw_{false};
  double odom_height_{0.15};
  int odom_div_{0};
  std::array<double, 3> imu_rpy_{};
  DeadReckoning dead_reckoning_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Time last_imu_;
  rclcpp::Time last_gyro_{0, 0, RCL_ROS_TIME};
  double gyro_stamp_{0.0};
  rclcpp::Time last_tick_;
  rclcpp::Time last_cmd_vel_;
  sensor_msgs::msg::JointState joint_msg_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr guard_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr terrain_sub_;
  rclcpp::Time last_terrain_;
  bool terrain_active_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_calibrated_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
  std::unique_ptr<dog_diagnostics::Reporter> diag_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  bool fall_announced_{false};
  int recover_attempts_{0};
  std::string risk_profile_{"bench"};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr profile_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dog_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<dog_control::LocomotionNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("locomotion"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
