// perception_node: terrain perception from the crossed (X) lidars, the GS2
// line lidar and the VL53L1X ToF sensors, and the reaction to hazards.
//
// Subscribes (relative, /dog namespace):
//   lidar_left/scan, lidar_right/scan  sensor_msgs/LaserScan
//   gs2/scan                           sensor_msgs/LaserScan (YDLIDAR GS2 line laser)
//   tof/<name>                         sensor_msgs/Range
//   joint_states, imu/data, state, odom
// Publishes:
//   perception/ground_lidar  geometry_msgs/Vector3Stamped  x roll, y pitch [rad],
//                            z body height [m] over the floor plane seen by the lidars
//   perception/ground_feet   same, from the leg kinematics (four-leg support + IMU)
//   perception/hazards       std_msgs/String  JSON list of hazards ahead of the feet
//   perception/tof           std_msgs/Float32MultiArray  every ToF reading (only with
//                            perception.debug_topics): [sensor index, measured, expected,
//                            residual] (NaN = none)
//   perception/map           std_msgs/Float32MultiArray  elevation map (1 Hz):
//                            [origin_x, origin_y, resolution, n, n*n mean heights (NaN = unknown)]
//   guard                    std_msgs/Float64MultiArray  10 Hz, to locomotion: [max forward
//                            speed m/s (inf = no limit), swing height m per leg LF, RF, LR, RR
//                            (NaN = gait default), gait (0 trot, 1 crawl), sideways m/s (going round)]
//   perception/guard         std_msgs/String  JSON {state: clear|caution|step_over|crawl|stop|avoid,
//                            max_vx, step, d (nearest hazard ahead of the body centre), gait, vy, avoid}
//   terrain/profile          std_msgs/Float32MultiArray  10 Hz, for the crawl gait: [x0, dx, n,
//                            n ground heights on the left foot line, n on the right] (body x, odom z)
//   perception/stats         std_msgs/Float64MultiArray  [process CPU s, lidar cycles,
//                            lidar points, lidar ms total, ToF messages, ToF ms total,
//                            messages received (all topics), their callback ms total,
//                            then count / callback ms for joint_states, imu, odom,
//                            then GS2 scans, GS2 points, GS2 ms total]
//
// Every reading is compared with the ground plane at the reading's own time
// stamp (IMU history): the body pitches by several degrees at ~2 Hz in the
// trot, and 50 ms of skew between a sensor and the IMU is already 1 deg.
// The map is kept in the odom frame. In simulation odom is the ground truth;
// on the robot it will come from an estimate and drift.
#include <time.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "dog_perception/core.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace dog_perception
{

namespace
{
const char * kJoints[12] = {
  "lf_hip_joint", "lf_thigh_joint", "lf_calf_joint", "rf_hip_joint", "rf_thigh_joint", "rf_calf_joint",
  "lr_hip_joint", "lr_thigh_joint", "lr_calf_joint", "rr_hip_joint", "rr_thigh_joint", "rr_calf_joint"};
const std::map<std::string, double> kCorridorY{{"left", 0.12}, {"centre", 0.0}, {"right", -0.12}};

double stampSec(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}

double processCpu()
{
  timespec ts{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/// JSON number (4 decimals) or null.
std::string num(double v, int decimals = 4)
{
  if (!std::isfinite(v)) {return "null";}
  char b[40];
  std::snprintf(b, sizeof(b), "%.*f", decimals, v);
  return b;
}

std::string str(const std::string & s) {return "\"" + s + "\"";}

struct Hazard
{
  std::string source, kind, corridor, how;
  double x{kNaN}, y{kNaN}, h{kNaN}, jump{kNaN};
  double range{kNaN}, expected{kNaN}, residual{kNaN};
  std::string guard;
};
}  // namespace

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode()
  : rclcpp::Node("perception")
  {
    SensorParams s;
    s.x_lidar = getB("sensors.x_lidar", s.x_lidar);
    s.x_lidar_x = getD("sensors.x_lidar_x", s.x_lidar_x);
    s.x_lidar_y = getD("sensors.x_lidar_y", s.x_lidar_y);
    s.x_lidar_z = getD("sensors.x_lidar_z", s.x_lidar_z);
    s.x_lidar_tilt_deg = getD("sensors.x_lidar_tilt_deg", s.x_lidar_tilt_deg);
    s.x_lidar_yaw_deg = getD("sensors.x_lidar_yaw_deg", s.x_lidar_yaw_deg);
    s.tof = getB("sensors.tof", s.tof);
    s.tof_names = declare_parameter("sensors.tof_names", s.tof_names);
    s.tof_x = getDV("sensors.tof_x", s.tof_x);
    s.tof_y = getDV("sensors.tof_y", s.tof_y);
    s.tof_z = getDV("sensors.tof_z", s.tof_z);
    s.tof_pitch_deg = getDV("sensors.tof_pitch_deg", s.tof_pitch_deg);
    s.tof_yaw_deg = getDV("sensors.tof_yaw_deg", s.tof_yaw_deg);
    s.gs2 = getB("sensors.gs2", false);
    s.gs2_x = getD("sensors.gs2_x", s.gs2_x);
    s.gs2_y = getD("sensors.gs2_y", s.gs2_y);
    s.gs2_z = getD("sensors.gs2_z", s.gs2_z);
    s.gs2_pitch_deg = getD("sensors.gs2_pitch_deg", s.gs2_pitch_deg);
    geometry_.hip_offset = getD("geometry.hip_offset", geometry_.hip_offset);
    geometry_.thigh = getD("geometry.thigh", geometry_.thigh);
    geometry_.calf = getD("geometry.calf", geometry_.calf);
    geometry_.hip_x = getD("geometry.hip_x", geometry_.hip_x);
    geometry_.hip_y = getD("geometry.hip_y", geometry_.hip_y);

    thr_ = getD("perception.threshold", 0.015);
    // FC looks far and flat (20 deg): 1 deg of pitch error is 23 mm there
    const auto tof_thr = getDV("perception.tof_threshold", {0.015, 0.015, 0.05, 0.015});
    const int tof_confirm = getI("perception.tof_confirm", 3);
    // per-sensor range offsets measured once on a flat floor and kept in the
    // config; auto-calibration at every start would take a ramp in view as an
    // offset (found in the slope simulation)
    const auto tof_offsets = getDV("perception.tof_offsets", {0.0, 0.0, 0.0, 0.0});
    const bool tof_autocal = getB("perception.tof_autocal", false);
    // 'auto' = the lidar plane when it agrees with the legs, else the legs; 'feet' = legs only
    reference_ = declare_parameter("perception.reference", std::string("auto"));
    const double map_size = getD("perception.map_size", 3.0);
    const double map_res = getD("perception.map_resolution", 0.02);
    gs2_thr_ = getD("perception.gs2_threshold", 0.012);
    gs2_confirm_ = getI("perception.gs2_confirm", 8);
    // whole line off the leg plane: 15 mm gave false reports on a flat floor
    gs2_plane_thr_ = getD("perception.gs2_plane_threshold", 0.02);
    // the GS2 line is 0.14 m ahead of the front feet: the leg plane is exact
    // enough there, the lidar plane already leans over a step and hid it
    gs2_reference_ = declare_parameter("perception.gs2_reference", std::string("feet"));

    mounts_ = mountsFromParams(s);
    // where the GS2 line lies on a flat floor in the stand pose (for 'gap' reports)
    const double stand = getD("stance.stand_height", 0.15);
    stand_height_ = stand;
    if (mounts_.count("gs2")) {
      const auto & g = mounts_.at("gs2");
      const double t = rayToPlane(g.p, g.beam(), Plane{{0.0, 0.0, 1.0}, -stand});
      gs2_line_x_ = std::isfinite(t) ? g.p.x + t * g.beam().x : g.p.x + 0.15;
    }
    // perception/tof (every ToF reading) is for the simulation checks and
    // videos; publishing it costs more than the check itself (0.1 ms)
    debug_topics_ = getB("perception.debug_topics", false);
    map_ = std::make_unique<ElevationMap>(map_size, map_res);
    if (s.tof) {
      for (size_t k = 0; k < s.tof_names.size(); ++k) {
        const auto & n = s.tof_names[k];
        tof_.emplace(n, TofDetector(mounts_.at("tof_" + n), k < tof_thr.size() ? tof_thr[k] : 0.015,
          tof_confirm, k < tof_offsets.size() ? tof_offsets[k] : 0.0, tof_autocal ? 50 : 0));
        tof_index_[n] = static_cast<int>(k);
      }
    }

    // reaction to hazards
    guard_on_ = getB("perception.guard", true);
    GuardParams g;
    g.slow_vx = getD("perception.guard_slow_vx", g.slow_vx);
    g.near_vx = getD("perception.guard_near_vx", g.near_vx);
    g.stop_dist = getD("perception.guard_stop_dist", g.stop_dist);
    g.pass_dist = getD("perception.guard_pass_dist", g.pass_dist);
    g.trot_climb = getD("perception.guard_trot_climb", g.trot_climb);
    g.trot_descend = getD("perception.guard_trot_descend", g.trot_descend);
    g.climb_max = getD("perception.guard_climb_max", g.climb_max);
    g.descend_max = getD("perception.guard_descend_max", g.descend_max);
    g.crawl = getB("perception.guard_crawl", g.crawl);
    g.crawl_dist = getD("perception.guard_crawl_dist", g.crawl_dist);
    g.crawl_pass = getD("perception.guard_crawl_pass", g.crawl_pass);
    g.max_step = getD("perception.guard_max_step", g.max_step);
    g.step_margin = getD("perception.guard_step_margin", g.step_margin);
    g.confirm = getI("perception.guard_confirm", g.confirm);
    g.stop_confirm = getI("perception.guard_stop_confirm", g.stop_confirm);
    g.deep_stop = getB("perception.guard_deep_stop", g.deep_stop);
    const double fy = geometry_.hip_y + geometry_.hip_offset;
    g.feet = {{{geometry_.hip_x, fy}, {geometry_.hip_x, -fy}, {-geometry_.hip_x, fy}, {-geometry_.hip_x, -fy}}};
    guard_ = HazardGuard(g);
    guard_stop_dist_ = g.stop_dist;
    climb_max_ = g.climb_max;
    // going round what is too tall to cross
    avoid_on_ = getB("perception.guard_avoid", true);
    AvoidParams ap;
    ap.vy = getD("perception.guard_avoid_vy", ap.vy);
    ap.max_shift = getD("perception.guard_avoid_max_shift", ap.max_shift);
    ap.half_width = g.half_width;
    avoider_ = Avoider(ap);
    lift_min_ = getD("perception.guard_lift_min", 0.018);

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    const auto sensor = rclcpp::SensorDataQoS();
    subs_.push_back(create_subscription<std_msgs::msg::String>("state", latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) {state_ = m->data;}));
    subs_.push_back(create_subscription<sensor_msgs::msg::Imu>("imu/data", sensor,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);}));
    subs_.push_back(create_subscription<sensor_msgs::msg::JointState>("joint_states", 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {onJoints(*m);}));
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>("odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const auto t0 = now_ms();
        odom_ = *m;
        count(t0, 12);
      }));
    if (s.x_lidar) {
      for (const std::string name : {"lidar_left", "lidar_right"}) {
        subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(name + "/scan", sensor,
          [this, name](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(name, *m);}));
      }
    }
    if (s.gs2) {
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>("gs2/scan", sensor,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onGs2(*m);}));
    }
    for (const auto & kv : tof_) {
      const std::string n = kv.first;
      subs_.push_back(create_subscription<sensor_msgs::msg::Range>("tof/" + n, sensor,
        [this, n](sensor_msgs::msg::Range::ConstSharedPtr m) {onTof(n, *m);}));
    }
    pub_ground_lidar_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("perception/ground_lidar", 10);
    pub_ground_feet_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("perception/ground_feet", 10);
    pub_hazards_ = create_publisher<std_msgs::msg::String>("perception/hazards", 10);
    pub_tof_ = create_publisher<std_msgs::msg::Float32MultiArray>("perception/tof", 10);
    pub_map_ = create_publisher<std_msgs::msg::Float32MultiArray>("perception/map", 1);
    pub_stats_ = create_publisher<std_msgs::msg::Float64MultiArray>("perception/stats", 1);
    if (guard_on_) {
      pub_guard_ = create_publisher<std_msgs::msg::Float64MultiArray>("guard", 10);
      pub_guard_state_ = create_publisher<std_msgs::msg::String>("perception/guard", 10);
      pub_terrain_ = create_publisher<std_msgs::msg::Float32MultiArray>("terrain/profile", 10);
      timers_.push_back(create_wall_timer(std::chrono::milliseconds(100), [this]() {publishGuard();}));
    }
    timers_.push_back(create_wall_timer(std::chrono::seconds(1), [this]() {publishSlow();}));
    std::string names;
    for (const auto & kv : mounts_) {names += (names.empty() ? "" : ", ") + kv.first;}
    RCLCPP_INFO(get_logger(), "perception (C++): %s%s", names.c_str(), guard_on_ ? ", guard on" : "");
  }

private:
  // ---------------------------------------------------------- parameters
  // Declared with dynamic typing: a YAML "0" for a float, or [0, 0] for a
  // float list, must not stop the node.
  rclcpp::ParameterValue declareAny(const std::string & name, const rclcpp::ParameterValue & def)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.dynamic_typing = true;
    return declare_parameter(name, def, d);
  }
  double getD(const std::string & name, double def)
  {
    const auto v = declareAny(name, rclcpp::ParameterValue(def));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {return static_cast<double>(v.get<int64_t>());}
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {return v.get<double>();}
    RCLCPP_WARN(get_logger(), "%s: not a number, using %g", name.c_str(), def);
    return def;
  }
  int getI(const std::string & name, int def)
  {
    const auto v = declareAny(name, rclcpp::ParameterValue(def));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {return static_cast<int>(v.get<int64_t>());}
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {return static_cast<int>(v.get<double>());}
    return def;
  }
  bool getB(const std::string & name, bool def)
  {
    const auto v = declareAny(name, rclcpp::ParameterValue(def));
    return v.get_type() == rclcpp::ParameterType::PARAMETER_BOOL ? v.get<bool>() : def;
  }
  std::vector<double> getDV(const std::string & name, const std::vector<double> & def)
  {
    const auto v = declareAny(name, rclcpp::ParameterValue(def));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {return v.get<std::vector<double>>();}
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      const auto a = v.get<std::vector<int64_t>>();
      return std::vector<double>(a.begin(), a.end());
    }
    return def;
  }

  // ---------------------------------------------------------- helpers
  static double now_ms()
  {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void count(double t0, int slot = -1)
  {
    const double ms = now_ms() - t0;
    stats_[6] += 1;
    stats_[7] += ms;
    if (slot >= 0) {
      stats_[slot] += 1;
      stats_[slot + 1] += ms;
    }
  }
  bool upright() const {return state_ == "stand" || state_ == "walk";}

  std::optional<M3> R_at(double t) const
  {
    if (imu_hist_.empty()) {return std::nullopt;}
    auto it = std::lower_bound(imu_hist_.begin(), imu_hist_.end(), t,
      [](const std::pair<double, M3> & h, double v) {return h.first < v;});
    if (it == imu_hist_.begin()) {return it->second;}
    if (it == imu_hist_.end()) {return imu_hist_.back().second;}
    const auto prev = std::prev(it);
    return (it->first - t < t - prev->first) ? it->second : prev->second;
  }

  std::optional<Plane> ground(double t) const
  {
    const auto R = R_at(t);
    const auto feet = feet_.current(R);
    if (reference_ == "auto" && lidar_plane_ && std::abs(t - lidar_t_) < 0.3) {
      Plane p = *lidar_plane_;
      if (R && lidar_R_) {p.n = mul(transpose(mul(transpose(*lidar_R_), *R)), p.n);}
      if (!feet || std::abs(p.c - feet->c) < 0.015) {return p;}
    }
    return feet;
  }

  void publishV3(const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr & pub,
    const builtin_interfaces::msg::Time & stamp, const Plane & p)
  {
    geometry_msgs::msg::Vector3Stamped m;
    m.header.stamp = stamp;
    m.header.frame_id = "base_link";
    rollPitchOfNormal(p.n, m.vector.x, m.vector.y);
    m.vector.z = -p.c;
    pub->publish(m);
  }

  bool odomPose(V3 & pos, M3 & R) const
  {
    if (!odom_) {return false;}
    const auto & p = odom_->pose.pose;
    pos = {p.position.x, p.position.y, p.position.z};
    R = quatToRot(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    return true;
  }

  // ---------------------------------------------------------- inputs
  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const auto t0 = now_ms();
    const auto & q = m.orientation;
    imu_hist_.emplace_back(stampSec(m.header.stamp), quatToRot(q.x, q.y, q.z, q.w));
    while (imu_hist_.size() > 400) {imu_hist_.pop_front();}  // ~4 s at 100 Hz
    count(t0, 10);
  }

  void onJoints(const sensor_msgs::msg::JointState & m)
  {
    const auto t0 = now_ms();
    std::array<double, 12> q{};
    for (size_t i = 0; i < m.name.size() && i < m.position.size(); ++i) {
      for (int k = 0; k < 12; ++k) {
        if (m.name[i] == kJoints[k]) {q[k] = m.position[i];}
      }
    }
    const auto feet = feetBody(geometry_, q);
    legs_body_ = legsBody(geometry_, q);
    have_legs_body_ = true;
    const double t = stampSec(m.header.stamp);
    V3 pos;
    M3 R;
    // four feet on one plane (standing, trot's four-leg phases): they are on
    // the ground - the map learns the ground under the robot, which the
    // lidars never see (10 Hz is plenty)
    if (feet_.update(feet, R_at(t)) && t - last_feet_map_ > 0.1 && upright() && odomPose(pos, R)) {
      std::vector<V3> world;
      for (const auto & f : feet) {world.push_back(mul(R, f) + pos);}
      map_->insert(world);
      last_feet_map_ = t;
    }
    count(t0, 8);
  }

  /// p (body frame) is on one of the robot's legs
  bool onOwnLeg(const V3 & p) const {return have_legs_body_ && onLeg(legs_body_, p, kLegRadius);}

  void onScan(const std::string & name, const sensor_msgs::msg::LaserScan & msg)
  {
    const auto t0 = now_ms();
    auto pts = scanToBody(mounts_.at(name), msg.ranges, msg.angle_min, msg.angle_increment,
      msg.range_min, msg.range_max);
    // own legs and body: nothing inside the robot's footprint
    // and the legs where they are: a front leg swung forward and up over a
    // bar reaches past the footprint and would be a tall obstacle on the map
    // (its calf leans back to the knee: the whole leg, not a column over the foot)
    pts.erase(std::remove_if(pts.begin(), pts.end(),
      [this](const V3 & p) {return (std::abs(p.x) < 0.22 && std::abs(p.y) < 0.17) || onOwnLeg(p);}), pts.end());
    const double now = stampSec(msg.header.stamp);
    const auto R_now = R_at(now);
    scans_[name] = {now, pts, R_now};
    const auto other = scans_.find(name == "lidar_left" ? "lidar_right" : "lidar_left");
    if (other == scans_.end() || now - other->second.t > 0.15 || !upright()) {
      stats_[2] += static_cast<double>(pts.size());
      stats_[3] += now_ms() - t0;
      count(t0);
      return;
    }
    // the other scan is up to 0.1 s old: turn it by the body rotation since
    std::vector<V3> both = pts;
    const auto & o = other->second;
    std::optional<M3> dR;
    if (R_now && o.R) {dR = mul(transpose(*R_now), *o.R);}
    for (const auto & p : o.pts) {both.push_back(dR ? mul(*dR, p) : p);}
    const auto stamp = msg.header.stamp;
    // 1. floor plane seen by the lidars (near band, below the body)
    std::vector<V3> near;
    for (const auto & p : both) {
      if (std::hypot(p.x, p.y) < 1.2 && p.z < -0.05) {near.push_back(p);}
    }
    const auto fit = robustPlane(near);
    if (fit) {
      publishV3(pub_ground_lidar_, stamp, fit->plane);
      if (fit->rms < 0.008 && fit->inlier_fraction > 0.6) {
        lidar_plane_ = fit->plane;
        lidar_R_ = R_now;
        lidar_t_ = now;
      }
    }
    // 2. hazards in the foot corridors relative to the ground under the feet
    if (const auto feet = feet_.current(R_now)) {publishV3(pub_ground_feet_, stamp, *feet);}
    auto ref = ground(now);
    if (!ref && fit) {ref = fit->plane;}
    if (ref) {
      std::vector<Hazard> hz;
      for (const auto & h : lidarHazards(both, *ref, thr_)) {
        Hazard z;
        z.source = "lidar";
        z.corridor = h.corridor;
        z.kind = h.kind;
        z.x = h.x;
        z.h = h.h;
        z.jump = h.jump;
        hz.push_back(z);
      }
      publishHazards(hz);
    }
    // 3. elevation map in the odom frame
    V3 pos;
    M3 R;
    if (odomPose(pos, R)) {
      std::vector<V3> world;
      world.reserve(both.size());
      for (const auto & p : both) {world.push_back(mul(R, p) + pos);}
      map_->recenter(pos.x, pos.y);
      map_->insert(world);
    }
    stats_[1] += 1;
    stats_[2] += static_cast<double>(pts.size());
    stats_[3] += now_ms() - t0;
    count(t0);
  }

  void onGs2(const sensor_msgs::msg::LaserScan & msg)
  {
    const auto t0 = now_ms();
    const auto & m = mounts_.at("gs2");
    const auto pts = scanToBody(m, msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max);
    if (upright()) {
      const double t = stampSec(msg.header.stamp);
      const auto ref = gs2_reference_ == "feet" ? feet_.current(R_at(t)) : ground(t);
      // a missing floor in the middle of the line counts only if the floor
      // should be well within range: nose-up pitch alone moves it past 0.3 m
      const double exp = ref ? rayToPlane(m.p, m.beam(), *ref) : kInf;
      const bool expect = exp < 0.85 * msg.range_max;
      std::map<std::string, int> seen;
      std::vector<Hazard> out;
      for (const auto & h : gs2Hazards(pts, ref, expect, gs2_thr_, gs2_plane_thr_)) {
        const std::string key = h.corridor + "/" + h.kind + "/" + h.how;
        const auto it = gs2_seen_.find(key);
        seen[key] = (it == gs2_seen_.end() ? 0 : it->second) + 1;
        if (seen[key] >= gs2_confirm_) {
          Hazard z;
          z.source = "gs2";
          z.corridor = h.corridor;
          z.kind = h.kind;
          z.how = h.how;
          z.x = h.x;
          z.y = h.y;
          z.h = h.h;
          out.push_back(z);
        }
      }
      gs2_seen_ = seen;
      publishHazards(out);
      V3 pos;
      M3 R;
      if (!pts.empty() && odomPose(pos, R)) {
        std::vector<V3> world;
        for (const auto & p : pts) {world.push_back(mul(R, p) + pos);}
        map_->insert(world);
      }
    }
    stats_[14] += 1;
    stats_[15] += static_cast<double>(pts.size());
    stats_[16] += now_ms() - t0;
    count(t0);
  }

  void onTof(const std::string & name, const sensor_msgs::msg::Range & msg)
  {
    const auto t0 = now_ms();
    const auto plane = ground(stampSec(msg.header.stamp));
    auto & det = tof_.at(name);
    if (plane && upright()) {
      if (!det.calibrated() && state_ == "stand") {
        det.calibrate(msg.range, *plane);
      } else if (det.calibrated()) {
        const auto r = det.check(msg.range, *plane);
        if (debug_topics_) {
          std_msgs::msg::Float32MultiArray a;
          a.data = {static_cast<float>(tof_index_[name]), fin(msg.range), fin(r.expected), fin(r.residual)};
          pub_tof_->publish(a);
        }
        if (!r.verdict.empty()) {
          Hazard z;
          z.source = "tof_" + name;
          z.kind = r.verdict;
          z.range = msg.range;
          z.expected = r.expected;
          z.residual = r.residual;
          publishHazards({z});
        }
      }
    }
    stats_[4] += 1;
    stats_[5] += now_ms() - t0;
    count(t0);
  }
  static float fin(double v) {return std::isfinite(v) ? static_cast<float>(v) : std::numeric_limits<float>::quiet_NaN();}

  // ---------------------------------------------------------- outputs
  void publishHazards(std::vector<Hazard> items)
  {
    if (items.empty()) {return;}
    const double t = now().seconds();
    std::ostringstream js;
    js << "[";
    for (size_t k = 0; k < items.size(); ++k) {
      auto & h = items[k];
      if (guard_on_) {guardAdd(t, h);}
      js << (k ? ", " : "") << "{\"source\": " << str(h.source) << ", \"kind\": " << str(h.kind);
      if (h.source == "lidar") {
        js << ", \"corridor\": " << str(h.corridor) << ", \"x\": " << num(h.x, 3) << ", \"h\": " << num(h.h, 3)
           << ", \"jump\": " << num(h.jump, 3);
      } else if (h.source == "gs2") {
        js << ", \"corridor\": " << str(h.corridor) << ", \"how\": " << str(h.how) << ", \"x\": " << num(h.x)
           << ", \"y\": " << num(h.y) << ", \"h\": " << num(h.h);
      } else {
        js << ", \"range\": " << num(h.range) << ", \"expected\": " << num(h.expected)
           << ", \"residual\": " << num(h.residual);
      }
      if (!h.guard.empty()) {js << ", \"guard\": " << str(h.guard);}
      js << ", \"t\": " << num(t, 3) << "}";
    }
    js << "]";
    std_msgs::msg::String m;
    m.data = js.str();
    pub_hazards_->publish(m);
  }

  /// Hazard report -> point in the odom frame with a stop / step verdict.
  void guardAdd(double t, Hazard & h)
  {
    V3 pos;
    M3 R;
    if (!odomPose(pos, R)) {return;}
    V3 b;
    double edge = kNaN, lift = 0.0;
    bool deep = false;
    if (h.source == "lidar") {
      b = {h.x, kCorridorY.at(h.corridor), 0.0};
      edge = h.jump;
      // a drop needs no higher swing; below lift_min a jump may be noise
      lift = h.kind == "up" && h.jump > lift_min_ ? h.jump : 0.0;
    } else if (h.source == "gs2") {
      deep = h.how == "gap";
      b = {std::isfinite(h.x) ? h.x : gs2_line_x_, std::isfinite(h.y) ? h.y : 0.0, 0.0};
      // 'line': an object shorter than the line, its height is an edge;
      // 'plane': a step or just a ramp - the line alone cannot tell
      edge = h.how == "line" ? h.h : kNaN;
      lift = h.kind == "up" && std::isfinite(edge) ? edge : 0.0;
    } else {  // ToF: the spot on the floor; one ray cannot tell an obstacle from a ramp
      const auto & m = mounts_.at(h.source);
      if (m.beam().x <= 0) {return;}  // rear sensor: backing off is not guarded
      const double rng = std::isfinite(h.range) ? h.range : h.expected;
      if (!std::isfinite(rng)) {return;}
      b = m.p + m.beam() * rng;
      deep = !std::isfinite(h.range) && h.kind == "down";
    }
    const V3 w = mul(R, b) + pos;
    h.guard = guard_.verdict(h.kind, edge, deep);
    guard_.add(t, w.x, w.y, h.guard, lift);
  }

  void publishGuard()
  {
    V3 pos;
    M3 R;
    if (!odomPose(pos, R)) {return;}
    const auto & q = odom_->pose.pose.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    auto c = guard_.command(now().seconds(), pos.x, pos.y, yaw);
    if (!upright()) {c = HazardGuard::Command();}
    // too tall to cross: go round it if it is narrow (elevation map), from
    // behind the rear feet to 1 m ahead
    double vy = 0.0;
    if (avoid_on_ && upright()) {
      Obstacle o = tallObstacle(*map_, pos.x, pos.y, yaw, pos.z - stand_height_, climb_max_ + kMapTallMargin, -0.35, 1.0, 0.8);
      // (the map's highest points carry the lidar noise: 2 cm more than the
      // edge rule, or a 60 mm bar counts as too tall; the map is the backstop
      // for gross misreads - a 150 mm block's face read as 60 mm from afar)
      // ... or its top above what the crawl climbs, by the cells' mean
      // heights: an 80 mm wall the edge rule read under 70 mm all the way (CI)
      // was crawled into; the means read its top 78-84 mm, a 60 mm bar's
      // under 65
      const Obstacle top = tallObstacle(*map_, pos.x, pos.y, yaw, pos.z - stand_height_, climb_max_ + kMapMeanMargin, -0.35, 1.0, 0.8, false);
      if (top.found && !o.found) {
        o = top;
      } else if (top.found) {
        o.lat_min = std::min(o.lat_min, top.lat_min);
        o.lat_max = std::max(o.lat_max, top.lat_max);
        o.d_min = std::min(o.d_min, top.d_min);
      }
      // The map sees a tall thing in the path: no crawl at it (a lidar jump
      // on its face, seen from afar, may read under climb_max), and stop at
      // stop_dist like any too tall edge
      // (going round it: the path narrows by half the side margin, or the
      // heading wandering 3 degrees brings its corner back into the path)
      // Only ahead of the front feet: what they already stand over or step
      // across (the crawl over a bar) is the crawl's, never a stop or a
      // switch to the trot under it
      const double hw = avoider_.pathHalfWidth(), feet_x = geometry_.hip_x + kFootReach;
      const bool tall_now = o.found && o.d_min > feet_x && o.lat_max > -hw && o.lat_min < hw;
      // and for a while: one noisy peak on a 60 mm bar read as tall for a
      // tick, the robot stood on four feet, switched to the trot and trotted
      // onto the bar when the crawl came back
      const double t_now = now().seconds();
      if (!tall_now) {
        tall_since_ = kNaN;
      } else if (!std::isfinite(tall_since_)) {
        tall_since_ = t_now;
      }
      const bool tall_ahead = tall_now && t_now - tall_since_ >= kTallConfirm;
      if (tall_ahead && o.d_min < 0.6) {c.gait = 0;}
      if (tall_ahead && o.d_min < guard_stop_dist_) {
        c.max_vx = 0.0;
        c.state = "stop";
        c.d = o.d_min;
      }
      // What to go round: something whose top, by the cells' mean heights,
      // is well above what the crawl climbs. What rises only a little above
      // climb_max (an 80 mm wall: means 78-84 mm, its highest points noisy)
      // is mapped too unreliably to size - a fragment of it looked narrow,
      // its end unmapped looked passed, and the robot walked into it; the
      // guard just stops at that. How wide: the means see a block's top
      // only in part from 0.35 m (few points on it that far), so the width
      // is that of the run of highest points over climb_max + 20 mm that
      // overlaps it - its face, seen whole; a noisy cell off to the side is
      // no part of it.
      Obstacle wide = tallObstacle(*map_, pos.x, pos.y, yaw, pos.z - stand_height_, climb_max_ + kAvoidMargin, -0.35, 1.0, 0.8, false);
      if (wide.found && o.found && o.lat_min <= wide.lat_max && o.lat_max >= wide.lat_min) {
        const Obstacle top = wide;
        wide = o;
        wide.lat_min = std::min(wide.lat_min, top.lat_min);
        wide.lat_max = std::max(wide.lat_max, top.lat_max);
        wide.d_min = std::min(wide.d_min, top.d_min);
      }
      const std::string before = avoider_.state();
      vy = avoider_.update(c.state == "stop" && wide.found && wide.d_min > feet_x, wide, pos.x, pos.y, yaw);
      if (avoider_.state() != before) {
        RCLCPP_INFO(get_logger(), "avoid: %s -> %s (offset %.2f m, needs %.2f m; obstacle %.2f..%.2f m%s%s, %.2f m ahead)",
          before.c_str(), avoider_.state().c_str(), avoider_.offset(), avoider_.needed(), wide.lat_min, wide.lat_max,
          wide.open_right ? ", open right" : "", wide.open_left ? ", open left" : "", wide.d_min);
      }
      if (avoider_.state() != "idle") {
        // going round: in the trot (the crawl sidesteps at ~1 cm/s and turns
        // away with it), and no forward step while it is still in the way
        c.gait = 0;
        if (avoider_.state() == "aside") {c.max_vx = 0.0;}
        if (c.state != "stop") {c.state = "avoid";}
      }
    }
    std_msgs::msg::Float64MultiArray g;
    g.data = {c.max_vx, c.step[0], c.step[1], c.step[2], c.step[3], static_cast<double>(c.gait), vy};
    pub_guard_->publish(g);
    publishTerrain(pos, yaw);
    if (c.state != guard_state_) {
      if (std::isfinite(c.d)) {
        RCLCPP_INFO(get_logger(), "guard: %s (hazard %.2f m ahead)", c.state.c_str(), c.d);
      } else {
        RCLCPP_INFO(get_logger(), "guard: %s", c.state.c_str());
      }
      guard_state_ = c.state;
    }
    std_msgs::msg::String m;
    m.data = "{\"state\": " + str(c.state) + ", \"max_vx\": " + num(c.max_vx) + ", \"step\": [" +
      num(c.step[0]) + ", " + num(c.step[1]) + ", " + num(c.step[2]) + ", " + num(c.step[3]) +
      "], \"d\": " + num(c.d) + ", \"gait\": " + std::to_string(c.gait) + ", \"vy\": " + num(vy) +
      ", \"avoid\": " + str(avoider_.state()) + "}";
    pub_guard_state_->publish(m);
  }

  /// Ground heights along both foot lines, body frame x from -0.3 to +0.7 m
  /// (odom z): what the crawl steps on and over.
  void publishTerrain(const V3 & pos, double yaw)
  {
    const double x0 = -0.3, dx = 0.02;
    const int n = 51;
    const double fy = geometry_.hip_y + geometry_.hip_offset, cs = std::cos(yaw), sn = std::sin(yaw);
    std_msgs::msg::Float32MultiArray m;
    m.data = {static_cast<float>(x0), static_cast<float>(dx), static_cast<float>(n)};
    for (double side : {1.0, -1.0}) {
      for (int i = 0; i < n; ++i) {
        const double xb = x0 + i * dx, yb = side * fy;
        m.data.push_back(fin(map_->heightAt(pos.x + cs * xb - sn * yb, pos.y + sn * xb + cs * yb)));
      }
    }
    pub_terrain_->publish(m);
  }

  void publishSlow()
  {
    stats_[0] = processCpu();
    std_msgs::msg::Float64MultiArray s;
    s.data.assign(stats_.begin(), stats_.end());
    pub_stats_->publish(s);
    std_msgs::msg::Float32MultiArray m;
    m.data = {static_cast<float>(map_->originX()), static_cast<float>(map_->originY()),
      static_cast<float>(map_->resolution()), static_cast<float>(map_->size())};
    const auto mean = map_->mean();
    m.data.insert(m.data.end(), mean.begin(), mean.end());
    pub_map_->publish(m);
  }

  struct Scan
  {
    double t{0.0};
    std::vector<V3> pts;
    std::optional<M3> R;
  };

  Geometry geometry_;
  std::map<std::string, SensorMount> mounts_;
  double thr_{0.015}, gs2_thr_{0.012}, gs2_plane_thr_{0.02}, lift_min_{0.018}, gs2_line_x_{0.28};
  bool debug_topics_{false};
  double stand_height_{0.15}, climb_max_{0.07}, last_feet_map_{0.0};
  bool avoid_on_{true};
  Avoider avoider_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_terrain_;
  int gs2_confirm_{8};
  std::string reference_, gs2_reference_, state_, guard_state_;
  std::map<std::string, TofDetector> tof_;
  std::map<std::string, int> tof_index_;
  std::map<std::string, int> gs2_seen_;
  FeetPlane feet_;
  std::array<LegChain, 4> legs_body_{};
  bool have_legs_body_{false};
  // lidar points this close to a thigh or calf are the leg (links, the foot,
  // and a swinging leg moving between the joint states and the scan)
  static constexpr double kLegRadius = 0.04;
  static constexpr double kFootReach = 0.05;  // a front foot swings this far ahead of its hip
  static constexpr double kTallConfirm = 0.6;  // [s] the map's tall obstacle holds this long before it acts
  double tall_since_{kNaN};
  std::deque<std::pair<double, M3>> imu_hist_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::map<std::string, Scan> scans_;
  std::optional<Plane> lidar_plane_;
  std::optional<M3> lidar_R_;
  double lidar_t_{0.0};
  std::unique_ptr<ElevationMap> map_;
  std::array<double, 17> stats_{};
  bool guard_on_{true};
  HazardGuard guard_;
  double guard_stop_dist_{0.30};
  static constexpr double kMapTallMargin = 0.02;
  static constexpr double kMapMeanMargin = 0.005;  // over climb_max, by the cells' mean heights
  static constexpr double kAvoidMargin = 0.03;  // ... to size an obstacle to go round

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_ground_lidar_, pub_ground_feet_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_hazards_, pub_guard_state_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_tof_, pub_map_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_stats_, pub_guard_;
};

}  // namespace dog_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_perception::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
