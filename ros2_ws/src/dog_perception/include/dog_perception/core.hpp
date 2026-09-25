// Terrain perception without ROS: sensor geometry, ground planes, elevation
// map, the detectors (X lidars, GS2 line lidar, VL53L1X) and the hazard guard.
// C++ port of dog_perception/core.py (kept as the reference for the offline
// tools); both follow the same conventions and tests.
//
// Frames: body = base_link (x forward, y left, z up, origin at the body centre).
// A plane (n, c): points p on it satisfy n . p = c, n is the unit normal
// pointing up, so c = -(height of the origin above it).
#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace dog_perception
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

struct V3
{
  double x{0.0}, y{0.0}, z{0.0};
  V3 operator+(const V3 & o) const {return {x + o.x, y + o.y, z + o.z};}
  V3 operator-(const V3 & o) const {return {x - o.x, y - o.y, z - o.z};}
  V3 operator*(double k) const {return {x * k, y * k, z * k};}
  double dot(const V3 & o) const {return x * o.x + y * o.y + z * o.z;}
  double norm() const {return std::sqrt(dot(*this));}
};

using M3 = std::array<std::array<double, 3>, 3>;

M3 rotRpy(double roll, double pitch, double yaw);  // URDF: Rz(yaw) Ry(pitch) Rx(roll)
M3 quatToRot(double x, double y, double z, double w);
M3 identity();
M3 transpose(const M3 & a);
M3 mul(const M3 & a, const M3 & b);
V3 mul(const M3 & a, const V3 & v);
/// Body roll / pitch [rad] relative to a plane with normal n (body frame);
/// REP-103 signs (pitch + = nose down).
void rollPitchOfNormal(const V3 & n, double & roll, double & pitch);
double median(std::vector<double> v);

struct Plane
{
  V3 n{0.0, 0.0, 1.0};
  double c{0.0};
  double residual(const V3 & p) const {return n.dot(p) - c;}
};

// ------------------------------------------------------------------ sensors
struct SensorMount
{
  std::string name;
  V3 p;
  M3 R{};
  V3 beam() const {return {R[0][0], R[1][0], R[2][0]};}  // sensor x axis
};

/// The 'sensors' section of robot.yaml.
struct SensorParams
{
  bool x_lidar{true};
  double x_lidar_x{0.10}, x_lidar_y{0.04}, x_lidar_z{0.062};
  double x_lidar_tilt_deg{30.0}, x_lidar_yaw_deg{40.0};
  bool tof{true};
  std::vector<std::string> tof_names{"fl", "fr", "fc", "rc"};
  std::vector<double> tof_x{0.115, 0.115, 0.115, -0.115};
  std::vector<double> tof_y{0.045, -0.045, 0.0, 0.0};
  std::vector<double> tof_z{0.0, 0.0, 0.012, 0.0};
  std::vector<double> tof_pitch_deg{40.0, 40.0, 20.0, 40.0};
  std::vector<double> tof_yaw_deg{23.0, -23.0, 0.0, 180.0};
  bool gs2{false};
  double gs2_x{0.115}, gs2_y{0.0}, gs2_z{0.0}, gs2_pitch_deg{40.0};
};

/// Same rules as dog_description.urdf.sensor_frames. Throws on inconsistent
/// ToF lists.
std::map<std::string, SensorMount> mountsFromParams(const SensorParams & s);

/// LaserScan ranges -> points in the body frame (invalid ranges dropped).
std::vector<V3> scanToBody(const SensorMount & m, const std::vector<float> & ranges,
  double angle_min, double angle_inc, double rmin = 0.03, double rmax = 12.0);

/// Distance along the unit ray p + t u to the plane, inf if it never hits.
double rayToPlane(const V3 & p, const V3 & u, const Plane & plane);

// ------------------------------------------------------------------ planes
/// Least-squares plane (normal up), nullopt with fewer than 3 points.
std::optional<Plane> fitPlane(const std::vector<V3> & pts);

struct RobustFit
{
  Plane plane;
  double rms{0.0};
  double inlier_fraction{0.0};
};

/// Plane fit with outlier rejection (residual > max(keep, 2.5 sigma)).
std::optional<RobustFit> robustPlane(const std::vector<V3> & pts, int iterations = 3, double keep = 0.02);

// ------------------------------------------------------------------ legs
struct Geometry
{
  double hip_offset{0.055}, thigh{0.105}, calf{0.105}, hip_x{0.09}, hip_y{0.06};
};

/// Foot contact points in the body frame from 12 joint angles
/// (LF, RF, LR, RR x hip, thigh, calf), same kinematics as dog_control;
/// calf runs to the contact point (robot.yaml, URDF): nothing to subtract.
std::array<V3, 4> feetBody(const Geometry & g, const std::array<double, 12> & q, double foot_radius = 0.0);

/// Ground under the robot from the leg kinematics: taken when all four feet
/// lie on one plane (four-leg support), carried with the IMU in between.
class FeetPlane
{
public:
  explicit FeetPlane(double tolerance = 0.0015) : tol_(tolerance) {}
  bool update(const std::array<V3, 4> & feet, const std::optional<M3> & R_imu);
  std::optional<Plane> current(const std::optional<M3> & R_imu) const;

private:
  double tol_;
  std::optional<Plane> plane_;
  std::optional<M3> R_at_;
};

// ------------------------------------------------------------------ elevation map
/// Rolling grid in the odom frame: mean height per cell.
class ElevationMap
{
public:
  ElevationMap(double size = 3.0, double res = 0.02);
  void recenter(double x, double y);
  void insert(const std::vector<V3> & world_pts);
  std::vector<float> mean() const;  // n*n, row = x index, NaN = unknown
  double originX() const {return ox_;}
  double originY() const {return oy_;}
  double resolution() const {return res_;}
  int size() const {return n_;}

private:
  double res_;
  int n_;
  double ox_{0.0}, oy_{0.0};
  std::vector<double> sum_;
  std::vector<int> cnt_;
};

// ------------------------------------------------------------------ detectors
struct Corridor
{
  const char * name;
  double y0, y1;
};
extern const std::array<Corridor, 3> kCorridors;  // left, centre, right foot corridors

struct LidarHazard
{
  std::string corridor, kind;  // kind: up | down
  double x{0.0};     // nearest point beyond the threshold [m, body]
  double h{0.0};     // median residual over the corridor [m]
  double jump{0.0};  // largest rise (up, > 0) / drop (down, < 0) over 3-6 cm along x
};

std::vector<LidarHazard> lidarHazards(const std::vector<V3> & pts, const Plane & plane, double thr = 0.015,
  double x_min = 0.25, double x_max = 1.0, int min_points = 3, double bin = 0.03, int bin_points = 3);

struct Gs2Hazard
{
  std::string corridor, kind, how;  // how: line | plane | gap
  double x{kNaN}, y{0.0}, h{kNaN};
};

std::vector<Gs2Hazard> gs2Hazards(const std::vector<V3> & pts, const std::optional<Plane> & plane,
  bool expected_centre, double thr_local = 0.012, double thr_abs = 0.015, int min_points = 3);

/// VL53L1X: measured range vs. the expected distance to the ground plane.
class TofDetector
{
public:
  TofDetector(const SensorMount & m, double thr, int confirm, double offset, int baseline_n = 0,
    double max_expected = 1.0);
  struct Result
  {
    std::string verdict;  // "", up, down
    double expected{kNaN};
    double residual{kNaN};
  };
  Result check(double measured, const Plane & plane);
  bool calibrate(double measured, const Plane & plane);
  bool calibrated() const {return static_cast<int>(offsets_.size()) >= baseline_n_;}
  const SensorMount & mount() const {return m_;}

private:
  SensorMount m_;
  double thr_, max_expected_;
  int confirm_, baseline_n_;
  double offset_;
  std::vector<double> offsets_;
  std::string last_;
  int run_{0};
};

// ------------------------------------------------------------------ guard
struct GuardParams
{
  double slow_vx{0.08}, near_vx{0.05}, stop_dist{0.30}, pass_dist{0.20}, half_width{0.20};
  double climb_max{0.04}, descend_max{0.06}, step_margin{0.01}, max_step{0.03}, memory{15.0};
  std::array<std::array<double, 2>, 4> feet{{{0.09, 0.115}, {0.09, -0.115}, {-0.09, 0.115}, {-0.09, -0.115}}};
  double leg_width{0.08}, leg_ahead{0.15}, leg_behind{0.06};
  int confirm{2}, stop_confirm{3};
  bool deep_stop{false};
};

/// Hazard reports -> forward speed limit and swing height per leg. Reports
/// are kept on a 5 cm grid in the odom frame; see HazardGuard in core.py for
/// the rules and why.
class HazardGuard
{
public:
  static constexpr double kCell = 0.05;
  explicit HazardGuard(const GuardParams & p = GuardParams()) : p_(p) {}
  /// "stop" or "step" for a hazard of this kind and edge height (NaN = unknown).
  std::string verdict(const std::string & kind, double edge, bool deep = false) const;
  void add(double t, double x, double y, const std::string & verdict, double lift);
  struct Command
  {
    double max_vx{kInf};
    std::array<double, 4> step{{kNaN, kNaN, kNaN, kNaN}};
    std::string state{"clear"};
    double d{kNaN};
  };
  Command command(double t, double x, double y, double yaw);
  size_t size() const {return cells_.size();}

private:
  struct Cell {double sx{0}, sy{0}; int n{0}, n_stop{0}; double t_last{0}, lift{0};};
  GuardParams p_;
  std::map<std::pair<long, long>, Cell> cells_;
};

}  // namespace dog_perception
