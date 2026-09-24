// Leg kinematics for a 3-DOF quadruped leg (hip abduction + thigh pitch + knee pitch).
//
// Frames (REP-103): x forward, y left, z up. Each leg frame sits on the hip
// abduction axis and is aligned with the body frame.
//
// Joint convention (right-hand rule, zero = leg hanging straight down):
//   q[0] hip   — rotation about +x (positive rolls the foot towards +y / left)
//   q[1] thigh — rotation about +y (positive swings the foot backwards)
//   q[2] knee  — rotation about +y, relative to the thigh
//
// The hip link is an offset of `hip` metres along ±y (side = +1 for left legs,
// -1 for right legs) between the abduction axis and the thigh pitch axis.
#pragma once

#include <array>

namespace dog_control
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Vec3 operator+(const Vec3 & a, const Vec3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
inline Vec3 operator-(const Vec3 & a, const Vec3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline Vec3 operator*(const Vec3 & a, double k) {return {a.x * k, a.y * k, a.z * k};}

using JointAngles = std::array<double, 3>;  // hip, thigh, knee [rad]

enum LegIndex : int { LF = 0, RF = 1, LR = 2, RR = 3 };
constexpr int kNumLegs = 4;
constexpr int kNumJoints = 12;

/// +1 for left legs, -1 for right legs.
constexpr int legSide(int leg) {return (leg == LF || leg == LR) ? 1 : -1;}
/// +1 for front legs, -1 for rear legs.
constexpr int legFront(int leg) {return (leg == LF || leg == RF) ? 1 : -1;}

struct LegGeometry
{
  double hip{0.055};    // abduction axis -> thigh axis, lateral offset [m]
  double thigh{0.105};  // thigh axis -> knee axis [m]
  double calf{0.105};   // knee axis -> foot contact [m]
};

struct IkResult
{
  JointAngles q{0.0, 0.0, 0.0};
  bool reachable{true};  // false when the target was clamped into the workspace
};

/// Foot position in the leg frame for the given joint angles.
Vec3 forwardKinematics(const LegGeometry & g, int side, const JointAngles & q);

/// Joint angles that place the foot at `foot` (leg frame).
/// knee_direction = -1 bends the knee backwards (knee behind the hip-foot line),
/// +1 bends it forwards. Unreachable targets are clamped to the workspace.
IkResult inverseKinematics(const LegGeometry & g, int side, const Vec3 & foot, int knee_direction = -1);

}  // namespace dog_control
