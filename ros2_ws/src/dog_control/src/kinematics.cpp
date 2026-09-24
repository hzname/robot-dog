#include "dog_control/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace dog_control
{

namespace
{
constexpr double kEps = 1e-6;

double wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a <= -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

Vec3 forwardKinematics(const LegGeometry & g, int side, const JointAngles & q)
{
  // Sagittal chain in the (already abducted) leg plane.
  const double xs = -g.thigh * std::sin(q[1]) - g.calf * std::sin(q[1] + q[2]);
  const double zs = -g.thigh * std::cos(q[1]) - g.calf * std::cos(q[1] + q[2]);
  const double y0 = side * g.hip;
  // Abduction: rotate (y0, zs) about the x axis by q[0].
  const double c = std::cos(q[0]);
  const double s = std::sin(q[0]);
  return {xs, y0 * c - zs * s, y0 * s + zs * c};
}

IkResult inverseKinematics(const LegGeometry & g, int side, const Vec3 & foot, int knee_direction)
{
  IkResult out;
  const double y0 = side * g.hip;

  // 1) Abduction: in the y-z plane the foot sits at distance D from the axis;
  //    the leg plane is offset by |y0|, leaving h as the in-plane leg height.
  const double d2 = foot.y * foot.y + foot.z * foot.z;
  const double h_min = std::abs(g.thigh - g.calf) + 0.01;
  double h2 = d2 - y0 * y0;
  if (h2 < h_min * h_min) {
    h2 = h_min * h_min;
    out.reachable = false;
  }
  double h = std::sqrt(h2);
  out.q[0] = wrapAngle(std::atan2(foot.z, foot.y) - std::atan2(-h, y0));

  // 2) Two-link planar IK for thigh + calf, target (x, -h).
  double x = foot.x;
  double r = std::hypot(x, h);
  const double r_max = g.thigh + g.calf - kEps;
  const double r_min = std::abs(g.thigh - g.calf) + kEps;
  if (r > r_max || r < r_min) {
    const double r_new = std::clamp(r, r_min, r_max);
    const double k = r > kEps ? r_new / r : 1.0;
    x *= k;
    h *= k;
    r = r_new;
    out.reachable = false;
  }
  const double cos_knee =
    (r * r - g.thigh * g.thigh - g.calf * g.calf) / (2.0 * g.thigh * g.calf);
  const double knee = (knee_direction >= 0 ? 1.0 : -1.0) * std::acos(std::clamp(cos_knee, -1.0, 1.0));
  out.q[2] = knee;
  out.q[1] = std::atan2(-g.calf * std::sin(knee), g.thigh + g.calf * std::cos(knee)) -
    std::atan2(x, h);
  return out;
}

}  // namespace dog_control
