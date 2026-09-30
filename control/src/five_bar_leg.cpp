#include "control/five_bar_leg.hpp"

#include <cmath>

namespace control {
namespace {

constexpr float kPi = 3.14159265358979f;
// sin(phi3 - phi2) 低于此值视为 BC/CD 共线（奇异位形）
constexpr float kSingularSin = 1e-3f;
constexpr float kMinLegLength = 1e-4f;

// 解 a·cos(x) + b·sin(x) = c，sign 选择两个解支之一（+1 / -1）
bool SolveTrig(float a, float b, float c, float sign, float& x) {
  const float disc = a * a + b * b - c * c;
  if (disc < 0.0f) {
    return false;
  }
  x = 2.0f * std::atan2(b + sign * std::sqrt(disc), a + c);
  return true;
}

}  // namespace

bool FiveBarLeg::Update(float phi1, float phi4) {
  const float xb = geo_.l1 * std::cos(phi1);
  const float yb = geo_.l1 * std::sin(phi1);
  const float xd = geo_.l5 + geo_.l4 * std::cos(phi4);
  const float yd = geo_.l4 * std::sin(phi4);

  const float dx = xd - xb;
  const float dy = yd - yb;
  const float a0 = 2.0f * geo_.l2 * dx;
  const float b0 = 2.0f * geo_.l2 * dy;
  const float c0 = geo_.l2 * geo_.l2 + dx * dx + dy * dy - geo_.l3 * geo_.l3;

  float phi2 = 0.0f;
  if (!SolveTrig(a0, b0, c0, +1.0f, phi2)) {
    valid_ = false;
    return false;
  }

  const float xc = xb + geo_.l2 * std::cos(phi2);
  const float yc = yb + geo_.l2 * std::sin(phi2);
  const float rx = xc - geo_.l5 * 0.5f;
  const float l0 = std::sqrt(rx * rx + yc * yc);
  const float phi0 = std::atan2(yc, rx);
  const float phi3 = kPi + std::atan2(yc - yd, xc - xd);

  const float s32 = std::sin(phi3 - phi2);
  if (std::fabs(s32) < kSingularSin || l0 < kMinLegLength) {
    valid_ = false;
    return false;
  }

  const float k1 = geo_.l1 * std::sin(phi1 - phi2) / s32;
  const float k4 = geo_.l4 * std::sin(phi3 - phi4) / s32;

  kin_.l0 = l0;
  kin_.phi0 = phi0;
  kin_.phi2 = phi2;
  kin_.phi3 = phi3;
  kin_.xc = xc;
  kin_.yc = yc;
  kin_.jacobian = {{{k1 * std::sin(phi0 - phi3), k4 * std::sin(phi0 - phi2)},
                    {k1 * std::cos(phi0 - phi3) / l0,
                     k4 * std::cos(phi0 - phi2) / l0}}};
  valid_ = true;
  return true;
}

LegRate FiveBarLeg::Velocity(float d_phi1, float d_phi4) const {
  const Mat2& j = kin_.jacobian;
  return {j[0][0] * d_phi1 + j[0][1] * d_phi4,
          j[1][0] * d_phi1 + j[1][1] * d_phi4};
}

JointTorque FiveBarLeg::TorqueFromForce(const LegForce& force) const {
  const Mat2& j = kin_.jacobian;
  return {j[0][0] * force.f + j[1][0] * force.tp,
          j[0][1] * force.f + j[1][1] * force.tp};
}

bool FiveBarLeg::ForceFromTorque(const JointTorque& torque,
                                 LegForce& out) const {
  // 解 J^T · [F, Tp]^T = [T1, T4]^T
  const Mat2& j = kin_.jacobian;
  const float det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
  if (std::fabs(det) < 1e-9f) {
    return false;
  }
  out.f = (j[1][1] * torque.t1 - j[1][0] * torque.t4) / det;
  out.tp = (-j[0][1] * torque.t1 + j[0][0] * torque.t4) / det;
  return true;
}

bool FiveBarLeg::Inverse(float l0, float phi0, JointAngle& out) const {
  const float xc = geo_.l5 * 0.5f + l0 * std::cos(phi0);
  const float yc = l0 * std::sin(phi0);

  // B 点：|AB| = l1，|BC| = l2
  const float a1 = 2.0f * geo_.l1 * xc;
  const float b1 = 2.0f * geo_.l1 * yc;
  const float c1 = geo_.l1 * geo_.l1 + xc * xc + yc * yc - geo_.l2 * geo_.l2;

  // D 点：|ED| = l4，|DC| = l3，取与 B 点相反的解支
  const float ex = xc - geo_.l5;
  const float a4 = 2.0f * geo_.l4 * ex;
  const float b4 = 2.0f * geo_.l4 * yc;
  const float c4 = geo_.l4 * geo_.l4 + ex * ex + yc * yc - geo_.l3 * geo_.l3;

  JointAngle result{};
  if (!SolveTrig(a1, b1, c1, +1.0f, result.phi1) ||
      !SolveTrig(a4, b4, c4, -1.0f, result.phi4)) {
    return false;
  }
  out = result;
  return true;
}

}  // namespace control
