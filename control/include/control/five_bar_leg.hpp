#pragma once

// 五连杆腿运动学 / VMC 映射。纯算法，无硬件依赖，可在主机上单测。
//
// 坐标系（与原 F4 vmc.c 保持一致）：
//   A = (0, 0)       后髋关节，电机角 phi1
//   E = (l5, 0)      前髋关节，电机角 phi4（l5 = 0 表示两髋共轴）
//   B = A + l1·(cos phi1, sin phi1)
//   D = E + l4·(cos phi4, sin phi4)
//   C 为 BC(l2) 与 DC(l3) 的交点，即轮轴
//   虚拟摆杆从 (l5/2, 0) 指向 C：长度 L0，角度 phi0
//
// 解支选择与原代码相同（phi1 > phi4 的"膝外翻"构型）。

#include <array>

namespace control {

struct FiveBarGeometry {
  float l1;  // 后大腿 AB
  float l2;  // 后小腿 BC
  float l3;  // 前小腿 DC
  float l4;  // 前大腿 ED
  float l5;  // 髋距 AE
};

// 2x2 矩阵，行优先：m[row][col]
using Mat2 = std::array<std::array<float, 2>, 2>;

struct LegKinematics {
  float l0 = 0.0f;    // 虚拟摆杆长度 (m)
  float phi0 = 0.0f;  // 虚拟摆杆角度 (rad)
  float phi2 = 0.0f;  // B 指向 C 的方向角
  float phi3 = 0.0f;  // C 指向 D 的方向角（与原代码定义一致）
  float xc = 0.0f;
  float yc = 0.0f;
  // [dL0, dphi0]^T = J · [dphi1, dphi4]^T
  Mat2 jacobian{};
};

struct LegRate {
  float d_l0;
  float d_phi0;
};

struct LegForce {
  float f;   // 沿摆杆方向的推力 (N)
  float tp;  // 绕髋的摆杆力矩 (N·m)
};

struct JointTorque {
  float t1;  // phi1 关节力矩 (N·m)
  float t4;  // phi4 关节力矩 (N·m)
};

struct JointAngle {
  float phi1;
  float phi4;
};

class FiveBarLeg {
 public:
  explicit FiveBarLeg(const FiveBarGeometry& geometry) : geo_(geometry) {}

  // 正运动学 + 雅可比。姿态不可达或处于奇异位形时返回 false，
  // 此时保留上一次的有效结果，不会写入 NaN。
  bool Update(float phi1, float phi4);

  bool Valid() const { return valid_; }
  const LegKinematics& Kinematics() const { return kin_; }

  // 关节角速度 -> 摆杆速度
  LegRate Velocity(float d_phi1, float d_phi4) const;

  // 期望摆杆力 -> 关节力矩：T = J^T · [F, Tp]^T
  JointTorque TorqueFromForce(const LegForce& force) const;

  // 实测关节力矩 -> 摆杆力（用于支撑力估计）：[F, Tp]^T = (J^T)^-1 · T
  // 雅可比奇异时返回 false。
  bool ForceFromTorque(const JointTorque& torque, LegForce& out) const;

  // 逆运动学：给定摆杆 (L0, phi0) 求关节角，不可达返回 false。
  bool Inverse(float l0, float phi0, JointAngle& out) const;

  const FiveBarGeometry& Geometry() const { return geo_; }

 private:
  FiveBarGeometry geo_;
  LegKinematics kin_{};
  bool valid_ = false;
};

}  // namespace control
