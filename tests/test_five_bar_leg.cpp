#include <gtest/gtest.h>

#include <cmath>

#include "control/five_bar_leg.hpp"

using control::FiveBarGeometry;
using control::FiveBarLeg;

namespace {

constexpr float kPi = 3.14159265358979f;
// 原 F4 车的腿（vmc.c: L1=0.215, L2=0.258, 共轴髋）
constexpr FiveBarGeometry kOriginal{0.215f, 0.258f, 0.258f, 0.215f, 0.0f};
// 非对称、髋距非零的一般构型，用来确认公式不依赖对称性
constexpr FiveBarGeometry kGeneral{0.15f, 0.27f, 0.27f, 0.15f, 0.10f};

struct Pose {
  float phi1;
  float phi4;
};

float Wrap(float a) { return std::remainder(a, 2.0f * kPi); }

void ExpectLinkLengths(const FiveBarLeg& leg, float phi1, float phi4) {
  const auto& g = leg.Geometry();
  const auto& k = leg.Kinematics();
  const float xb = g.l1 * std::cos(phi1);
  const float yb = g.l1 * std::sin(phi1);
  const float xd = g.l5 + g.l4 * std::cos(phi4);
  const float yd = g.l4 * std::sin(phi4);
  EXPECT_NEAR(std::hypot(k.xc - xb, k.yc - yb), g.l2, 1e-5f);
  EXPECT_NEAR(std::hypot(k.xc - xd, k.yc - yd), g.l3, 1e-5f);
}

}  // namespace

// 与原 vmc.c 公式在同一姿态下的输出对照（数值由原公式离线计算）
TEST(FiveBarLeg, MatchesOriginalFormulaAtSymmetricPose) {
  FiveBarLeg leg(kOriginal);
  ASSERT_TRUE(leg.Update(kPi / 2 + 0.6f, kPi / 2 - 0.6f));
  const auto& k = leg.Kinematics();
  EXPECT_NEAR(k.l0, 0.4051f, 1e-4f);
  EXPECT_NEAR(k.phi0, kPi / 2, 1e-5f);
  EXPECT_NEAR(k.jacobian[0][0], -0.10801f, 1e-4f);
  EXPECT_NEAR(k.jacobian[0][1], 0.10801f, 1e-4f);
  EXPECT_NEAR(k.jacobian[1][0], 0.5f, 1e-4f);
  EXPECT_NEAR(k.jacobian[1][1], 0.5f, 1e-4f);
}

class FiveBarPoseTest
    : public ::testing::TestWithParam<std::tuple<FiveBarGeometry, Pose>> {};

TEST_P(FiveBarPoseTest, SatisfiesLinkConstraints) {
  const auto [geo, pose] = GetParam();
  FiveBarLeg leg(geo);
  ASSERT_TRUE(leg.Update(pose.phi1, pose.phi4));
  ExpectLinkLengths(leg, pose.phi1, pose.phi4);
}

TEST_P(FiveBarPoseTest, JacobianMatchesFiniteDifference) {
  const auto [geo, pose] = GetParam();
  FiveBarLeg leg(geo);
  ASSERT_TRUE(leg.Update(pose.phi1, pose.phi4));
  const auto j = leg.Kinematics().jacobian;

  constexpr float h = 1e-3f;
  auto eval = [&](float p1, float p4) {
    FiveBarLeg probe(geo);
    EXPECT_TRUE(probe.Update(p1, p4));
    return std::pair{probe.Kinematics().l0, probe.Kinematics().phi0};
  };
  const auto [l0_p1p, phi0_p1p] = eval(pose.phi1 + h, pose.phi4);
  const auto [l0_p1m, phi0_p1m] = eval(pose.phi1 - h, pose.phi4);
  const auto [l0_p4p, phi0_p4p] = eval(pose.phi1, pose.phi4 + h);
  const auto [l0_p4m, phi0_p4m] = eval(pose.phi1, pose.phi4 - h);

  EXPECT_NEAR(j[0][0], (l0_p1p - l0_p1m) / (2 * h), 2e-3f);
  EXPECT_NEAR(j[0][1], (l0_p4p - l0_p4m) / (2 * h), 2e-3f);
  EXPECT_NEAR(j[1][0], (phi0_p1p - phi0_p1m) / (2 * h), 2e-3f);
  EXPECT_NEAR(j[1][1], (phi0_p4p - phi0_p4m) / (2 * h), 2e-3f);
}

// 虚功原理：F·dL0 + Tp·dphi0 == T1·dphi1 + T4·dphi4，对任意关节速度成立
TEST_P(FiveBarPoseTest, TorqueMappingConservesPower) {
  const auto [geo, pose] = GetParam();
  FiveBarLeg leg(geo);
  ASSERT_TRUE(leg.Update(pose.phi1, pose.phi4));

  const control::LegForce force{85.0f, -3.2f};
  const auto torque = leg.TorqueFromForce(force);
  for (const auto [w1, w4] : {std::pair{1.0f, 0.0f}, std::pair{0.0f, 1.0f},
                              std::pair{-0.7f, 2.3f}}) {
    const auto rate = leg.Velocity(w1, w4);
    const float leg_power = force.f * rate.d_l0 + force.tp * rate.d_phi0;
    const float joint_power = torque.t1 * w1 + torque.t4 * w4;
    EXPECT_NEAR(leg_power, joint_power, 1e-3f);
  }
}

TEST_P(FiveBarPoseTest, ForceFromTorqueInvertsTorqueFromForce) {
  const auto [geo, pose] = GetParam();
  FiveBarLeg leg(geo);
  ASSERT_TRUE(leg.Update(pose.phi1, pose.phi4));

  const control::LegForce force{60.0f, 1.5f};
  control::LegForce recovered{};
  ASSERT_TRUE(leg.ForceFromTorque(leg.TorqueFromForce(force), recovered));
  EXPECT_NEAR(recovered.f, force.f, 1e-2f);
  EXPECT_NEAR(recovered.tp, force.tp, 1e-3f);
}

TEST_P(FiveBarPoseTest, InverseRoundTrips) {
  const auto [geo, pose] = GetParam();
  FiveBarLeg leg(geo);
  ASSERT_TRUE(leg.Update(pose.phi1, pose.phi4));

  control::JointAngle joints{};
  ASSERT_TRUE(leg.Inverse(leg.Kinematics().l0, leg.Kinematics().phi0, joints));
  EXPECT_NEAR(Wrap(joints.phi1 - pose.phi1), 0.0f, 1e-4f);
  EXPECT_NEAR(Wrap(joints.phi4 - pose.phi4), 0.0f, 1e-4f);
}

INSTANTIATE_TEST_SUITE_P(
    Poses, FiveBarPoseTest,
    ::testing::Values(
        std::tuple{kOriginal, Pose{kPi / 2 + 0.6f, kPi / 2 - 0.6f}},
        std::tuple{kOriginal, Pose{kPi / 2 + 0.9f, kPi / 2 - 0.3f}},
        std::tuple{kOriginal, Pose{kPi / 2 + 1.2f, kPi / 2 - 1.2f}},
        std::tuple{kGeneral, Pose{kPi / 2 + 0.9f, kPi / 2 - 0.7f}},
        std::tuple{kGeneral, Pose{kPi / 2 + 0.5f, kPi / 2 - 0.4f}}));

// 共轴对称腿：逆解满足原代码的简化关系 phi4 = 2·phi0 - phi1
TEST(FiveBarLeg, SymmetricInverseMatchesSerialShortcut) {
  FiveBarLeg leg(kOriginal);
  control::JointAngle joints{};
  ASSERT_TRUE(leg.Inverse(0.25f, kPi / 2 + 0.1f, joints));
  EXPECT_NEAR(Wrap(joints.phi4 - (2 * (kPi / 2 + 0.1f) - joints.phi1)), 0.0f,
              1e-4f);
}

TEST(FiveBarLeg, UnreachablePoseKeepsLastValidState) {
  // 小腿很短的构型，才存在 |BD| > l2 + l3 的闭链断开姿态
  constexpr FiveBarGeometry kShortShank{0.2f, 0.1f, 0.1f, 0.2f, 0.0f};
  FiveBarLeg leg(kShortShank);
  ASSERT_TRUE(leg.Update(kPi / 2 + 0.3f, kPi / 2 - 0.3f));
  const float l0 = leg.Kinematics().l0;

  // B = (-0.2, 0)，D = (0.2, 0)，|BD| = 0.4 > 0.2
  EXPECT_FALSE(leg.Update(kPi, 0.0f));
  EXPECT_FALSE(leg.Valid());
  EXPECT_FLOAT_EQ(leg.Kinematics().l0, l0);
  EXPECT_FALSE(std::isnan(leg.Kinematics().jacobian[0][0]));

  control::JointAngle joints{};
  EXPECT_FALSE(leg.Inverse(1.0f, kPi / 2, joints));  // 超过最大腿长
}
