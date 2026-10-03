#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "drivers/mit_motor.hpp"

using drivers::CanPayload;
using drivers::DecodeFeedback;
using drivers::EncodeCommand;
using drivers::MitCommand;
using drivers::MitFeedback;
using drivers::MitRange;

namespace {

// ---------- 原版 F4 固件的黄金向量（见 tests/golden/gen_f4_vectors.c） ----------
struct EncRow {
  float pmax, vmax, tmax;
  float pos, vel, kp, kd, torque;
  CanPayload frame;
};
struct DecRow {
  float pmax, vmax, tmax;
  CanPayload frame;
  float pos, vel, torque;
  int mos, rotor, status;
};

const EncRow kEnc[] = {
#define MIT_GOLDEN_ENC
#include "golden/mit_motor_f4_vectors.inc"
#undef MIT_GOLDEN_ENC
};
const DecRow kDec[] = {
#define MIT_GOLDEN_DEC
#include "golden/mit_motor_f4_vectors.inc"
#undef MIT_GOLDEN_DEC
};

// 原固件把车上电机的 CAN_ID 配成 5（Tx ID 0x05），反馈帧低 4 位必须等于它才会被解析
constexpr std::uint8_t kGoldenMotorId = 5;

// 字段拆包
struct Fields {
  std::uint32_t p, v, kp, kd, t;
};
Fields Unpack(const CanPayload& d) {
  return {static_cast<std::uint32_t>((d[0] << 8) | d[1]), static_cast<std::uint32_t>((d[2] << 4) | (d[3] >> 4)),
          static_cast<std::uint32_t>(((d[3] & 0x0F) << 8) | d[4]), static_cast<std::uint32_t>((d[5] << 4) | (d[6] >> 4)),
          static_cast<std::uint32_t>(((d[6] & 0x0F) << 8) | d[7])};
}

MitRange RangeOf(float p, float v, float t) { return MitRange{p, v, t}; }

// 指令取到 -Max 时原固件会翻转符号（见下文专项测试），此类行单独验证
bool AtNegativeFullScale(const EncRow& r) {
  return r.pos <= -r.pmax || r.vel <= -r.vmax || r.torque <= -r.tmax;
}

void ExpectNear(std::uint32_t a, std::uint32_t b, std::uint32_t tol, const char* what, std::size_t row) {
  const auto diff = a > b ? a - b : b - a;
  EXPECT_LE(diff, tol) << what << " 行 " << row << ": 新=" << a << " 原版=" << b;
}

}  // namespace

// 常规量程内的指令，新编码与原固件输出相差不超过 1 个量化单位
// （原固件 0 点落在 2^(n-1)-1，新实现按达妙参考实现线性映射，斜率差约 0.02%）
TEST(MitEncode, MatchesOriginalFirmwareWithinOneLsb) {
  std::size_t checked = 0;
  for (std::size_t i = 0; i < std::size(kEnc); ++i) {
    const auto& r = kEnc[i];
    if (AtNegativeFullScale(r)) {
      continue;
    }
    const auto got = Unpack(EncodeCommand({r.pos, r.vel, r.kp, r.kd, r.torque}, RangeOf(r.pmax, r.vmax, r.tmax)));
    const auto ref = Unpack(r.frame);
    ExpectNear(got.p, ref.p, 1, "position", i);
    ExpectNear(got.v, ref.v, 1, "velocity", i);
    ExpectNear(got.kp, ref.kp, 1, "kp", i);
    ExpectNear(got.kd, ref.kd, 1, "kd", i);
    ExpectNear(got.t, ref.t, 1, "torque", i);
    ++checked;
  }
  EXPECT_GE(checked, 20u);  // 防止过滤条件写错导致空跑
}

// 零指令的字节与原固件完全一致：切换固件后电机看到的"零"不变
TEST(MitEncode, ZeroCommandIsByteIdenticalToOriginal) {
  const CanPayload expected{0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  EXPECT_EQ(EncodeCommand({}, RangeOf(95.5f, 45.0f, 18.0f)), expected);
  for (const auto& r : kEnc) {
    if (r.pos == 0 && r.vel == 0 && r.kp == 0 && r.kd == 0 && r.torque == 0) {
      EXPECT_EQ(r.frame, expected) << "黄金向量里的零指令行应与此一致";
    }
  }
}

// 原固件错误 1：指令取 -Max 时编码为 -1，截断成无符号后变成 +Max，
// 速度/力矩字段还会把高位溢出到相邻字段。新实现饱和到字段最小值 0。
TEST(MitEncode, NegativeFullScaleSaturatesInsteadOfFlippingSign) {
  const auto range = RangeOf(95.5f, 45.0f, 18.0f);
  std::size_t flipped_in_original = 0;
  for (const auto& r : kEnc) {
    if (!AtNegativeFullScale(r)) {
      continue;
    }
    const auto ref = Unpack(r.frame);
    const auto got = Unpack(EncodeCommand({r.pos, r.vel, r.kp, r.kd, r.torque}, range));
    if (r.pos <= -r.pmax) {
      EXPECT_EQ(ref.p, 0xFFFFu) << "原固件：-PMAX 被编码成 +PMAX";
      EXPECT_EQ(got.p, 0u);
    }
    if (r.vel <= -r.vmax) {
      EXPECT_EQ(ref.v, 0xFFFu) << "原固件：-VMAX 被编码成 +VMAX";
      EXPECT_EQ(got.v, 0u);
    }
    if (r.torque <= -r.tmax) {
      EXPECT_EQ(ref.t, 0xFFFu) << "原固件：-TMAX 被编码成 +TMAX";
      EXPECT_EQ(got.t, 0u);
    }
    ++flipped_in_original;
  }
  EXPECT_GE(flipped_in_original, 3u);
}

// 关键的安全性质：指令越负，编码值单调不增；不会在饱和处跳到另一侧
TEST(MitEncode, MonotonicAcrossFullRangeIncludingSaturation) {
  const auto range = RangeOf(12.5f, 30.0f, 10.0f);
  std::uint32_t last_t = 0, last_v = 0, last_p = 0;
  for (int i = -300; i <= 300; ++i) {  // 覆盖 ±Max 之外的过载指令
    const float f = static_cast<float>(i) / 200.0f;
    const auto got = Unpack(EncodeCommand({f * 12.5f, f * 30.0f, 0, 0, f * 10.0f}, range));
    if (i > -300) {
      EXPECT_GE(got.t, last_t) << "torque i=" << i;
      EXPECT_GE(got.v, last_v) << "velocity i=" << i;
      EXPECT_GE(got.p, last_p) << "position i=" << i;
    }
    last_t = got.t;
    last_v = got.v;
    last_p = got.p;
  }
}

TEST(MitEncode, OutOfRangeCommandsClampToLimits) {
  const auto range = RangeOf(12.5f, 30.0f, 10.0f);
  const auto hi = Unpack(EncodeCommand({99.0f, 99.0f, 9999.0f, 99.0f, 99.0f}, range));
  EXPECT_EQ(hi.p, 0xFFFFu);
  EXPECT_EQ(hi.v, 0xFFFu);
  EXPECT_EQ(hi.kp, 0xFFFu);
  EXPECT_EQ(hi.kd, 0xFFFu);
  EXPECT_EQ(hi.t, 0xFFFu);
  const auto lo = Unpack(EncodeCommand({-99.0f, -99.0f, -5.0f, -5.0f, -99.0f}, range));
  EXPECT_EQ(lo.p, 0u);
  EXPECT_EQ(lo.v, 0u);
  EXPECT_EQ(lo.kp, 0u);  // kp/kd 不能为负
  EXPECT_EQ(lo.kd, 0u);
  EXPECT_EQ(lo.t, 0u);
}

TEST(MitEncode, NonFiniteCommandsBecomeNeutral) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const auto range = RangeOf(12.5f, 30.0f, 10.0f);
  const auto neutral = EncodeCommand({}, range);
  EXPECT_EQ(EncodeCommand({nan, nan, nan, nan, nan}, range), neutral);
  EXPECT_EQ(EncodeCommand({inf, -inf, inf, -inf, inf}, range), neutral);
}

TEST(MitEncode, InvalidRangeYieldsNeutralFrameNotMotion) {
  const auto neutral = EncodeCommand({}, RangeOf(12.5f, 30.0f, 10.0f));
  EXPECT_EQ(EncodeCommand({5.0f, 5.0f, 100.0f, 2.0f, 5.0f}, RangeOf(0.0f, 30.0f, 10.0f)), neutral);
  EXPECT_EQ(EncodeCommand({5.0f, 5.0f, 100.0f, 2.0f, 5.0f}, RangeOf(12.5f, -1.0f, 10.0f)), neutral);
}

TEST(MitRoundTrip, FeedbackDecodesWhatCommandEncodesWithinOneLsb) {
  const auto range = RangeOf(95.5f, 45.0f, 18.0f);
  const float p_lsb = 2 * 95.5f / 65535.0f, v_lsb = 2 * 45.0f / 4095.0f, t_lsb = 2 * 18.0f / 4095.0f;
  for (float f : {-1.0f, -0.73f, -0.2f, 0.0f, 0.31f, 0.9f, 1.0f}) {
    const auto frame = EncodeCommand({f * 95.5f, f * 45.0f, 0, 0, f * 18.0f}, range);
    // 把指令帧当反馈帧的布局重排：位置 [0:1] -> 反馈 [1:2]；速度/力矩字段位置不同，故只拼反馈格式
    const auto u = Unpack(frame);
    CanPayload fb{0x15,
                  static_cast<std::uint8_t>(u.p >> 8),
                  static_cast<std::uint8_t>(u.p & 0xFF),
                  static_cast<std::uint8_t>(u.v >> 4),
                  static_cast<std::uint8_t>(((u.v & 0x0F) << 4) | (u.t >> 8)),
                  static_cast<std::uint8_t>(u.t & 0xFF),
                  0,
                  0};
    MitFeedback out;
    ASSERT_TRUE(DecodeFeedback(fb, range, out));
    EXPECT_NEAR(out.position, f * 95.5f, 1.5f * p_lsb) << "f=" << f;
    EXPECT_NEAR(out.velocity, f * 45.0f, 1.5f * v_lsb) << "f=" << f;
    EXPECT_NEAR(out.torque, f * 18.0f, 1.5f * t_lsb) << "f=" << f;
  }
}

// ---------- 解码 ----------

// 速度、力矩、温度：与原固件一致（这些路径在原固件里没有错误）
TEST(MitDecode, VelocityTorqueAndTemperatureMatchOriginalFirmware) {
  std::size_t checked = 0;
  for (std::size_t i = 0; i < std::size(kDec); ++i) {
    const auto& r = kDec[i];
    if ((r.frame[0] & 0x0F) != kGoldenMotorId) {
      continue;  // 原固件按电机 ID 丢弃，输出全零，没有可比较的内容
    }
    MitFeedback out;
    ASSERT_TRUE(DecodeFeedback(r.frame, RangeOf(r.pmax, r.vmax, r.tmax), out));
    const float v_lsb = 2 * r.vmax / 4095.0f, t_lsb = 2 * r.tmax / 4095.0f;
    EXPECT_NEAR(out.velocity, r.vel, 1.5f * v_lsb) << "行 " << i;
    EXPECT_NEAR(out.torque, r.torque, 1.5f * t_lsb) << "行 " << i;
    EXPECT_EQ(out.mos_temp, r.mos) << "行 " << i;
    EXPECT_EQ(out.rotor_temp, r.rotor) << "行 " << i;
    ++checked;
  }
  EXPECT_GE(checked, 10u);
}

// 原固件错误 2：Math_Endian_Reverse_16_(&x, &x) 源与目的相同，第二次赋值读到被覆盖的值，
// 结果两个字节都变成高字节，位置只剩 8 位有效分辨率（约 0.1 rad）。
TEST(MitDecode, PositionUsesBothBytes) {
  std::size_t differing_rows = 0;
  for (std::size_t i = 0; i < std::size(kDec); ++i) {
    const auto& r = kDec[i];
    if ((r.frame[0] & 0x0F) != kGoldenMotorId) {
      continue;
    }
    const float expect_right =
        static_cast<float>((r.frame[1] << 8) | r.frame[2]) * (2 * r.pmax) / 65535.0f - r.pmax;
    // 原固件解码位置时把量程硬编码成 ±12.5，不随 Angle_Max 变化（发送侧却用 Angle_Max）
    constexpr float kOriginalRxPMax = 12.5f;
    const float expect_original =
        static_cast<float>((r.frame[1] << 8) | r.frame[1]) * (2 * kOriginalRxPMax) / 65535.0f - kOriginalRxPMax;

    MitFeedback out;
    ASSERT_TRUE(DecodeFeedback(r.frame, RangeOf(r.pmax, r.vmax, r.tmax), out));
    EXPECT_NEAR(out.position, expect_right, 1e-3f) << "行 " << i;
    // 原固件输出 == "高字节重复" 的结果：这既说明了错误原因，也验证了黄金向量本身
    EXPECT_NEAR(r.pos, expect_original, 1e-3f) << "行 " << i << " 原固件输出应等于高字节重复";
    if (r.frame[1] != r.frame[2]) {
      EXPECT_GT(std::fabs(r.pos - expect_right), 1e-3f) << "行 " << i << ": 原固件在高低字节不同时位置出错";
      ++differing_rows;
    }
  }
  EXPECT_GE(differing_rows, 4u);
}

// 原固件接收侧位置量程与 Angle_Max 无关：同一帧在 Angle_Max=95.5 和 12.5 下输出相同。
// 新实现收发统一用 p_max，所以两者给出不同结果。电机实际用哪个量程要上板确认。
TEST(MitDecode, PositionRangeFollowsConfiguredPMax) {
  const CanPayload frame{0x15, 0xC0, 0x00, 0x80, 0x08, 0x00, 0x28, 0x2D};
  MitFeedback wide, narrow;
  ASSERT_TRUE(DecodeFeedback(frame, RangeOf(95.5f, 45.0f, 18.0f), wide));
  ASSERT_TRUE(DecodeFeedback(frame, RangeOf(12.5f, 45.0f, 18.0f), narrow));
  EXPECT_NEAR(wide.position / narrow.position, 95.5f / 12.5f, 1e-3f);

  std::size_t pairs = 0;
  for (std::size_t i = 0; i + 1 < std::size(kDec); ++i) {
    if (kDec[i].frame == kDec[i + 1].frame && kDec[i].pmax != kDec[i + 1].pmax) {
      EXPECT_FLOAT_EQ(kDec[i].pos, kDec[i + 1].pos) << "原固件位置应与 Angle_Max 无关，行 " << i;
      ++pairs;
    }
  }
  EXPECT_GE(pairs, 6u);
}

// 原固件错误 3：状态读的是 CAN_ID 位域右移 4 位（该位域只有 4 位，恒为 0）。
TEST(MitDecode, StateComesFromHighNibble) {
  for (const auto& r : kDec) {
    if ((r.frame[0] & 0x0F) != kGoldenMotorId) {
      continue;
    }
    MitFeedback out;
    ASSERT_TRUE(DecodeFeedback(r.frame, RangeOf(r.pmax, r.vmax, r.tmax), out));
    EXPECT_EQ(out.state, r.frame[0] >> 4);
    EXPECT_EQ(r.status, 0) << "原固件状态恒为 0";
  }
  MitFeedback out;
  ASSERT_TRUE(DecodeFeedback({0x85, 0x80, 0x00, 0x80, 0x08, 0x00, 0x28, 0x2D}, RangeOf(12.5f, 45.0f, 18.0f), out));
  EXPECT_EQ(out.state, 0x8);  // 过压
  EXPECT_EQ(out.motor_id, 0x5);
}

// 共用同一个反馈 ID 的多个电机靠数据区低 4 位区分
TEST(MitDecode, ReportsMotorIdForDispatch) {
  MitFeedback out;
  for (std::uint8_t id = 1; id <= 6; ++id) {
    ASSERT_TRUE(DecodeFeedback({static_cast<std::uint8_t>(0x10 | id), 0x80, 0, 0x80, 0x08, 0, 0, 0},
                               RangeOf(12.5f, 45.0f, 18.0f), out));
    EXPECT_EQ(out.motor_id, id);
    EXPECT_EQ(out.state, 0x1);
  }
}

TEST(MitDecode, FullScaleFramesHitRangeLimits) {
  MitFeedback out;
  ASSERT_TRUE(DecodeFeedback({0x15, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x80}, RangeOf(12.5f, 45.0f, 18.0f), out));
  EXPECT_FLOAT_EQ(out.position, 12.5f);
  EXPECT_FLOAT_EQ(out.velocity, 45.0f);
  EXPECT_FLOAT_EQ(out.torque, 18.0f);
  EXPECT_EQ(out.mos_temp, 127);
  EXPECT_EQ(out.rotor_temp, -128);  // int8，与原固件一致
  ASSERT_TRUE(DecodeFeedback({0x15, 0, 0, 0, 0, 0, 0, 0}, RangeOf(12.5f, 45.0f, 18.0f), out));
  EXPECT_FLOAT_EQ(out.position, -12.5f);
  EXPECT_FLOAT_EQ(out.velocity, -45.0f);
  EXPECT_FLOAT_EQ(out.torque, -18.0f);
}

TEST(MitDecode, InvalidRangeRejectedAndOutputUntouched) {
  MitFeedback out;
  out.position = 1.25f;
  EXPECT_FALSE(DecodeFeedback({}, RangeOf(0.0f, 1.0f, 1.0f), out));
  EXPECT_FLOAT_EQ(out.position, 1.25f);
}

// ---------- 模式帧 ----------
TEST(MitFrames, ModeFramesMatchOriginalFirmware) {
  EXPECT_EQ(drivers::kEnableFrame, (CanPayload{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}));
  EXPECT_EQ(drivers::kDisableFrame, (CanPayload{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}));
  EXPECT_EQ(drivers::kSetZeroFrame, (CanPayload{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}));
  EXPECT_EQ(drivers::kClearErrorFrame, (CanPayload{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB}));
}
