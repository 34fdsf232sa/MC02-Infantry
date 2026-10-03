#include "drivers/mit_motor.hpp"

#include <cmath>

namespace drivers {
namespace {

// 非有限值（NaN / ±inf）按 0 处理：电机收到中性值，而不是未定义的转换结果
float Sanitize(float x) { return std::isfinite(x) ? x : 0.0f; }

float Clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// [lo, hi] -> [0, 2^bits - 1]，截断取整（与达妙参考实现一致，0 落在 2^(bits-1)-1）
std::uint16_t ToUint(float x, float lo, float hi, int bits) {
  const float full = static_cast<float>((1u << bits) - 1u);
  const float v = (Clamp(Sanitize(x), lo, hi) - lo) / (hi - lo) * full;
  const auto u = static_cast<std::uint32_t>(v);  // v ∈ [0, full]，转换不会越界
  return static_cast<std::uint16_t>(u > (1u << bits) - 1u ? (1u << bits) - 1u : u);
}

float ToFloat(std::uint16_t u, float lo, float hi, int bits) {
  const float full = static_cast<float>((1u << bits) - 1u);
  return static_cast<float>(u) * (hi - lo) / full + lo;
}

}  // namespace

CanPayload EncodeCommand(const MitCommand& cmd, const MitRange& range) {
  const MitCommand zero{};
  const MitCommand& c = range.Valid() ? cmd : zero;
  const MitRange r = range.Valid() ? range : MitRange{1.0f, 1.0f, 1.0f};

  const std::uint16_t p = ToUint(c.position, -r.p_max, r.p_max, 16);
  const std::uint16_t v = ToUint(c.velocity, -r.v_max, r.v_max, 12);
  const std::uint16_t kp = ToUint(c.kp, 0.0f, r.kp_max, 12);
  const std::uint16_t kd = ToUint(c.kd, 0.0f, r.kd_max, 12);
  const std::uint16_t t = ToUint(c.torque, -r.t_max, r.t_max, 12);

  return CanPayload{
      static_cast<std::uint8_t>(p >> 8),
      static_cast<std::uint8_t>(p & 0xFF),
      static_cast<std::uint8_t>(v >> 4),
      static_cast<std::uint8_t>(((v & 0x0F) << 4) | (kp >> 8)),
      static_cast<std::uint8_t>(kp & 0xFF),
      static_cast<std::uint8_t>(kd >> 4),
      static_cast<std::uint8_t>(((kd & 0x0F) << 4) | (t >> 8)),
      static_cast<std::uint8_t>(t & 0xFF),
  };
}

bool DecodeFeedback(const CanPayload& d, const MitRange& range, MitFeedback& out) {
  if (!range.Valid()) {
    return false;
  }
  const auto p = static_cast<std::uint16_t>((d[1] << 8) | d[2]);
  const auto v = static_cast<std::uint16_t>((d[3] << 4) | (d[4] >> 4));
  const auto t = static_cast<std::uint16_t>(((d[4] & 0x0F) << 8) | d[5]);

  out.motor_id = d[0] & 0x0F;
  out.state = d[0] >> 4;
  out.position = ToFloat(p, -range.p_max, range.p_max, 16);
  out.velocity = ToFloat(v, -range.v_max, range.v_max, 12);
  out.torque = ToFloat(t, -range.t_max, range.t_max, 12);
  out.mos_temp = static_cast<std::int8_t>(d[6]);
  out.rotor_temp = static_cast<std::int8_t>(d[7]);
  return true;
}

}  // namespace drivers
