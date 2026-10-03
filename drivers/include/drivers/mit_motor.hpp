#pragma once

// MIT 运控协议编解码（铁蛋关节电机 / 达妙风格）。纯函数，无硬件依赖，可在主机上单测。
//
// 帧格式（标准帧，8 字节，大端位序）：
//   指令  [0:1] 位置 16 位 | [2] 速度[11:4] | [3] 速度[3:0]<<4 | kp[11:8] | [4] kp[7:0]
//         [5] kd[11:4] | [6] kd[3:0]<<4 | 力矩[11:8] | [7] 力矩[7:0]
//   反馈  [0] 状态<<4 | 电机ID | [1:2] 位置 16 位 | [3] 速度[11:4] | [4] 速度[3:0]<<4 | 力矩[11:8]
//         [5] 力矩[7:0] | [6] MOS 温度 (int8) | [7] 转子温度 (int8)
//   模式帧  FF FF FF FF FF FF FF {FC 使能, FD 失能, FE 存零点, FB 清错误}
//
// 由原 F4 固件 mi_motor.c 移植，并修正了其中三处错误（见 tests/test_mit_motor.cpp）：
//   1. 指令取 -Max 时编码为 -1，截断成无符号后变成 +Max（符号翻转），并溢出到相邻字段；
//   2. 反馈位置用 Math_Endian_Reverse_16_(&x, &x) 原地交换字节，结果两个字节都变成高字节；
//   3. 反馈状态读 CAN_ID 字段右移 4 位，恒为 0。

#include <array>
#include <cstdint>

namespace drivers {

using CanPayload = std::array<std::uint8_t, 8>;

// 量程必须与电机上位机里设置的 PMAX / VMAX / TMAX 一致。
// 原固件发送侧用 Angle_Max，接收侧却把位置硬编码成 ±12.5；本模块收发统一用 p_max，
// 该值是否与实际电机一致需要上板实测确认（见 README）。
struct MitRange {
  float p_max;        // 位置量程 (rad)，对称 ±p_max
  float v_max;        // 速度量程 (rad/s)
  float t_max;        // 力矩量程 (N·m)
  float kp_max = 500.0f;
  float kd_max = 5.0f;

  constexpr bool Valid() const {
    return p_max > 0.0f && v_max > 0.0f && t_max > 0.0f && kp_max > 0.0f && kd_max > 0.0f;
  }
};

struct MitCommand {
  float position = 0.0f;  // rad
  float velocity = 0.0f;  // rad/s
  float kp = 0.0f;
  float kd = 0.0f;
  float torque = 0.0f;  // N·m 前馈
};

struct MitFeedback {
  std::uint8_t motor_id = 0;  // 数据区 [0] 低 4 位，调用方据此区分共用反馈 ID 的多个电机
  std::uint8_t state = 0;     // 数据区 [0] 高 4 位：0 失能 1 使能 8 超压 9 欠压 A 过流 B MOS 过温
                              //                     C 绕组过温 D 通讯丢失 E 过载
  float position = 0.0f;
  float velocity = 0.0f;
  float torque = 0.0f;
  std::int8_t mos_temp = 0;
  std::int8_t rotor_temp = 0;
};

inline constexpr CanPayload kEnableFrame{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
inline constexpr CanPayload kDisableFrame{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
inline constexpr CanPayload kSetZeroFrame{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
inline constexpr CanPayload kClearErrorFrame{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

// 指令先限幅到量程（非有限值按 0 处理），再量化；-Max 编码为 0，不会翻转符号。
// 量程无效时返回全零指令帧（零位置、零增益、零力矩），不会给电机施加力。
CanPayload EncodeCommand(const MitCommand& cmd, const MitRange& range);

// 解码反馈帧。量程无效返回 false，此时 out 不被修改。
// 这里不做电机 ID 过滤：共用反馈 ID 的多个电机由调用方按 motor_id 分发。
bool DecodeFeedback(const CanPayload& data, const MitRange& range, MitFeedback& out);

}  // namespace drivers
