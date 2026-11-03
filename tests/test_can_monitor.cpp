#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "drivers/can_monitor.hpp"

using drivers::CanFrameRecord;
using drivers::CanMonitor;

namespace {
using Monitor = CanMonitor<3, 4>;
constexpr std::uint8_t kFeedback[8] = {0x12, 0x34, 0x00, 0x64, 0x00, 0x0A, 0x1E, 0x00};
}  // namespace

TEST(CanMonitor, AggregatesFramesPerBusAndId) {
  Monitor m;
  m.OnFrame(0, 0x201, false, kFeedback, 8, 100);
  m.OnFrame(0, 0x201, false, kFeedback, 8, 101);
  m.OnFrame(0, 0x202, false, kFeedback, 8, 101);
  m.OnFrame(1, 0x201, false, kFeedback, 8, 105);

  EXPECT_EQ(m.Total(0), 3u);
  EXPECT_EQ(m.Total(1), 1u);
  EXPECT_EQ(m.Total(2), 0u);
  EXPECT_EQ(m.Used(0), 2u);
  EXPECT_EQ(m.Used(1), 1u);

  CanFrameRecord r;
  ASSERT_TRUE(m.Snapshot(0, 0, r));
  EXPECT_EQ(r.id, 0x201u);
  EXPECT_EQ(r.count, 2u);
  EXPECT_EQ(r.first_ms, 100u);
  EXPECT_EQ(r.last_ms, 101u);
  EXPECT_EQ(r.data[0], 0x12);
  EXPECT_EQ(r.data[7], 0x00);
}

TEST(CanMonitor, SameIdOnDifferentBusesIsIndependent) {
  Monitor m;
  m.OnFrame(0, 0x201, false, kFeedback, 8, 1);
  CanFrameRecord r;
  EXPECT_FALSE(m.Snapshot(1, 0, r));  // 总线 1 上没有任何条目
  EXPECT_EQ(m.Used(1), 0u);
}

TEST(CanMonitor, StandardAndExtendedIdsDoNotCollide) {
  Monitor m;
  m.OnFrame(0, 0x201, false, kFeedback, 8, 1);
  m.OnFrame(0, 0x201, true, kFeedback, 8, 2);
  EXPECT_EQ(m.Used(0), 2u);
}

TEST(CanMonitor, KeepsLatestPayload) {
  Monitor m;
  const std::uint8_t a[8] = {1, 1, 1, 1, 1, 1, 1, 1};
  const std::uint8_t b[8] = {9, 8, 7, 6, 5, 4, 3, 2};
  m.OnFrame(0, 0x205, false, a, 8, 1);
  m.OnFrame(0, 0x205, false, b, 8, 2);
  CanFrameRecord r;
  ASSERT_TRUE(m.Snapshot(0, 0, r));
  EXPECT_EQ(r.data, (std::array<std::uint8_t, 8>{9, 8, 7, 6, 5, 4, 3, 2}));
  EXPECT_EQ(r.count, 2u);
}

TEST(CanMonitor, ShortFramePadsWithZeroAndClampsLength) {
  Monitor m;
  const std::uint8_t full[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  m.OnFrame(0, 0x100, false, full, 8, 1);
  const std::uint8_t two[2] = {0x11, 0x22};
  m.OnFrame(0, 0x100, false, two, 2, 2);  // 上一帧的残留数据不能泄露进短帧
  CanFrameRecord r;
  ASSERT_TRUE(m.Snapshot(0, 0, r));
  EXPECT_EQ(r.len, 2);
  EXPECT_EQ(r.data, (std::array<std::uint8_t, 8>{0x11, 0x22, 0, 0, 0, 0, 0, 0}));

  m.OnFrame(0, 0x101, false, full, 200, 3);  // 异常长度：钳到 8，不越界读
  ASSERT_TRUE(m.Snapshot(0, 1, r));
  EXPECT_EQ(r.len, 8);
}

TEST(CanMonitor, CapacityOverflowIsCountedNotCorrupting) {
  Monitor m;  // 每条总线 4 个槽
  for (std::uint32_t id = 0x201; id < 0x201 + 6; ++id) {
    m.OnFrame(0, id, false, kFeedback, 8, id);
  }
  EXPECT_EQ(m.Used(0), 4u);
  EXPECT_EQ(m.Total(0), 6u);
  EXPECT_EQ(m.Overflow(0), 2u);

  // 已存在的 ID 在满了之后仍可继续更新
  m.OnFrame(0, 0x201, false, kFeedback, 8, 999);
  CanFrameRecord r;
  ASSERT_TRUE(m.Snapshot(0, 0, r));
  EXPECT_EQ(r.count, 2u);
  EXPECT_EQ(r.last_ms, 999u);
  EXPECT_EQ(m.Overflow(0), 2u);
}

TEST(CanMonitor, OutOfRangeBusIsRejectedSafely) {
  Monitor m;
  m.OnFrame(3, 0x201, false, kFeedback, 8, 1);
  m.OnFrame(99, 0x201, false, kFeedback, 8, 1);
  EXPECT_EQ(m.BadBus(), 2u);
  EXPECT_EQ(m.Total(0) + m.Total(1) + m.Total(2), 0u);
  EXPECT_EQ(m.Total(3), 0u);  // 越界读也是安全的
  CanFrameRecord r;
  EXPECT_FALSE(m.Snapshot(3, 0, r));
  EXPECT_FALSE(m.Snapshot(0, 0, r));  // 空总线
}

TEST(CanMonitor, SnapshotIndexOutOfRangeFails) {
  Monitor m;
  m.OnFrame(0, 0x201, false, kFeedback, 8, 1);
  CanFrameRecord r;
  EXPECT_TRUE(m.Snapshot(0, 0, r));
  EXPECT_FALSE(m.Snapshot(0, 1, r));
}

TEST(CanMonitor, ResetClearsEverything) {
  Monitor m;
  m.OnFrame(0, 0x201, false, kFeedback, 8, 1);
  m.OnFrame(9, 0x201, false, kFeedback, 8, 1);
  m.Reset();
  EXPECT_EQ(m.Total(0), 0u);
  EXPECT_EQ(m.Used(0), 0u);
  EXPECT_EQ(m.BadBus(), 0u);
}

// 这是 8 个 M3508 同时上线时的真实规模：ID 0x201..0x208，每个 1 kHz
TEST(CanMonitor, HandlesEightMotorsAtOneKilohertz) {
  CanMonitor<1, 16> m;
  for (std::uint32_t ms = 0; ms < 1000; ++ms) {
    for (std::uint32_t id = 0x201; id <= 0x208; ++id) {
      m.OnFrame(0, id, false, kFeedback, 8, ms);
    }
  }
  EXPECT_EQ(m.Total(0), 8000u);
  EXPECT_EQ(m.Used(0), 8u);
  CanFrameRecord r;
  for (std::size_t i = 0; i < 8; ++i) {
    ASSERT_TRUE(m.Snapshot(0, i, r));
    EXPECT_EQ(r.count, 1000u);
    EXPECT_EQ(r.first_ms, 0u);
    EXPECT_EQ(r.last_ms, 999u);
  }
}
