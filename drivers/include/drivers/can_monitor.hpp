#pragma once

// CAN 总线被动监视：按 (总线, ID) 聚合接收到的帧。纯逻辑，无硬件依赖，可在主机上单测。
//
// 设计约束：
//  - OnFrame() 可能在中断里被调用：O(1)、无分配、无锁，只做计数和拷贝。
//  - 只有一个写者（该总线的接收中断），读者（终端命令）只读快照，
//    每个槽用序号（奇数=写入中）保证读到一致的帧，读者遇到写入中就重试。
//  - 容量固定，满了之后新的 ID 计入 overflow，不会丢统计，也不会越界。
//  - 本模块不提供任何发送能力。

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace drivers {

struct CanFrameRecord {
  std::uint32_t id = 0;
  bool extended = false;
  std::uint8_t len = 8;
  std::array<std::uint8_t, 8> data{};
  std::uint32_t count = 0;         // 累计帧数
  std::uint32_t first_ms = 0;      // 首次收到的时间
  std::uint32_t last_ms = 0;       // 最近一次收到的时间
};

template <std::size_t kBuses, std::size_t kSlots>
class CanMonitor {
 public:
  // 在接收回调里调用。bus 越界的帧被忽略并计入 bad_bus。
  void OnFrame(std::size_t bus, std::uint32_t id, bool extended, const std::uint8_t* data,
               std::uint8_t len, std::uint32_t now_ms) {
    if (bus >= kBuses) {
      ++bad_bus_;
      return;
    }
    Bus& b = buses_[bus];
    ++b.total;
    Slot* slot = Find(b, id, extended);
    const bool is_new = (slot == nullptr);
    if (is_new) {
      if (b.used >= kSlots) {
        ++b.overflow;
        return;
      }
      slot = &b.slots[b.used];
    }

    // 单写者 seqlock：序号奇数表示写入中。单核 MCU 上中断可能打断读者，
    // 所以只需要编译器屏障，防止编译器把字段写入挪到序号更新的外面。
    slot->seq = slot->seq + 1;
    std::atomic_signal_fence(std::memory_order_seq_cst);
    if (is_new) {
      slot->rec.id = id;
      slot->rec.extended = extended;
      slot->rec.first_ms = now_ms;
      slot->rec.count = 0;
    }
    slot->rec.len = len > 8 ? 8 : len;
    for (std::size_t i = 0; i < 8; ++i) {
      slot->rec.data[i] = i < len ? data[i] : 0;
    }
    slot->rec.last_ms = now_ms;
    ++slot->rec.count;
    std::atomic_signal_fence(std::memory_order_seq_cst);
    slot->seq = slot->seq + 1;

    // 新条目填完整之后才发布，读者永远看不到半成品
    if (is_new) {
      std::atomic_signal_fence(std::memory_order_seq_cst);
      b.used = b.used + 1;
    }
  }

  std::size_t BusCount() const { return kBuses; }
  std::uint32_t Total(std::size_t bus) const { return bus < kBuses ? buses_[bus].total : 0; }
  std::uint32_t Overflow(std::size_t bus) const { return bus < kBuses ? buses_[bus].overflow : 0; }
  std::uint32_t BadBus() const { return bad_bus_; }
  std::size_t Used(std::size_t bus) const { return bus < kBuses ? buses_[bus].used : 0; }

  // 读取第 index 个 ID 的一致快照；失败（越界或连续写入冲突）返回 false。
  bool Snapshot(std::size_t bus, std::size_t index, CanFrameRecord& out) const {
    if (bus >= kBuses || index >= buses_[bus].used) {
      return false;
    }
    const Slot& s = buses_[bus].slots[index];
    for (int attempt = 0; attempt < 8; ++attempt) {
      const std::uint32_t before = s.seq;
      if (before & 1u) {
        continue;
      }
      std::atomic_signal_fence(std::memory_order_seq_cst);
      out = s.rec;
      std::atomic_signal_fence(std::memory_order_seq_cst);
      if (s.seq == before) {
        return true;
      }
    }
    return false;
  }

  void Reset() {
    for (auto& b : buses_) {
      b = Bus{};
    }
    bad_bus_ = 0;
  }

 private:
  struct Slot {
    volatile std::uint32_t seq = 0;
    CanFrameRecord rec{};
  };
  struct Bus {
    std::array<Slot, kSlots> slots{};
    volatile std::size_t used = 0;
    std::uint32_t total = 0;
    std::uint32_t overflow = 0;
  };

  static Slot* Find(Bus& b, std::uint32_t id, bool extended) {
    for (std::size_t i = 0; i < b.used; ++i) {
      if (b.slots[i].rec.id == id && b.slots[i].rec.extended == extended) {
        return &b.slots[i];
      }
    }
    return nullptr;
  }

  std::array<Bus, kBuses> buses_{};
  std::uint32_t bad_bus_ = 0;
};

}  // namespace drivers
