#pragma once
#include <atomic>
#include <cstdint>

namespace planner {
namespace sas {
namespace parallel_SOC {

class IdAllocator {
    std::atomic<uint64_t> next_{0}; // 次の ID を保持する変数
public:
    // ID を取得する関数
    uint64_t alloc() noexcept {
        return next_.fetch_add(1, std::memory_order_relaxed); // 現在の next_{?} の値を取得して、1 だけインクリメントする
    }
    // ID を start の値にリセットする関数
    void reset(uint64_t start=0) noexcept { // start のデフォルト値は 0 とする
        next_.store(start, std::memory_order_relaxed);
    }
};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
