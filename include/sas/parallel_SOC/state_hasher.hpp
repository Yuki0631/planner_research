#pragma once
#include <vector>
#include <cstddef>
#include <cstdint>

namespace planner{ 
namespace sas {

     using State = std::vector<int>; // state

} // namespace sas
} // namespace planner


namespace planner { 
namespace sas {
namespace parallel_SOC {

struct StateEq {
    bool operator()(const sas::State& a, const sas::State& b) const noexcept {
        return a == b;
    }
};

// Fowler-Noll-Vo (FNV) 方式を用いた 64-bit 乗算ハッシュ
struct StateHash {
    size_t operator()(const sas::State& s) const noexcept {
        uint64_t x = 1469598103934665603ull; // FNV オフセット基数
        for (int v : s) {
            uint64_t y = static_cast<uint64_t>(v) ^ (static_cast<uint64_t>(v) << 32);
            x ^= y;
            x *= 1099511628211ull; // FNV プライム
        }
        return static_cast<size_t>(x);
    }
};

}
}
} // namespace
