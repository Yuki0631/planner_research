#pragma once
#include <cstdint>
#include <vector>
#include <limits>

namespace planner {
namespace sas {
// 状態を表すベクタ
    using State = std::vector<int>;

} // namespace sas
} // namespace planner

namespace planner {
namespace sas {
namespace parallel_SOC {

struct Node {
    uint64_t id; // 状態 ID
    int g; // g-value
    int h; // h-value
    uint32_t op_id; // 適用した演算子の ID
    uint64_t parent; // 親状態 ID
    // 軽量化のために State は保持しない

    inline int f() const { // f-value を計算する関数
        return g + h;
    }
};

struct NodeLess {
    bool operator()(const Node& a, const Node& b) const { // 比較演算子
        if (a.f() != b.f()) { // f-value が小さい方を優先する
            return a.f() > b.f();
        }
        if (a.h != b.h) { // タイブレークは h-value が小さい方を優先する
            return a.h > b.h;
        }
        // f-value も h-value も同じ場合は、ID の大きさによって決定する
        return a.id > b.id;
    }
};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
