#pragma once
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include <optional>
#include <unordered_map>
#include "sas/parallel_SOC/state_hasher.hpp"
#include "sas/parallel_SOC/node.hpp"

namespace planner {
namespace sas {

struct Task;

} // namespace sas
} // namespace planner


namespace planner {
namespace sas {
namespace parallel_SOC {

// Closed に格納する最小情報
struct ClosedEntry {
    int best_g = INT32_MAX; // 最良の g-value
    uint64_t node_id = UINT64_MAX; // node-id
};

class ClosedTable {
    using Map = std::unordered_map<sas::State, ClosedEntry, StateHash, StateEq>; // 状態と closed entry のハッシュマップ、ハッシュには StateHash, 等価演算子には StateEq を用いる
    const uint32_t stripes_; // 分割数
    std::vector<Map> maps_; // 部分ハッシュマップ数
    std::vector<std::shared_mutex> locks_; // 各部分マップに対応するロック

    // state から何番目のハッシュマップに入れるのか求める関数
    inline uint32_t stripe_of(const sas::State& s) const {
        return static_cast<uint32_t>(StateHash{}(s) % stripes_); // 分割数で割った時の余りを用いる
    }

public:
    explicit ClosedTable(uint32_t stripes = 1024): stripes_(stripes), maps_(stripes), locks_(stripes) {} // コンストラクタ、stripes で各変数を初期化する

    // 既出で g-value の改善がなければ true, 新規または g-value の改善がある場合は false
    bool prune_or_update(const sas::State& s, int g, uint64_t node_id) {
        const uint32_t tmp = stripe_of(s); // index を求める
        std::unique_lock lk(locks_[tmp]); // lock を掛ける

        auto& mp = maps_[tmp];
        auto it = mp.find(s);

        if (it == mp.end()) { // もし、状態が見つからなかった場合 (新規ノードの場合)
            mp.emplace(s, ClosedEntry{g, node_id}); // ハッシュマップに追加する
            return false;
        }

        if (g >= it->second.best_g) { // g-value が悪化した場合
            return true;
        }

        // 既出ノードで、g-value が改善した場合
        it->second.best_g = g; // g-value の更新
        it->second.node_id = node_id; // node-id の更新
        return false;
    }

    // closed list に登録されている状態から、ClosedEntry を取り出す関数
    std::optional<ClosedEntry> get(const sas::State& s) const {
        const uint32_t tmp = stripe_of(s);
        std::shared_lock lk(locks_[tmp]);
        auto& mp = maps_[tmp];
        auto it = mp.find(s);
        if (it == mp.end()) { // 見つからなかった場合
            return std::nullopt;
        }
        return it->second;
    }
};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
