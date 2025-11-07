#include "sas/parallel_SOC/expander.hpp"
#include <cassert>
#include <utility>

namespace planner {
 namespace sas {

extern bool violates_mutex(const Task& T, const State& s); // sas_reader.cpp 側

} // namespace sas
} // namespace planner

namespace planner {
namespace sas {
namespace parallel_SOC {

// prevail 条件を満たしているか確認する関数
static bool check_prevail(const Operator& op, const State& s) {
    for (auto [v,val] : op.prevail) {
        if (s[v] != val) { // prevail のドメイン値と、状態のドメイン値が異なる場合
            return false;
        }
    }
    return true;
}

// pre_post の Cond が一致するかどうか確かめる関数
static bool check_preconds(const Operator& op, const State& s) {
    for (auto& pp : op.pre_posts) {
        const auto& conds = std::get<0>(pp);
        int var = std::get<1>(pp);
        int pre = std::get<2>(pp);

        if (pre >= 0 && s[var] != pre) {
            return false;
        }

        for (auto [cv,val] : conds) {
            if (s[cv] != val) { // var_id と domain-value が一致しない場合
                return false;
            }
        }
    }
    return true;
}

// 状態の拡張を行う関数
void Expander::apply(const sas::Task& T, const sas::State& s, std::vector<Generated>& out) {
    out.clear(); // 一度 out をクリアする
    out.reserve(T.ops.size()/4 + 1); // 演算子のサイズの 1/4 だけサイズを確保する
    for (uint32_t oi=0; oi < T.ops.size(); ++oi) {
        const auto& op = T.ops[oi];

        if (!check_prevail(op, s)) { // prevail 条件のチェック
            continue;
        }
        if (!check_preconds(op, s)) { // pre_post の条件チェック
            continue;
        }

        sas::State ns = s;
        int add_cost = op.cost;
        // 効果適用（最後に書かれた post が勝つ想定）
        for (auto& pp : op.pre_posts) {
            ns[std::get<1>(pp)] = std::get<3>(pp);
        }

        // mutex まわりの判定は、一時コメントアウトしておく
        // if (violates_mutex(T, ns)) {
        //     continue;
        // }

        out.push_back(Generated{std::move(ns), oi, add_cost});
    }
}

// インプレース関数の実装
namespace {
    // 変更した要素の情報を補完するための struct
    struct DiffEntry{
        uint32_t v; // 変更した変数の id
        int old_val; // 元の domain-value
    };

    // 効果を適用する関数
    inline void apply_effects(const Operator& op, State& s, std::vector<DiffEntry>& diff) {
        diff.clear(); // 変更情報を積んだベクタをクリアする
        diff.reserve(op.pre_posts.size()); // 効果の数だけ、ベクタの容量を確保する

        for (auto& pp: op.pre_posts) {
            uint32_t v = std::get<1>(pp); // 変更する変数 ID
            int new_val = std::get<3>(pp);
            int old_val = s[v];
            if (old_val != new_val) {
                diff.push_back({v, old_val});
                s[v] = new_val;
            }
        }
    }

    // 効果の適用を戻す関数
    inline void revert_effects(State& s, const std::vector<DiffEntry>& diff) {
        for (const auto& d: diff) {
            s[d.v] = d.old_val; // 変更した変数の domain-value を元の値に戻す
        }
    }
} // namespace (anonymous)

void Expander::for_each_inplace(const sas::Task& T, sas::State& s, const std::function<void(uint32_t, int, const sas::State&)>& cb) {
    std::vector<DiffEntry> diff; // 変更情報を積んだベクタ

    for (uint32_t oi=0; oi < T.ops.size(); ++oi) {
        const auto& op = T.ops[oi];
        if (!check_prevail(op, s)) { // prevail 条件を満たしているか確認する
            continue;
        }
        if (!check_preconds(op, s)) { // pre_post の条件が一致するか確かめる
            continue;
        }

        // 差分適用
        apply_effects(op, s, diff);

        // 必要なら、ここで mutex チェックを行う
        // if (violates_mutex(T, s)) {
        //     revert_effects(s, diff);
        //     continue;
        // }

        // コールバック関数 (後継ノードは、今の s の参照で与えられる)
        cb(oi, op.cost, s);

        // 元に戻す
        revert_effects(s, diff);
    }

}


} // namespace parallel_SOC
} // namespace sas
} // namespace planner
