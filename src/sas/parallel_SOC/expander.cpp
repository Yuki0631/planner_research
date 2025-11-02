#include "sas/parallel_SOC/expander.hpp"
#include <cassert>

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
        for (auto [v,val] : std::get<0>(pp)) {
            if (s[v] != val) { // var_id と domain-value が一致しない場合
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

// インプレース関数などは後で実施する

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
