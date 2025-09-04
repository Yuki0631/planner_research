#pragma once
#include "strips.hpp"
#include <functional>
#include <cstdint>
#include <vector>

namespace planner {

// ヒューリスティック関数型
using HeuristicFn = std::function<double(const StripsTask&, const StripsState&)>;

// --- ヒューリスティック関数一覧 ---
// 常に 0 を返す（ブラインド探索）
HeuristicFn make_blind();

// 違反ゴール数（ゴール条件で true のはずが false な事実 + ゴール条件で false なはずが true な事実）
HeuristicFn make_goalcount();

// 係数付き違反ゴール数
HeuristicFn make_weighted_goalcount(double w);

// 逐次ヒューリスティック関数を追加していく

// --- ユーティリティ関数 ---
// ビットチェックを行う
inline bool test_bit_inline(const std::vector<std::uint64_t>& b, int i) {
    return (b[static_cast<std::size_t>(i) >> 6] >> (i & 63)) & 1ull; // i 番目の真偽値を返す関数
}

} // namespace planner
