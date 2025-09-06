#include "heuristic.hpp"

namespace planner {

// ブラインド探索、常に 0 を返す
HeuristicFn make_blind() {
    return [](const StripsTask&, const StripsState&) -> double {
        return 0.0;
    };
}

// 違反ゴール数を数えるヒューリスティック関数
HeuristicFn make_goalcount() {
    return [](const StripsTask& st, const StripsState& s) -> int {
        int h = 0; // 違反ゴール数

        // ゴール条件で true なはずなのに false なものを数える
        for (int f : st.goal_pos) {
            if (!test_bit_inline(s.bits, f)) ++h;
        }

        // ゴール条件で false なはずなのに true のものを数える
        for (int f : st.goal_neg) {
            if (test_bit_inline(s.bits, f)) ++h;
        }

        return h;
    };
}

HeuristicFn make_weighted_goalcount(double w) {
    HeuristicFn base = make_goalcount();
    return [w, base](const StripsTask& st, const StripsState& s) -> double { // w と make_goalcount() をキャプチャし、その積を返す
        return w * base(st, s);
    };
}

// 逐次ヒューリスティック関数を追加していく

} // namespace planner
