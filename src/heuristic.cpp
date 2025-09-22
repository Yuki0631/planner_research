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
        // 64bit ワード列を積んだベクトル
        struct Bits {
            std::vector<std::uint64_t> v;
        };
        // state の状態を保存しておくキャッシュ
        struct CacheVal {
            Bits pos, neg;
            int words = 0;
        };

        static CacheVal cache;
        static bool initialized = false;

        // 初期化
        if (!initialized) {
            cache.words = (st.num_facts() + 63) >> 6; // ワード数の計算
            cache.pos.v.assign(cache.words, 0ull);
            cache.neg.v.assign(cache.words, 0ull);
            for (int f : st.goal_pos) cache.pos.v[f >> 6] |= (1ull << (f & 63)); // positive な命題の bit を 1 にする
            for (int f : st.goal_neg) cache.neg.v[f >> 6] |= (1ull << (f & 63)); // negative な命題の bit を 1 にする
            initialized = true;
        }

        const auto& pos = cache.pos.v;
        const auto& neg = cache.neg.v;
        const int W = cache.words;

        int h = 0;
        for (int i = 0; i < W; ++i) {
            std::uint64_t sb = (i < (int)s.bits.size()) ? s.bits[i] : 0ull;
            std::uint64_t v1 = pos[i] & ~sb; // 本来は 1 のはずだが 0 になっている bit の個数
            std::uint64_t v2 = neg[i] &  sb; // 本来は 0 のはずだが 1 になっている bit の個数
            h += __builtin_popcountll(v1);
            h += __builtin_popcountll(v2);
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
