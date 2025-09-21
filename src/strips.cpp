#include "strips.hpp"
#include <unordered_set>
#include <sstream>
#include <queue>
#include <cstring>
#include <algorithm>
#include <cassert>

namespace planner {


// --- 内部ユーティリティ関数 ---

// nbits が 何ワード分かを求める関数 (いくつの 64bit 構造分か)
static inline int nwords(int nbits) {
    return (nbits + 63) >> 6; // 2^6 で割った余りを求める
}

// i 番目の真偽値を返す関数
static inline bool test_bit(const std::vector<std::uint64_t>& b, int i) {
    return (b[i >> 6] >> (i & 63)) & 1ull; // 2^6 で割った時の商が、ベクトルのどのブロックに入っているのかに対応し、
                                           // 2^6 で割った時の余りが、そのブロックのどこに入っているのかに対応する
}

// i 番目の真偽値を true にセットする関数
static inline void set_bit(std::vector<std::uint64_t>& b, int i) {
    b[i >> 6] |= (1ull << (i & 63)); // OR 演算
}

// i 番目の真偽値を false にセットする関数
static inline void clear_bit(std::vector<std::uint64_t>& b, int i) {
    b[i >> 6] &= ~(1ull << (i & 63)); // AND 演算
}

// GroundAtom の predicate の id と args の ids を文字列にして返す関数
static inline std::string key_of(const GroundAtom& ga) {
    std::ostringstream oss;
    oss << ga.pred << ":";
    for (int id : ga.args) oss << id << ",";
    return oss.str();
}

// 事実を StripsTask の 事実全集合に追加する関数 (id の 発行も含まれる)
static int intern_fact(StripsTask& st, const GroundAtom& ga, const GroundTask& gt) {
    const std::string key = key_of(ga); // grounatom の predicate id と args ids を取得する

    auto it = st.fid.find(key);
    if (it != st.fid.end()) return it->second; // key が 存在したら、その id を取得する

    // id の生成
    int id = static_cast<int>(st.facts.size());
    st.fid.emplace(key, id); // map に追加
    st.facts.push_back(ga); // Ground Atom の集合に追加
    st.fact_names.push_back(to_string(ga, gt)); // fact_names に 命題の名前を追加
    return id;
}

// --- 変換 ---

// GroundTask を STRIPSTask に変換する関数
StripsTask compile_to_strips(const GroundTask& gt) {
    StripsTask st; // STRIPS Task

    // 各 GroundAtom を 事実集合に入れるラムダ関数 (intern_fact 関数を流用した実装)
    auto collect = [&](const GroundAtom& ga) {
        (void) intern_fact(st, ga, gt);
    };

    // 事実の収集
    for (auto& f : gt.init_pos) collect(f);
    for (auto& f : gt.goal_pos) collect(f);
    for (auto& f : gt.goal_neg) collect(f);
    for (auto& a : gt.actions) {
        for (auto& x : a.pre_pos) collect(x);
        for (auto& x : a.pre_neg) collect(x);
        for (auto& x : a.eff_add) collect(x);
        for (auto& x : a.eff_del) collect(x);
    }

    // init
    st.init_true.reserve(gt.init_pos.size()); // サイズの確保
    for (auto& f : gt.init_pos) st.init_true.push_back(st.fid[key_of(f)]); // id を 順に入れていく

    // goal positive
    st.goal_pos.reserve(gt.goal_pos.size());
    for (auto& f : gt.goal_pos) st.goal_pos.push_back(st.fid[key_of(f)]);

    // goal negative
    st.goal_neg.reserve(gt.goal_neg.size());
    for (auto& f : gt.goal_neg) st.goal_neg.push_back(st.fid[key_of(f)]);

    // actions
    st.actions.reserve(gt.actions.size());

    bool any_costful = false; // domain にコストが付いているかどうか表すフラグ
    for (const auto& ga2 : gt.actions) {
        if (ga2.cost != 0.0) { any_costful = true; break; }
    }

    for (auto& ga : gt.actions) { // Ground Task に含まれる各 Action に対して
        StripsAction a; // STIRPS Action
        a.name = ga.name;
        a.cost = (any_costful ? ga.cost : 1.0); // ga.cost に 値がある場合以外は、 1.0 にデフォルトで設定しておく

        a.pre_pos.reserve(ga.pre_pos.size()); // サイズの確保
        for (auto& f : ga.pre_pos) a.pre_pos.push_back(st.fid[key_of(f)]); // id を順に入れていく

        a.pre_neg.reserve(ga.pre_neg.size());
        for (auto& f : ga.pre_neg) a.pre_neg.push_back(st.fid[key_of(f)]);

        a.add.reserve(ga.eff_add.size());
        for (auto& f : ga.eff_add) a.add.push_back(st.fid[key_of(f)]);

        a.del.reserve(ga.eff_del.size());
        for (auto& f : ga.eff_del) a.del.push_back(st.fid[key_of(f)]);

        st.actions.push_back(std::move(a)); // move を用いて、STRIPS Task の aactions に追加する
    }

    return st;
}

// --- 基本演算 ---

// 初期状態の真偽ベクトルを作成する関数
StripsState make_init_state(const StripsTask& st) {
    StripsState s;
    s.bits.resize(nwords(st.num_facts()));
    std::fill(s.bits.begin(), s.bits.end(), 0ull); // 必要語数だけ 0 にする
    for (int f : st.init_true) set_bit(s.bits, f); // true の 事実を 1 にする
    return s;
}

// 提供可能か判定する関数
bool is_applicable(const StripsTask& st, const StripsState& s, const StripsAction& a) {
    // pre_pos ⊆ s かどうか判定する
    for (int f : a.pre_pos) {
        if (!test_bit(s.bits, f)) return false;
    }
    // pre_neg ∩ s = ∅ かどうか判定する
    for (int f : a.pre_neg) {
        if (test_bit(s.bits, f)) return false; // もし negative な fact が true だった場合
    }
    return true;
}

// 適用を行う関数
void apply(const StripsTask& st, const StripsState& s, const StripsAction& a, StripsState& out) {
    (void) st;
    out = s;
    for (int f : a.del) clear_bit(out.bits, f); // 削除効果が与えられる事実は、 false にする
    for (int f : a.add) set_bit(out.bits, f); // 追加効果が与えられる事実は、 true にする
}

// Goal 状態か判定する関数
bool is_goal(const StripsTask& st, const StripsState& s) {
    for (int f : st.goal_pos) { // goal 状態で true の箇所が false でないかチェックする
        if (!test_bit(s.bits, f)) return false;
    }
    for (int f : st.goal_neg) { // goal 状態で false の箇所が true でないかチェックする
        if (test_bit(s.bits, f)) return false;
    }
    return true;
}

// --- ユーティリティ関数 ---

// 状態の真偽ベクトルが一致しているかどうか判定するファンクタ
bool operator==(const StripsState& a, const StripsState& b) {
    return a.bits == b.bits;
}

// Strips State を ハッシュ値に変換する関数
std::size_t StripsStateHash::operator()(const StripsState& s) const {
    // 簡易混合（64bit の xor 折り畳み）
    std::uint64_t h = 0x9e3779b97f4a7c15ull; // Knuth の推奨値
    for (auto w : s.bits) {
        h ^= w + 0x9e3779b97f4a7c15ull + (h<<6) + (h>>2); // Boost の混合パターン
    }
    return static_cast<std::size_t>(h);
}

// 事実を文字列にする関数
std::string fact_to_string(const StripsTask& st, int fid, const GroundTask& gt) {
    (void) gt;
    if (fid < 0 || fid >= (int)st.fact_names.size()) return "<bad-fid>"; // id が範囲外にある場合
    return st.fact_names[fid]; // fact_names を返す
}

// 初期状態の真偽ベクトルを文字列にする関数
std::string state_to_string(const StripsTask& st, const StripsState& s, const GroundTask& gt, int max_items) {
    std::ostringstream oss;
    oss << "{";
    int shown = 0; // 表示数
    for (int i = 0, n = st.num_facts(); i < n; ++i) {
        if (test_bit(s.bits, i)) {
            if (shown++) oss << ", ";
            oss << st.fact_names[i];
            if (shown >= max_items && shown < n) { // 最大表示数を超えた場合
                 oss << ", ..."; break;
            }
        }
    }
    oss << "}";
    return oss.str();
}

// StripsState も int 型のように出力できるようにするための、演算子オーバーロード
std::ostream& operator<<(std::ostream& os, const StripsState& s) {
    os << "StripsState[" << s.bits.size() << " words]";
    return os;
}

// --- 差分適用用の関数群 ---
// State のビット反転を適用し、その id を Undo に積む関数
void apply_inplace(const StripsTask& st, const StripsAction& a, StripsState& s, Undo& u) {
    (void) st;
    for (int f : a.del) {
        if (test_bit(s.bits, f)) {
            clear_bit(s.bits, f);
            u.flipped.push_back(f);
        }
    }

    for (int f : a.add) {
        if (!test_bit(s.bits, f)) {
            set_bit(s.bits, f);
            u.flipped.push_back(f);
        }
    }
}

// Undo スタックを mark まで巻き戻す関数
void undo_to(StripsState& s, Undo& u, std::size_t mark) {
    assert(mark <= u.flipped.size());
    for (std::size_t i = u.flipped.size(); i-- > mark;) {
        int f = u.flipped[i];
        s.bits[f >> 6] ^= (1ull << (f & 63)); // XOR 演算で、1 -> 0, 0 -> 1 として元に戻す
    }
}

} // namespace planner
