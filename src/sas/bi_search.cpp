#include "sas/bi_search.hpp"
#include "bucket_pq.hpp"
#include <robin_hood.h>

#include <unordered_map>
#include <queue>
#include <limits>
#include <cmath>
#include <cassert>
#include <iostream>

// コンパイラに対するヒントのためのマクロ変数
#define likely(x)   __builtin_expect(!!(x), 1) // よく起こりやすい条件分岐
#define unlikely(x) __builtin_expect(!!(x), 0) // あまり起きない条件分岐

namespace planner { namespace sas {

// --- 必要なデータ構造の読み込み ---
using Task     = planner::sas::Task;
using Operator = planner::sas::Operator;

// --- sas_search.cpp と同様に、mutex チェックの判定のシステムを設計する ---
extern int g_mutex_mode;
enum { MUTEX_AUTO=0, MUTEX_ON=1, MUTEX_OFF=2 };

static inline bool should_check_mutex_runtime(const Task& T) {
    using namespace planner::sas;
    if (g_mutex_mode == MUTEX_OFF) {
        return false;
    }
    if (g_mutex_mode == MUTEX_ON)  {
        return true;
    }
    return !T.mutexes.empty();
}

// --- unordered_map 用のハッシュ関数と等価比較関数の設計 ---
// 状態 (vector<int>) のハッシュ関数 (FNV-1a)
struct FwdHash {
    std::size_t operator()(const State& v) const noexcept {
        std::size_t h = 1469598103934665603ull;
        for (int x : v) {
            std::size_t y = static_cast<std::size_t>(x) + 0x9e3779b97f4a7c15ull;
            h ^= y;
            h *= 1099511628211ull;
        }
        return h;
    }
};

// 状態 (vector<int>) の等価比較を行う関数オブジェクト
struct FwdEq {
    bool operator()(const State& a, const State& b) const noexcept {
        return a == b;
    }
};

// --- unknown を扱う後ろ向き探索用の state を表すデータ構造 ---
using RegState = std::vector<int>;

// --- Regstate 用のハッシュ関数と等価比較関数の設計 (unordered_map 用) ---
// ハッシュ関数 (FNV-1a, unknown の -1 も考慮した設計)
struct RegHash {
    std::size_t operator()(const RegState& v) const noexcept {
        std::size_t h = 1469598103934665603ull;
        for (int x : v) {
            std::size_t y = static_cast<std::size_t>(x + 1) + 0x9e3779b97f4a7c15ull;
            h ^= y;
            h *= 1099511628211ull;
        }
        return h;
    }
};

// 等価関数
struct RegEq {
    bool operator()(const RegState& a, const RegState& b) const noexcept {
        return a == b; // 現在は = で判定しているが、包含関係を含めて考えてみてもいいかもしれない
    }
};

// --- ユーティリティ関数 ---
// ゴール判定（前向き）
static inline bool is_goal(const Task& T, const State& s) {
    for (auto [v,val] : T.goal) {
        if (likely(s[v] != val)) {
            return false;
        }
    }
    return true;
}

// regression search の initial state を作成する関数
static RegState make_goal_reg_state(const Task& T) {
    const int nvars = static_cast<int>(T.vars.size());
    RegState g(nvars, -1); // 最初にすべて unknown 状態のベクタを用意する

    for (auto [v,val] : T.goal) {
        g[v] = val;
    }

    return g;
}

// forward state が regression state の条件を満たしているか判定する関数
static bool forward_state_satisfies_reg(const State& s, const RegState& reg) {
    const int n = static_cast<int>(reg.size());

    assert(static_cast<int>(s.size()) >= n);

    for (int v=0; v<n; ++v) {
        if (reg[v] >= 0) {
            if (likely(s[v] != reg[v])) { // regression state が具体的な値を持っている (0 以上) 変数値は一致する必要がある
                return false;
            }
        }
    }

    return true;
}

// a ⊑ b (b subsumes a) の判定、a が具体的な値を持つ部分は、b も具体的な値を持ちその値が同一であるかの判定
inline bool subsume(const RegState& a, const RegState& b) {
    assert(a.size() == b.size());

    const int n = static_cast<int>(a.size());

    for (int v = 0; v < n; ++v) {
        if (a[v] >= 0) {
            if (likely(a[v] != b[v])) { // regstate a が具体的な値を持っている (0 以上) の変数値は一致する必要がある
                return false;
            }
        }
    }
    return true;
}

// 2 つの regstate が同一の状態か判定する (お互いにお互いを包含 (subsume) しているかどうか) 
inline bool equivalent(const RegState& a, const RegState& b) {
    assert(a.size() == b.size());
    return subsume(a, b) && subsume(b, a);
}

// unknown は除外し、すべての変数の値が一致するかで、前向きと後ろ向きの出会いの判定を行う
inline bool meet_forward_backward(const State& s, const RegState& reg) {
    const int n = static_cast<int>(reg.size());
    assert(static_cast<int>(s.size()) >= n);
    for (int v = 0; v < n; ++v) {
        if (likely(s[v] != reg[v])) {
            return false;
        }
    }
    return true;
}

// 前向き探索における演算子の適用の可否を判定する関数 (sas_search.cpp の is_applicable と同等)
static inline bool is_applicable_forward(const Task& T, const State& s, const Operator& op) {
    // prevail 条件
    for (auto [v,val] : op.prevail) {
        if (s[v] != val) {
            return false;
        }
    }

    // 条件付き効果の条件
    for (const auto& pp : op.pre_posts) {
        const auto& conds = std::get<0>(pp);
        for (auto [cv,cval] : conds) {
            if (s[cv] != cval) {
                return false;
            }
        }
    }

    // pre_post の pre チェック（-1 は don't care）
    for (const auto& pp : op.pre_posts) {
        int var = std::get<1>(pp);
        int pre = std::get<2>(pp);
        if (pre >= 0 && s[var] != pre) {
            return false;
        }
    }
    return true;
}

// 1ステップの前向き遷移を行うための関数群 (inplace, undo, sas_search と同等)
using Undo = std::vector<std::pair<int,int>>; // (var, old_value)

static inline std::size_t undo_mark(const Undo& u) {
    return u.size();
}

static inline void undo_to(State& s, Undo& u, std::size_t mark) {
    for (std::size_t i = u.size(); i-- > mark; ) {
        const auto [var, oldv] = u[i];
        s[var] = oldv;
    }

    u.resize(mark);
}

static inline void apply_inplace(const Task&, const Operator& op, State& s, Undo& u) {
    for (const auto& pp : op.pre_posts) {
        int var  = std::get<1>(pp);
        int post = std::get<3>(pp);

        if (s[var] != post) {
            u.emplace_back(var, s[var]);
            s[var] = post;
        }
    }
}

// RAII で undo を自動巻き戻すための structure
struct UndoGuard {
    State& work;
    Undo& undo;
    std::size_t mark;

    ~UndoGuard() {
        undo_to(work, undo, mark);
    }
};

// 前向きノードからプランを復元する関数（sas_search.cpp の extract_plan と同等）
static std::vector<uint32_t> extract_plan_forward(const std::vector<Node>& nodes, int goal_id) {
    std::vector<uint32_t> acts;

    for (int v = goal_id; v >= 0 && nodes[v].parent >= 0; v = nodes[v].parent) {
        acts.push_back(nodes[v].act_id);
    }
    std::reverse(acts.begin(), acts.end());
    return acts;
}

// プランコストの評価を行う関数（sas_search.cpp と同等）
static double eval_plan_cost_local(const Task& T, const std::vector<uint32_t>& plan) {
    double c = 0.0;

    for (uint32_t a : plan) {
        c += T.ops[a].cost;
    }

    return c;
}

// --- regression による状態集合の展開 ---
// mutex, e-deletion, invariant や、インプレース化は未実装
// regression search 専用のヒューリスティックも未実装
static bool regress_state(const Task& T, const Operator& op, const RegState& reg_state, RegState& prev_out) {
    const int nvars = static_cast<int>(T.vars.size()); // 変数の数
    assert(static_cast<int>(reg_state.size()) == nvars); // 変数の数と状態の変数の数が一致するかどうか

    prev_out = reg_state; // 後ろ向き探索で展開した (1 個前の) 後の状態、最初は展開前の状態で初期化する

    // 効果の整合性と「少なくとも一つのゴールをサポートしているか？」の判定
    bool relevant = false;

    for (const auto& pp : op.pre_posts) {
        const auto& conds = std::get<0>(pp);
        (void)conds; // conds のチェックは後ほど実施する
        int var  = std::get<1>(pp);
        int pre  = std::get<2>(pp);
        int post = std::get<3>(pp);

        int gv = reg_state[var]; // 展開する状態の値

        if (gv >= 0) { // unknown (-1) 出ない場合
            if (gv != post) { // 演算子の効果と現在の変数の値が異なる場合
                return false;
            } else {
                relevant = true;
            }
        }

        (void)pre;
    }

    if (!relevant) { // 演算子の効果は現在の Regstate と矛盾しないが、一つも効果を追加しない場合
        return false;
    }

    // 整合性のチェック、演算子の適用前に満たされるべき条件を prev_out に反映させる
    for (auto [v,val] : op.prevail) {
        // prevail 条件は before の世界でも after の世界でも不変
        if (reg_state[v] >= 0 && reg_state[v] != val) { // after の世界で矛盾する場合
            return false;
        }
        if (prev_out[v] >= 0 && prev_out[v] != val) { // before の世界で矛盾する場合
            return false;
        }

        prev_out[v] = val; // unknown である場合は、before の世界の変数の値を prevail 条件に合わせる
    }

    // 条件付き効果の判定と反映
    for (const auto& pp : op.pre_posts) {
        const auto& conds = std::get<0>(pp);
        int var  = std::get<1>(pp);
        int pre  = std::get<2>(pp);
        (void)var; (void)pre;

        for (auto [cv,cval] : conds) {
            // FD の変換器だと、conds に現れる変数の値は効果に関して不変
            if (reg_state[cv] >= 0 && reg_state[cv] != cval) {
                return false;
            }
            if (prev_out[cv] >= 0 && prev_out[cv] != cval) {
                return false;
            }

            prev_out[cv] = cval; // unknown である場合は、before の世界の変数の値を conds 条件に合わせる
        }
    }

    // pre_post の pre (var==pre) を before 側に反映する
    for (const auto& pp : op.pre_posts) {
        int var  = std::get<1>(pp);
        int pre  = std::get<2>(pp);
        int post = std::get<3>(pp);

        if (pre >= 0) {
            int before_val = prev_out[var];

            if (before_val >= 0 && before_val != pre) {
                if (!(reg_state[var] >= 0 && before_val == reg_state[var])) { // reg_state から効果の値をコピーしたが異なる場合
                    return false;
                }
            }

            prev_out[var] = pre;
        }

        (void)post;
    }

    // before 側の演算子の効果の delete
    // for (const auto& pp : op.pre_posts) {
    //     int var  = std::get<1>(pp);
    //     int post = std::get<3>(pp);
    // 
    //     if (reg_state[var] == post) {
    //         // この var=post は add(a) に含まれており、before 側では上で pre に置き換えているので、特別な操作は必要ない
    //     }
    // }

    // domain-value が定義内にあるか判定する (コメントアウトしてもよい)
    for (int v=0; v<nvars; ++v) {
        if (prev_out[v] < 0) { // ドメイン値が -1
            continue;
        }

        const int dom = T.vars[v].domain;

        if (prev_out[v] >= dom) { // domain-value がドメインの個数以上の場合 (定義外の場合)
            std::cerr << "invalid domain value" << "\n";
            return false;
        }
    }

    return true;
}

// コストの整数判定と rounding 関数（sas_search.cpp と同等）
static bool all_action_costs_are_integers(const Task& T, double eps = 1e-12) {
    for (const auto& op : T.ops) {
        if (!std::isfinite(op.cost)) {
            return false;
        }

        double nearest = std::round(op.cost);

        if (std::fabs(op.cost - nearest) > eps) {
            return false;
        }
    }

    return true;
}

static int rounding(double v) {
    long long k = std::llround(v);
    if (k < 0) {
        throw std::runtime_error("negative value not supported");
    }
    return static_cast<int>(k);
}



// --- Bidirectional search (前向き A* + 後ろ向き UCS) ---
// 後ろ向きノード
struct BackNode {
    RegState s;
    int parent; // 親 BackNode の ID
    int act_id; // regression に使った演算子 ID
};

Result bidir_astar(const Task& T, HeuristicFn h, bool h_is_integer, const Params& p) {
    Result R;
    R.solved = false;
    R.plan_cost = 0.0;
    R.plan.clear();
    R.nodes.clear();
    R.stats = Stats{};
    R.meet = false;
    R.reg_plan_len = 0;

    const int nvars = static_cast<int>(T.vars.size()); // 変数の数

    // forward search の初期化
    State s0(nvars);
    for (int v=0; v<nvars; ++v) {
        s0[v] = T.init[v];
    }

    // backward search の初期化
    RegState g0 = make_goal_reg_state(T);

    // 初期状態がすでにゴールを満たしている場合の判定
    if (forward_state_satisfies_reg(s0, g0)) {
        R.solved = true;
        R.plan_cost = 0.0;
        R.plan.clear();
        R.nodes.push_back(Node{ s0, -1, -1 });
        R.meet = true;
        R.reg_plan_len = 0;
        return R;
    }

    // 初期ノードを探索ノード管理用のベクタに積む (forward search)
    R.nodes.push_back(Node{ s0, -1, -1 });

    // 前向きノードの状態と ID を保管するハッシュマップ
#if defined(USE_ROBIN_HOOD)
    robin_hood::unordered_map<State, int, FwdHash, FwdEq> index_fwd;
#else
    std::unordered_map<State, int, VecHash, VecEq> index_fwd;
    index_fwd.max_load_factor(0.5f);
#endif
    index_fwd.reserve(1<<15);
    index_fwd.emplace(R.nodes[0].s, 0);

    // regression search 用の探索ノード管理用のベクタの設計と初期ノードの登録
    std::vector<BackNode> back_nodes;
    back_nodes.push_back(BackNode{ g0, -1, -1 }); // id=0: goal-partial

    // 前向きノードの状態と ID を保管するハッシュマップ
#if defined(USE_ROBIN_HOOD)
    robin_hood::unordered_map<RegState, int, RegHash, RegEq> index_bwd;
#else
    std::unordered_map<RegState, int, RegHash, RegEq> index_bwd;
    index_bwd.max_load_factor(0.5f);
#endif
    index_bwd.reserve(1<<15);
    index_bwd.emplace(back_nodes[0].s, 0);


    // forwarding と regression search の meeting 情報
    bool   have_meeting = false;
    double best_cost = std::numeric_limits<double>::infinity();
    int best_f = -1;
    int best_b = -1;

    // mutex 情報を使用するかの切り替え (sas_search と同等)
    {
        const bool do_mutex = should_check_mutex_runtime(T);
        if (do_mutex) {
            std::cout << "Mutex check: ON\n";
        } else {
            std::cout << "Mutex check: OFF\n";
        }
    }

    // 整数モードかの判定 (sas_search と同等)
    const bool integer_mode = (all_action_costs_are_integers(T) && h_is_integer);

    if (likely(integer_mode)) {
        std::cout << "Note: all action costs are integers; using integer bidirectional A* + BucketPQ.\n";

        // クローズドリスト用のデータ構造
        struct MetaF { int g; int h; bool closed; };
        struct MetaB { int g; bool closed; };

        // クローズリストの初期化
        std::vector<MetaF> meta_fwd(1, MetaF{0, 0, false});
        std::vector<MetaB> meta_bwd(1, MetaB{0, false});

        // オープンリスト (bucket_pq.hpp を使用)
        TwoLevelBucketPQ open_fwd;
        TwoLevelBucketPQ open_bwd;

        // 初期ノードのヒューリスティック
        const int h0 = rounding(h(T, s0));
        ++R.stats.evaluated;
        meta_fwd[0] = MetaF{0, h0, false};
        open_fwd.insert(0, pack_fh_asc(h0, h0)); // f = g + h = h0, h = h0

        // 後ろ向き側は UCS（h=0）で管理する
        meta_bwd[0] = MetaB{0, false};
        open_bwd.insert(0, pack_fh_asc(0, 0)); // f=g, h=0
    
        // work state (インプレース化)
        State work_f = s0;
        Undo  undo_f;
        RegState work_b;

        bool expand_forward_turn = true; // forward/backward どちらの方向を展開するのか表すフラグ

        while (!open_fwd.empty() || !open_bwd.empty()) {
            if (unlikely(planner::sas::time_exceeded_cpu())) {
                std::cerr << "error: CPU time limit exceeded (" << planner::sas::g_cpu_limit_sec << " sec)\n";
                std::exit(101);
            }

            if (unlikely(R.stats.expanded > p.max_expansions)) {
                break;
            }

            bool did_expand = false; // ノードを展開したか表す変数

            // --- forward search ---
            if (expand_forward_turn) {
                if (likely(!open_fwd.empty())) {
                    auto [id32, key] = open_fwd.extract_min();
                    const int u = static_cast<int>(id32);
                    const int fu = unpack_f(key);
                    const int hu = unpack_h(key);

                    if (u < 0 || u >= (int)meta_fwd.size()) { // ノード ID が定義の外にある場合
                        continue; // スキップして次の反復に移る
                    }

                    if (meta_fwd[u].closed) { // オープンリストから取り出したノードがクローズドリストに含まれる場合
                        continue;
                    }

                    const int fu_now = meta_fwd[u].g + meta_fwd[u].h;

                    if (fu != fu_now || hu != meta_fwd[u].h) { // 取り出したノードが古いノードであった場合
                        continue;
                    }

                    const State& su = R.nodes[u].s; // 現在のノード

                    // forward search でゴールにたどり着いてしまった場合
                    if (is_goal(T, su)) {
                        R.solved = true;
                        R.plan = extract_plan_forward(R.nodes, u);
                        R.plan_cost = eval_plan_cost(T, R.plan);
                        R.meet = false;
                        R.reg_plan_len = 0;
                        return R;
                    }

                    meta_fwd[u].closed = true;
                    ++R.stats.expanded;
                    did_expand = true;

                    // ノードの展開を行う
                    for (int a=0; a < (int)T.ops.size(); ++a) {
                        const auto& op = T.ops[a];

                        if (likely(!is_applicable_forward(T, su, op))) { // アクションが適用不可能な場合
                            continue;
                        }

                        // インプレース化のための処理 (Undo pre_process)
                        work_f = su;
                        undo_f.clear();
                        const std::size_t mark = undo_mark(undo_f);
                        UndoGuard ug{work_f, undo_f, mark};

                        apply_inplace(T, op, work_f, undo_f); // アクションを work_f に適用する
                        ++R.stats.generated;

                        // mutex チェック（前向き側）
                        if (should_check_mutex_runtime(T)) {
                            if (planner::sas::violates_mutex(T, work_f)) {
                                continue;
                            }
                        }

                        const int step_cost = rounding(op.cost);
                        const int tentative_g = meta_fwd[u].g + step_cost;

                        auto it = index_fwd.find(work_f); // 展開後の state の ID の確認
                        int v; // state の新規 ID または既存 ID

                        if (it == index_fwd.end()) { // 新規ノードの場合
                            v = (int)R.nodes.size(); // ID の割り当て

                            R.nodes.push_back(Node{work_f, u, a}); // ノードの登録を行う
                            index_fwd.emplace(R.nodes[v].s, v); // ノードと ID のハッシュマップへの登録も行う

                            if ((int)meta_fwd.size() <= v) { // ID がクローズドリストのサイズよりも大きい場合
                                meta_fwd.resize(v+1, MetaF{0,0,false});
                            }

                            const int hv = rounding(h(T, R.nodes[v].s));
                            ++R.stats.evaluated;

                            // クローズリストへの登録
                            meta_fwd[v].g = tentative_g;
                            meta_fwd[v].h = hv;
                            meta_fwd[v].closed = false; // ノードを取り出すまではクローズリストに入れない

                            open_fwd.insert(static_cast<BucketPQ::Value>(v), pack_fh_asc(tentative_g + hv, hv)); // オープンリストへの挿入
                        } else { // 新規ノードでない場合
                            v = it->second; // 既存 ID の取り出し

                            if (tentative_g < meta_fwd[v].g) { // g-value が更新される場合
                                meta_fwd[v].g = tentative_g;
                                R.nodes[v].parent = u;
                                R.nodes[v].act_id = a;

                                const int hv = rounding(h(T, R.nodes[v].s));
                                ++R.stats.evaluated;
                                meta_fwd[v].h = hv;

                                const UKey new_key = pack_fh_asc(meta_fwd[v].g + meta_fwd[v].h, meta_fwd[v].h);

                                if (likely(meta_fwd[v].closed)) { // クローズドリストに含まれる場合
                                    if (!p.reopen_closed) { // オープンリストからの再オープンが禁じられている場合
                                        ++R.stats.duplicates;
                                        continue;
                                    }
                                    
                                    meta_fwd[v].closed = false; // クローズドリストから外す
                                    open_fwd.insert(static_cast<BucketPQ::Value>(v), new_key); // 新たにオープンリストに挿入する
                                } else { // クローズドリストに含まれない場合
                                    auto val = static_cast<BucketPQ::Value>(v);

                                    if (open_fwd.contains(val)) { // オープンリストにノードがある場合
                                        const auto cur_key = open_fwd.key_of(val);
                                        // オープンリスト内にあるノードの値の更新
                                        if (new_key < cur_key) {
                                            open_fwd.decrease_key(val, new_key);
                                        } else if (new_key > cur_key) {
                                            open_fwd.increase_key(val, new_key);
                                        }
                                    } else { // 何らかの理由で open にノードがない場合
                                        open_fwd.insert(val, new_key); // 再挿入を行う
                                    }
                                }
                            } else { // g-value が更新されない場合
                                ++R.stats.duplicates;
                                continue; // 単純にスキップする
                            }
                        }

                        // meeting 判定（新しいまたは改善された forward 状態 v に対して backward 側の全ノードをチェックする）
                        const State& sv = R.nodes[v].s; // state
                        const int gv = meta_fwd[v].g; // g-value

                        for (int b_id=0; b_id < (int)back_nodes.size(); ++b_id) {
                            if (!forward_state_satisfies_reg(sv, back_nodes[b_id].s)) { // state v が b_id の後ろ向き探索の state を subsume しない場合
                                continue;
                            }

                            const int gb = (b_id < (int)meta_bwd.size()) ? meta_bwd[b_id].g : 0; // backward 側のコスト
                            const double cand = static_cast<double>(gv) + static_cast<double>(gb); // forward 側と backward 側のっコストの合計

                            if (cand + 1e-12 < best_cost) { // meeting したノードの中で、一番短いコストを持つものを記録する
                                best_cost = cand;
                                have_meeting = true;
                                best_f = v;
                                best_b = b_id;
                            }
                        }
                    }
                }
            } else if (!expand_forward_turn && !open_bwd.empty()) {

                // regression search
                auto [id32, key] = open_bwd.extract_min();
                const int u = static_cast<int>(id32);
                const int fu = unpack_f(key); // 現段階では、f=g
                (void)fu;

                if (u < 0 || u >= (int)meta_bwd.size()) { // 取り出したノード ID が定義の外にある場合
                    continue;
                }

                if (meta_bwd[u].closed) { // クローズドリストに含まれている場合
                    continue;
                }

                const RegState& su = back_nodes[u].s; // 取り出したノードの state

                // regression search で初期状態にたどり着いてしまった場合
                if (forward_state_satisfies_reg(s0, su)) {
                    R.solved = true;
                    
                    for (int id = u; id >= 0 && back_nodes[id].parent >=0; id = back_nodes[id].parent) {
                        R.plan.push_back(back_nodes[id].act_id);
                    }

                    R.plan_cost = eval_plan_cost(T, R.plan);
                    R.meet = false;
                    R.reg_plan_len = static_cast<uint8_t>(R.plan.size());
                    return R;
                }

                meta_bwd[u].closed = true;
                ++R.stats.expanded;
                did_expand = true;

                for (int a=0; a < (int)T.ops.size(); ++a) {
                    const auto& op = T.ops[a];

                    RegState prev;

                    if (!regress_state(T, op, su, prev)) { // regression ができない場合
                        continue;
                    }

                    ++R.stats.generated;

                    const int step_cost = rounding(op.cost);
                    const int tentative_g = meta_bwd[u].g + step_cost;

                    auto it = index_bwd.find(prev); // regresision 適用前の state をハッシュマップから探す
                    int v;

                    if (it == index_bwd.end()) { // 新規ノードの場合
                        v = (int)back_nodes.size(); // 新しい ID の割り当て

                        back_nodes.push_back(BackNode{prev, u, a});
                        index_bwd.emplace(back_nodes[v].s, v);

                        if ((int)meta_bwd.size() <= v) { // ノード記録用のベクタのサイズが小さい場合
                            meta_bwd.resize(v+1, MetaB{0,false});
                        }

                        meta_bwd[v].g = tentative_g;
                        meta_bwd[v].closed = false; // ノードを生成した段階ではクローズリストに含めない

                        open_bwd.insert(static_cast<BucketPQ::Value>(v), pack_fh_asc(tentative_g, 0)); // オープンリストへの挿入
                    } else { // 既存ノードの場合
                        v = it->second; // ID の取り出し

                        if (tentative_g < meta_bwd[v].g) { // g-value が改善された場合
                            meta_bwd[v].g = tentative_g;
                            back_nodes[v].parent = u;
                            back_nodes[v].act_id = a;

                            const UKey new_key = pack_fh_asc(meta_bwd[v].g, 0); // g を f として、h は 0 として扱う

                            if (likely(meta_bwd[v].closed)) { // 既にクローズドリストに含まれている場合
                                if (!p.reopen_closed) { // オープンリストの再オープンが禁止の場合
                                    ++R.stats.duplicates;
                                    continue;
                                }

                                // 再オープンが許される場合
                                meta_bwd[v].closed = false; // そのノードをクローズリストから取り除く
                                open_bwd.insert(static_cast<BucketPQ::Value>(v), new_key); // オープンリストに挿入する

                            } else { // 何らかの理由でクローズドリストに含まれない場合
                                auto val = static_cast<BucketPQ::Value>(v);

                                if (open_bwd.contains(val)) { // オープンリストに含まれる場合
                                    const auto cur_key = open_bwd.key_of(val); // 現在の値

                                    // forward 側と同様に key の増減を行う
                                    if (new_key < cur_key) {
                                        open_bwd.decrease_key(val, new_key);
                                    } else if (new_key > cur_key) {
                                        open_bwd.increase_key(val, new_key);
                                    }
                                } else { // 何らかの理由でオープンリストに含まれない場合
                                    open_bwd.insert(val, new_key); // オープンリストに再挿入する
                                }
                            }
                        } else { // g-value が改善されない場合
                            ++R.stats.duplicates;
                            continue;
                        }
                    }

                    // meeting 判定（新しいまたは改善された backward 状態 v に対して forward 側の全ノードをチェックする）
                    const RegState& rv = back_nodes[v].s;
                    const int gv = meta_bwd[v].g;

                    for (int f_id=0; f_id < (int)R.nodes.size(); ++f_id) {
                        const State& sf = R.nodes[f_id].s;
                        if (!forward_state_satisfies_reg(sf, rv)) { // forward の状態が該当の backward の状態を subsume しない場合
                            continue;
                        }

                        // subsume する forward state が見つかった場合
                        int gf = 0;

                        if (f_id < (int)meta_fwd.size()) {
                            gf = meta_fwd[f_id].g; // forward state の g-value を記録
                        }

                        const double cand = static_cast<double>(gf) + static_cast<double>(gv);

                        if (cand + 1e-12 < best_cost) { // meeting した状態の中で一番コストの小さいものを記録する
                            best_cost = cand;
                            have_meeting = true;
                            best_f = f_id;
                            best_b = v;
                        }
                    }
                }
            }

            if (!did_expand) { // 展開が行えなかった場合
                if (open_fwd.empty() && open_bwd.empty()) { // 両者のオープンリストがともに空の場合
                    break;
                }
            }

            // 前後を交互に切り替える
            expand_forward_turn = !expand_forward_turn;

            if (p.stop_on_first_meet && have_meeting) { // forward search と regression search が meet した場合
                break;
            }
        }

    } else { // コストに整数以外のものが含まれている場合 (未実装)
        std::cerr << "error: action costs or heuristic are non-integer.\n";
        std::exit(201);
    }

    // --- プラン復元 ---
    // 前向き側: init → meeting forward state
    std::vector<uint32_t> prefix = extract_plan_forward(R.nodes, best_f); // 関数内で reverse されているので、initial state -> meeting forward state の順の action となる

    // 後ろ向き側: meeting regression state → goal
    std::vector<uint32_t> suffix;
    for (int b_id = best_b; b_id >= 0 && back_nodes[b_id].parent >= 0; b_id = back_nodes[b_id].parent) {
        suffix.push_back(back_nodes[b_id].act_id); // meeting regression state -> goal の順で action を積んでいく
    }

    R.plan.clear();
    R.plan.reserve(prefix.size() + suffix.size());
    for (auto a : prefix) { // forward search のプランを順番にベクタに積む
        R.plan.push_back(a);
    }
    for (auto a : suffix) {
        R.plan.push_back(a); // regression search のプランを順番にベクタに積む
    }

    R.solved     = true;
    R.plan_cost  = eval_plan_cost_local(T, R.plan);

    return R;
}

}} // namespace planner::sas
