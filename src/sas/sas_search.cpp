#include "sas/sas_search.hpp"
#include "bucket_pq.hpp"
#include <robin_hood.h>
#include <atomic>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <limits>
#include <cassert>
#include <iostream>

namespace planner { namespace sas {

using Task = planner::sas::Task;
using Operator = planner::sas::Operator;

int g_mutex_mode = 0;
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

// ハッシュ／比較
struct VecHash {
    std::size_t operator()(const State& v) const noexcept {
        // シンプルな 64-bit mix
        std::size_t h = 1469598103934665603ull;
        for (int x : v) {
            std::size_t y = static_cast<std::size_t>(x) + 0x9e3779b97f4a7c15ull;
            h ^= y;
            h *= 1099511628211ull;
        }
        return h;
    }
};

struct VecEq {
    bool operator()(const State& a, const State& b) const noexcept {
        return a == b;
    }
};

// プラン評価/表示
double eval_plan_cost(const Task& T, const std::vector<int>& plan) {
    double c = 0.0;
    for (int a : plan) {
        c += T.ops[a].cost;
    }
    return c;
}

std::string plan_to_string(const Task& T, const std::vector<int>& plan) {
    std::ostringstream oss;
    for (std::size_t i=0;i<plan.size();++i) {
        if (i) oss << "\n";
        int a = plan[i];
        oss << i << ": " << T.ops[a].name << " [cost=" << T.ops[a].cost << "]";
    }
    return oss.str();
}

std::string plan_to_val(const Task& T, const std::vector<int>& plan) {
    std::ostringstream oss;
    for (std::size_t i=0;i<plan.size();++i) {
        if (i) oss << "\n";
        oss << "(" << T.ops[plan[i]].name << ")";
    }
    oss << "\n; cost = " << std::setprecision(17) << eval_plan_cost(T, plan);
    oss << "\n; length = " << plan.size() << "\n";
    return oss.str();
}


// ゴール判定
static inline bool is_goal(const Task& T, const State& s) {
    for (auto [v,val] : T.goal) {
        if (s[v] != val) {
            return false;
        }
    }
    return true;
}

// 適用判定
static inline bool is_applicable(const Task& T, const State& s, const Operator& op) {
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

// 差分適用（Undo 付き）
using Undo = std::vector<std::pair<int,int>>; // (var, old_value)

static inline std::size_t undo_mark(const Undo& u) { return u.size(); }

static inline void undo_to(State& s, Undo& u, std::size_t mark) {
    for (std::size_t i = u.size(); i-- > mark; ) {
        const auto [var, oldv] = u[i];
        s[var] = oldv;
    }
    u.resize(mark);
}

static inline void apply_inplace(const Task& T, const Operator& op, State& s, Undo& u) {
    // 代入効果：var := post
    for (const auto& pp : op.pre_posts) {
        int var  = std::get<1>(pp);
        int post = std::get<3>(pp);
        if (s[var] != post) {
            u.emplace_back(var, s[var]);
            s[var] = post;
        }
    }
}

// ノード→プラン復元
static std::vector<int> extract_plan(const std::vector<Node>& nodes, int goal_id) {
    std::vector<int> acts;
    for (int v = goal_id; v >= 0 && nodes[v].parent >= 0; v = nodes[v].parent) {
        acts.push_back(nodes[v].act_id);
    }
    std::reverse(acts.begin(), acts.end());
    return acts;
}

// 整数判定／丸め
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

// RAII で必ず巻き戻すためのガード
struct UndoGuard {
    State& work;
    Undo& undo;
    std::size_t mark;
    ~UndoGuard() { undo_to(work, undo, mark); }
};

// CPU 時間の audit の設定
std::atomic<bool> g_search_timed_out{false};
bool g_cpu_budget_enabled = true;
double g_cpu_limit_sec = -1.0;
double g_cpu_start_sec = 0.0;

// A* search
Result astar(const Task& T, HeuristicFn h, const bool h_int, const Params& p) {
    Result R;

    // 初期ノード
    State s0(T.vars.size());
    for (int v=0; v<(int)T.vars.size(); ++v) {
        s0[v] = T.init[v];
    }
    R.nodes.push_back(Node{ s0, -1, -1 });

    if (is_goal(T, s0)) {
        R.solved = true;
        R.plan_cost = 0.0;
        R.plan.clear();
        return R;
    }

#if defined(USE_ROBIN_HOOD)
    robin_hood::unordered_map<State, int, VecHash, VecEq> index_of;
#else
    std::unordered_map<State, int, VecHash, VecEq> index_of;
    index_of.max_load_factor(0.50f);
#endif
    index_of.reserve(1<<15);
    index_of.emplace(R.nodes[0].s, 0);

    if (all_action_costs_are_integers(T) && h_int) {
        // 実際のモード表示
        {
            const bool do_mutex = should_check_mutex_runtime(T);
            if (do_mutex) {
                std::cout << "Mutex check: ON\n";
            } else {
                std::cout << "Mutex check: OFF\n";
            }
        }

        std::cout << "Note: all action costs are integers; using integer A* + BucketPQ.\n";

        struct MetaI { int g; int h; bool closed; };
        std::vector<MetaI> meta(1, MetaI{0,0,false});

        TwoLevelBucketPQ open;
        const int h0 = rounding(h(T, s0));
        ++R.stats.evaluated;
        meta[0] = MetaI{0, h0, false};
        open.insert(0, pack_fh_asc(h0, h0));

        State work;
        Undo undo;
        work = s0;
        undo.clear();

        while (!open.empty()) {
            if (planner::sas::time_exceeded_cpu()) {
                std::cerr << "error: CPU time limit exceeded (" << planner::sas::g_cpu_limit_sec << " sec)\n";
                std::exit(101);
            }

            auto [u32, key] = open.extract_min();
            const int u = static_cast<int>(u32);
            const int fu = unpack_f(key);
            const int hu = unpack_h(key);

            const int fu_now = meta[u].g + meta[u].h;
            if (fu != fu_now || hu != meta[u].h) continue;

            const State su = R.nodes[u].s;

            if (is_goal(T, su)) {
                R.solved = true;
                R.plan = extract_plan(R.nodes, u);
                R.plan_cost = eval_plan_cost(T, R.plan);
                return R;
            }

            meta[u].closed = true;

            ++R.stats.expanded;
            if (R.stats.expanded > p.max_expansions) break;

            for (int a=0; a<(int)T.ops.size(); ++a) {
                const auto& op = T.ops[a];
                if (!is_applicable(T, su, op)) continue;

                work = su;
                undo.clear();
                const std::size_t mark = undo_mark(undo);

                UndoGuard ug{work, undo, mark};

                apply_inplace(T, op, work, undo);
                ++R.stats.generated;

                // 生成状態が mutex 違反なら捨てる
                if (should_check_mutex_runtime(T)) {
                    if (planner::sas::violates_mutex(T, work)) {
                        continue;
                    }
                }

                const int w = rounding(op.cost);
                const int tentative_g = meta[u].g + w;

                auto it = index_of.find(work);
                if (it == index_of.end()) {
                    const int v = (int)R.nodes.size();
                    R.nodes.push_back(Node{work, u, a});
                    index_of.emplace(R.nodes[v].s, v);

                    const int hv = rounding(h(T, R.nodes[v].s));
                    ++R.stats.evaluated;
                    if ((int)meta.size() <= v) meta.resize(v+1);
                    meta[v] = MetaI{tentative_g, hv, false};

                    open.insert(static_cast<BucketPQ::Value>(v), pack_fh_asc(tentative_g + hv, hv));
                } else {
                    const int v = it->second;
                    if (tentative_g < meta[v].g) {
                        meta[v].g = tentative_g;
                        R.nodes[v].parent = u;
                        R.nodes[v].act_id = a;

                        meta[v].h = rounding(h(T, R.nodes[v].s));
                        ++R.stats.evaluated;
                        const UKey new_key = pack_fh_asc(meta[v].g + meta[v].h, meta[v].h);

                        if (meta[v].closed) {
                            if (!p.reopen_closed) {
                                ++R.stats.duplicates;
                                continue;
                            }
                            meta[v].closed = false;
                            open.insert(static_cast<BucketPQ::Value>(v), new_key);
                        } else {
                            if (open.contains(static_cast<BucketPQ::Value>(v))) {
                                const auto cur_key = open.key_of(static_cast<BucketPQ::Value>(v));
                                if (new_key < cur_key) {
                                    open.decrease_key(static_cast<BucketPQ::Value>(v), new_key);
                                } else if (new_key > cur_key) {
                                    open.increase_key(static_cast<BucketPQ::Value>(v), new_key);
                                }
                            } else {
                                open.insert(static_cast<BucketPQ::Value>(v), new_key);
                            }
                        }
                    } else {
                        ++R.stats.duplicates;
                        if (meta[v].closed && !p.reopen_closed) {
                            continue;
                        }
                    }
                }
            }
        }
        return R;

    } else {
        {
            const bool do_mutex = should_check_mutex_runtime(T);
            if (do_mutex) {
                std::cout << "Mutex check: ON\n";
            } else {
                std::cout << "Mutex check: OFF\n";
            }
        }

        std::cout << "Note: action costs are not all integers; using non-integer A*.\n";

        struct MetaD { double g; double h; bool closed; };
        std::vector<MetaD> meta(1, MetaD{0.0, 0.0, false});

        struct QEl { double f; double h; int id; };
        auto cmp = [](const QEl& a, const QEl& b){
            if (a.f != b.f) return a.f > b.f;
            return a.h > b.h;
        };
        std::priority_queue<QEl, std::vector<QEl>, decltype(cmp)> open(cmp);

        meta[0] = MetaD{0.0, h(T, s0), false};
        ++R.stats.evaluated;
        open.push({ meta[0].g + meta[0].h, meta[0].h, 0 });

        State work;
        Undo undo;
        work = s0;
        undo.clear();
        constexpr double EPS = 1e-12;

        while (!open.empty()) {
            if (planner::sas::time_exceeded_cpu()) {
                std::cerr << "error: CPU time limit exceeded (" << planner::sas::g_cpu_limit_sec << " sec)\n";
                std::exit(101);
            }
            QEl cur = open.top(); open.pop();
            const int u = cur.id;

            const double fu_now = meta[u].g + meta[u].h;
            if (std::fabs(cur.f - fu_now) > EPS) continue;

            const State su = R.nodes[u].s;

            if (is_goal(T, su)) {
                R.solved = true;
                R.plan = extract_plan(R.nodes, u);
                R.plan_cost = eval_plan_cost(T, R.plan);
                return R;
            }

            meta[u].closed = true;

            ++R.stats.expanded;
            if (R.stats.expanded > p.max_expansions) {
                break;
            }

            for (int a=0; a<(int)T.ops.size(); ++a) {
                const auto& op = T.ops[a];
                if (!is_applicable(T, su, op)) {
                    continue;
                }

                work = su; undo.clear();
                const std::size_t mark = undo_mark(undo);

                UndoGuard ug{work, undo, mark};

                apply_inplace(T, op, work, undo);
                ++R.stats.generated;

                // 生成状態が mutex 違反なら捨てる
                if (should_check_mutex_runtime(T)) {
                    if (planner::sas::violates_mutex(T, work)) {
                        continue;
                    }
                }

                const double tentative_g = meta[u].g + op.cost;

                auto it = index_of.find(work);
                if (it == index_of.end()) {
                    const int v = (int)R.nodes.size();
                    R.nodes.push_back(Node{work, u, a});
                    index_of.emplace(R.nodes[v].s, v);

                    const double hv = h(T, R.nodes[v].s);
                    ++R.stats.evaluated;
                    if ((int)meta.size() <= v) {
                        meta.resize(v+1);
                    }
                    meta[v] = MetaD{tentative_g, hv, false};
                    open.push({ tentative_g + hv, hv, v });
                } else {
                    const int v = it->second;
                    if (tentative_g + EPS < meta[v].g) {
                        meta[v].g   = tentative_g;
                        R.nodes[v].parent = u;
                        R.nodes[v].act_id = a;

                        meta[v].h = h(T, R.nodes[v].s);
                        ++R.stats.evaluated;
                        if (meta[v].closed && !p.reopen_closed) {
                            ++R.stats.duplicates;
                            continue;
                        }
                        open.push({ meta[v].g + meta[v].h, meta[v].h, v });
                    } else {
                        ++R.stats.duplicates;
                        if (meta[v].closed && !p.reopen_closed) continue;
                    }
                }
            }
        }
        return R;
    }
}

// GBFS
Result gbfs(const Task& T, HeuristicFn h, const bool h_int, const Params& p) {
    Result R;

    State s0(T.vars.size());
    for (int v=0; v<(int)T.vars.size(); ++v) {
        s0[v] = T.init[v];
    }
    R.nodes.push_back(Node{ s0, -1, -1 });

    if (is_goal(T, s0)) {
        R.solved = true; R.plan_cost = 0.0; R.plan.clear();
        return R;
    }

#if defined(USE_ROBIN_HOOD)
    robin_hood::unordered_map<State, int, VecHash, VecEq> index_of;
#else
    std::unordered_map<State, int, VecHash, VecEq> index_of;
    index_of.max_load_factor(0.50f);
#endif
    index_of.reserve(1<<15);
    index_of.emplace(s0, 0);

    const bool integer_mode = (all_action_costs_are_integers(T) && h_int);

    {
            const bool do_mutex = should_check_mutex_runtime(T);
            if (do_mutex) {
                std::cout << "Mutex check: ON\n";
            } else {
                std::cout << "Mutex check: OFF\n";
            }
    }

    if (integer_mode) {
        std::cout << "Note: all action costs and heuristic are integer; using BucketPQ GBFS.\n";

        struct MetaI { int g; int h; bool closed; };
        std::vector<MetaI> meta(1, MetaI{0, 0, false});

        TwoLevelBucketPQ open_pref; // preferred
        TwoLevelBucketPQ open_norm; // not-preferred

        const int h0 = rounding(h(T, s0));
        ++R.stats.evaluated;
        meta[0] = MetaI{0, h0, false};
        open_norm.insert(0, pack_fh_asc(h0, 0)); // pack_fh means pack_hg here, 初期状態では、not-preferred 

        State work;
        Undo undo;
        work = s0;
        undo.clear();

        while (!open_pref.empty() || !open_norm.empty()) {
            if (planner::sas::time_exceeded_cpu()) {
                std::cerr << "error: CPU time limit exceeded (" << planner::sas::g_cpu_limit_sec << " sec)\n";
                std::exit(101);
            }

            // open_pref があればそこから pop() し、なければ open_norm から pop() する関数
            auto pick = [&]() {
                if (!open_pref.empty()) {
                    return open_pref.extract_min();
                }
                return open_norm.extract_min();
            };

            auto [id32, key] = pick();
            const int u = static_cast<int>(id32);
            const int hu = unpack_f(key);
            const int gu = unpack_h(key);

            const State su = R.nodes[u].s;

            if (is_goal(T, su)) {
                R.solved = true;
                R.plan = extract_plan(R.nodes, u);
                R.plan_cost = eval_plan_cost(T, R.plan);
                return R;
            }

            meta[u].closed = true;

            ++R.stats.expanded;
            if (R.stats.expanded > p.max_expansions) {
                break;
            }

            for (int a=0; a<(int)T.ops.size(); ++a) {
                const auto& op = T.ops[a];
                if (!is_applicable(T, su, op)) {
                    continue;
                }

                work = su; undo.clear();
                const std::size_t mark = undo_mark(undo);

                UndoGuard ug{work, undo, mark};

                apply_inplace(T, op, work, undo);
                ++R.stats.generated;

                // 生成状態が mutex 違反なら捨てる
                if (should_check_mutex_runtime(T)) {
                    if (planner::sas::violates_mutex(T, work)) {
                        continue;
                    }
                }

                auto it = index_of.find(work);
                if (it == index_of.end()) {
                    const int hv = rounding(h(T, work));
                    ++R.stats.evaluated;
                    const bool is_preferred = (hv < meta[u].h);

                    const int v = (int)R.nodes.size();
                    R.nodes.push_back(Node{work, u, a});
                    index_of.emplace(R.nodes[v].s, v);


                    if ((int)meta.size() <= v) {
                        meta.resize(v<<1);
                    }
                    const int gv = meta[u].g + rounding(op.cost);
                    meta[v] = MetaI{gv, hv, false};
                    if (is_preferred) {
                        open_pref.insert(static_cast<uint32_t>(v), pack_fh_asc(hv, gv));
                    } else {
                        open_norm.insert(static_cast<uint32_t>(v), pack_fh_asc(hv, gv));
                    }
                } else {
                    ++R.stats.duplicates;
                    continue;
                }
            }
        }
        return R;

    } else {
        std::cout << "Note: heuristic or costs are non-integer; using std::priority_queue GBFS.\n";

        struct MetaD { double h; double g; bool closed; };
        std::vector<MetaD> meta(1, MetaD{0.0, 0.0, false});

        struct QEl { double h; double g; int id; };
        auto cmp = [](const QEl& a, const QEl& b){
            if (a.h != b.h) {
                return a.h > b.h;
                return a.g > b.g;
            }
        };
        std::priority_queue<QEl, std::vector<QEl>, decltype(cmp)> open_pref(cmp);
        std::priority_queue<QEl, std::vector<QEl>, decltype(cmp)> open_norm(cmp);

        meta[0] = MetaD{ h(T,s0), 0.0, false };
        ++R.stats.evaluated;
        open_norm.push({ meta[0].h, meta[0].g, 0 });

        State work; Undo undo; work = s0; undo.clear();

        while (!open_pref.empty() || !open_norm.empty()) {
            if (planner::sas::time_exceeded_cpu()) {
                std::cerr << "error: CPU time limit exceeded (" << planner::sas::g_cpu_limit_sec << " sec)\n";
                std::exit(101);
            }
            QEl cur;

            if (!open_pref.empty()) {
                cur = open_pref.top();
                open_pref.pop();
            } else {
                cur = open_norm.top();
                open_norm.pop();
            }

            const int u = cur.id;
            const State su = R.nodes[u].s;

            if (is_goal(T, su)) {
                R.solved = true;
                R.plan = extract_plan(R.nodes, u);
                R.plan_cost = eval_plan_cost(T, R.plan);
                return R;
            }

            meta[u].closed = true;

            ++R.stats.expanded;
            if (R.stats.expanded > p.max_expansions) break;

            for (int a=0; a<(int)T.ops.size(); ++a) {
                const auto& op = T.ops[a];
                if (!is_applicable(T, su, op)) {
                    continue;
                }

                work = su; undo.clear();
                const std::size_t mark = undo_mark(undo);

                UndoGuard ug{work, undo, mark};

                apply_inplace(T, op, work, undo);
                ++R.stats.generated;

                if (should_check_mutex_runtime(T)) {
                    if (planner::sas::violates_mutex(T, work)) {
                        continue;
                    }
                }

                auto it = index_of.find(work);
                if (it == index_of.end()) {
                    const int v = (int)R.nodes.size();
                    R.nodes.push_back(Node{work, u, a});
                    index_of.emplace(R.nodes[v].s, v);

                    const double hv = h(T, R.nodes[v].s);
                    ++R.stats.evaluated;
                    const bool is_preferred = (hv < meta[u].h);

                    if ((int)meta.size() <= v) {
                        meta.resize(v<<1);
                    }
                    const double gv = meta[u].g + op.cost;
                    meta[v] = MetaD{hv, gv, false};
                    if (is_preferred) {
                        open_pref.push({ hv, gv, v });
                    } else {
                        open_norm.push({ hv, gv, v });
                    }
                } else {
                    ++R.stats.duplicates;
                    continue;
                }
            }
        }
        return R;
    }
}

}} // namespace planner::sas
