#include "sas/sas_heuristic.hpp"
#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>

namespace planner { namespace sas {

HeuristicFn goalcount() {
    return [](const Task& T, const State& s) -> double {
        int miss = 0;
        for (auto [v, val] : T.goal) {
            if (s[v] != val) ++miss;
        }
        return static_cast<double>(miss);
    };
}

HeuristicFn blind() {
    return [](const Task&, const State&) -> double {
        return 0.0;
    };
}

// hff 用のインターフェース
namespace { // 衝突を避けるために無名名前空間を使用する

// FF 用のデータ構造をまとめたもの
struct FFData {
    const Task* T; // read_sas で読み取ったタスク

    int nvars = 0; // 変数の数
    int nfacts = 0; // 事実の数 (var=d)

    std::vector<int> var_offset; // fact_id(v,val) = var_offset[v] + val, 事実 ID 計算用の各変数のオフセット値

    // 緩和した演算子用の structure
    struct RelaxAction {
        std::vector<int> pre; // 満たすべき事実の集合 (fact-id)
        std::vector<int> add; // 真になる事実の集合 (fact-id)
        double cost = 1.0; // コスト
    };

    // 緩和演算子を積む用のベクタ
    std::vector<RelaxAction> actions;

    // コンストラクタ
    explicit FFData(const Task& task) : T(&task) {
        nvars = static_cast<int>(task.vars.size()); // 変数の数

        var_offset.resize(nvars + 1);

        int counter = 0;
        for (int v = 0; v < nvars; ++v) {
            var_offset[v] = counter;
            counter += task.vars[v].domain;   // 各変数の domain サイズ
        }

        var_offset[nvars] = counter;
        nfacts = counter;

        // SAS の operator から relaxed action を生成する
        actions.reserve(task.ops.size());

        for (const auto& op : task.ops) {
            RelaxAction ra;
            ra.cost = static_cast<double>(op.cost);

            // fact-id を計算するための関数
            auto fact_id = [&](int v, int val) { // v: variable, val: donain-value
                return var_offset[v] + val;
            };

            // prevail 条件をすべて pre に入れる
            for (auto [v, val] : op.prevail) {
                ra.pre.push_back(fact_id(v, val));
            }

            // 各 pre_post 行：
            // conds（条件）を pre に、
            // pre >= 0 なら (v == pre) も pre に、
            // post を add に
            for (const auto& pp : op.pre_posts) {
                const auto& conds = std::get<0>(pp);
                int var = std::get<1>(pp);
                int pre = std::get<2>(pp);
                int post = std::get<3>(pp);

                // conds を pre に入れる
                for (auto [cv, cval] : conds) {
                    ra.pre.push_back(fact_id(cv, cval));
                }

                if (pre >= 0) {
                    ra.pre.push_back(fact_id(var, pre));
                }

                ra.add.push_back(fact_id(var, post));
            }

            // 重複を削る関数
            auto uniq = [](std::vector<int>& v) {
                std::sort(v.begin(), v.end());
                v.erase(std::unique(v.begin(), v.end()), v.end());
            };

            uniq(ra.pre);
            uniq(ra.add);

            actions.push_back(std::move(ra));
        }
    }

    // 状態 s に対する h^FF(s) を計算
    double compute(const State& s) const {
        const double INF = std::numeric_limits<double>::infinity();
        const double PSEUDOINF = 1 << 16; // 疑似的な INF を 2^16 とする
        const Task& task = *T;


        // h_add と best-supporter を積むためのベクタ
        std::vector<double> h(nfacts, INF);
        std::vector<int> supporter(nfacts, -1);

        // fact が状態 s において真か偽かを登録するためのベクタ
        std::vector<bool> fact_in_s(nfacts, false);

        // 実状態で真な fact のコストを 0 にする
        for (int v = 0; v < nvars; ++v) {
            int val = s[v];
            int f = var_offset[v] + val; // fact-id
            fact_in_s[f] = true;
            h[f] = 0.0;
        }

        // h_add の反復緩和
        bool changed = true;
        while (changed) {
            changed = false;
            for (int ai = 0; ai < static_cast<int>(actions.size()); ++ai) {
                const auto& act = actions[ai]; // 緩和アクション

                double pre_cost = 0.0;
                bool unreachable = false;

                for (int p : act.pre) {
                    if (!(p >= 0 && p < nfacts)) { // 前提条件の数が負または fact の数以上の場合
                        unreachable = true;
                        break;
                    }

                    if (!std::isfinite(h[p])) { // 前提条件の h_add のコストが無限大の場合
                        unreachable = true;
                        break;
                    }

                    pre_cost += h[p];  // h_add: 前提のコストを足し合わせる
                }

                if (unreachable) {
                    continue;
                }

                double cand = pre_cost + act.cost;

                for (int q : act.add) {
                    if (!(q >= 0 && q < nfacts)) {
                        continue;
                    }

                    // ベストサポータの更新
                    if (cand + 1e-12 < h[q]) {
                        h[q] = cand;
                        supporter[q] = ai;
                        changed = true;
                    }
                }
            }
        }

        // ゴール事実が到達不能であるか判定する
        for (auto [v, val] : task.goal) {
            int g = var_offset[v] + val;

            if (g < 0 || g >= nfacts) { // ゴールの変数と変数値がそもそも定義の外にある場合
                return PSEUDOINF;
            }

            if (!std::isfinite(h[g])) { // ゴール条件が到達不可能な場合
                return PSEUDOINF;
            }
        }

        // relaxed plan の抽出（バックチェイン）
        std::vector<bool> closed(nfacts, false);
        std::vector<bool> in_plan(actions.size(), false);

        std::vector<int> stack;
        stack.reserve(task.goal.size() * 4);

        double cost = 0.0;

        // ゴール事実を push する
        for (auto [v, val] : task.goal) {
            int g = var_offset[v] + val;
            if (fact_in_s[g]) { // すでに初期状態で fact が満たされている場合
                continue;
            }

            if (supporter[g] < 0) { // サポータの値が -1 (unreachable) の場合
                return PSEUDOINF;
            }

            stack.push_back(g);
        }

        // サブゴールに対して bs(p) から前提を再帰的に積んでいく
        while (!stack.empty()) {
            int g = stack.back();
            stack.pop_back();

            if (g < 0 || g >= nfacts) { // fact-id が定義の外である場合
                continue;
            }

            if (closed[g]) { // 一度既出の fact の場合
                continue;
            }

            closed[g] = true;

            int a = supporter[g]; // ベストサポータ

            if (a < 0) { // s で真になっている場合など、サポータの値が -1 の場合
                continue;
            }

            if (a < 0 || a >= static_cast<int>(actions.size())) { // action-id が定義の外にある場合
                continue;
            }

            if (!in_plan[a]) { // アクションがプランに入っていない場合
                in_plan[a] = true;
                cost += actions[a].cost;

                const auto& act = actions[a];

                // act.pre を新たなサブゴールとして積む
                for (int p : act.pre) {
                    if (p < 0 || p >= nfacts) { // pre fact が定義の外である場合
                        continue;
                    }

                    if (fact_in_s[p]) { // すでに初期状態で真の場合
                        continue;
                    }

                    if (closed[p]) { // 既出の場合
                        continue;
                    }

                    if (supporter[p] < 0) { // サポータの値が -1 の場合 (unreachable)
                        return PSEUDOINF;
                    }

                    stack.push_back(p); // スタックに積む
                }
            }
        }

        return cost;
    }
};

} // anonymous namespace

// ランドマークヒューリスティック用のデータ構造
struct LMData {
    const Task* T;

    int nvars = 0; // 変数の数
    int nfacts = 0; // fact の数 (変数の数 x 平均ドメインサイズ)

    std::vector<int> var_offset; // fact_id(v,val) = var_offset[v] + val

    // fact-id から変数名とドメイン値を高速に見つけるためのベクタ
    std::vector<int> fact_var;
    std::vector<int> fact_val;

    // landmark-fact 用のデータ構造
    struct Landmark {
        int fact; // fact-id
        double weight; // コスト
    };

    std::vector<Landmark> landmarks;

    // コンストラクタ
    LMData(const Task& task) : T(&task) { // プランニングタスクを受け取り、T にポインタを保管する
        // fact-id の割り振り
        nvars = static_cast<int>(task.vars.size());
        var_offset.resize(nvars + 1);

        int counter = 0;
        for (int v = 0; v < nvars; ++v) {
            var_offset[v] = counter;
            counter += task.vars[v].domain;
        }
        var_offset[nvars] = counter;
        nfacts = counter;

        fact_var.resize(nfacts);
        fact_val.resize(nfacts);

        for (int v = 0; v < nvars; ++v) {
            for (int d = 0; d < task.vars[v].domain; ++d) {
                int f = var_offset[v] + d;
                // 変数とドメイン値用のベクタへの登録
                fact_var[f] = v;
                fact_val[f] = d;
            }
        }

        // fact-id を求める関数
        auto fact_id = [&](int v, int val) {
            return var_offset[v] + val;
        };

        // 初期状態で真な fact を登録する
        std::vector<bool> fact_in_init(nfacts, false);

        for (int v = 0; v < nvars; ++v) {
            int val = task.init[v];
            int f = fact_id(v, val);
            if (0 <= f && f < nfacts) {
                fact_in_init[f] = true;
            }
        }

        // 各演算子の、前提 fact(s) と追加 fact(s) を計算、登録する
        // 演算子情報を登録するためのデータ構造
        struct ActionInfo {
            std::vector<int> pre; // prevail, conds, pre の fact-id を積むためのベクタ
        };

        std::vector<ActionInfo> act_info(task.ops.size());

        std::vector<std::vector<int>> achievers(nfacts); // ある fact を達成するための演算子の集合を積んだベクタ

        for (int a = 0; a < (int)task.ops.size(); ++a) {
            const auto& op = task.ops[a];
            auto& info = act_info[a];

            // prevail 条件を action info に登録する
            for (auto [v, val] : op.prevail) {
                info.pre.push_back(fact_id(v, val));
            }

            // pre_posts の前提条件を同様に登録する
            for (const auto& pp : op.pre_posts) {
                const auto& conds = std::get<0>(pp);
                int var  = std::get<1>(pp);
                int pre  = std::get<2>(pp);
                int post = std::get<3>(pp);

                for (auto [cv, cval] : conds) {
                    info.pre.push_back(fact_id(cv, cval));
                }

                if (pre >= 0) {
                    info.pre.push_back(fact_id(var, pre));
                }

                int q = fact_id(var, post);
                if (0 <= q && q < nfacts) {
                    achievers[q].push_back(a);
                }
            }

            // action info の前提条件の重複除去を行う
            std::sort(info.pre.begin(), info.pre.end());
            info.pre.erase(std::unique(info.pre.begin(), info.pre.end()), info.pre.end());
        }

        // ゴール状態で満たされるべき事実をランドマークに登録する
        std::vector<bool> is_landmark_fact(nfacts, false); // landmark fact かどうか登録するベクタ
        for (auto [v, val] : task.goal) {
            int g = fact_id(v, val);
            if (0 <= g && g < nfacts && !is_landmark_fact[g]) { // fact-id が定義内にある かつ 事前に landmark fact に登録されていない
                is_landmark_fact[g] = true;
                landmarks.push_back(Landmark{ g, 1.0 });
            }
        }

        // 後ろ向きに、ランドマークを登録していく
        // ある landmark fact f を達成可能にする全ての achiever の pre の共通部分を P とする
        // P に含まれる fact は f の前に必ず満たす必要があるので、それらを新たなランドマークとする

        for (std::size_t idx = 0; idx < landmarks.size(); ++idx) {
            int f = landmarks[idx].fact;
            const auto& achs = achievers[f]; // landmark-fact f を達成するための fact の集合

            if (achs.empty()) { // achiever が存在しない場合
                continue;
            }

            // 共通部分の計算
            std::vector<int> inter; // intersection
            bool first = true;

            for (int a : achs) {
                const auto& pre = act_info[a].pre;

                if (pre.empty()) { // 前提条件が空の achiever がある -> intersection は常に空
                    inter.clear();
                    first = false;
                    break;
                }

                if (first) { // 最初に前提条件を登録する場合
                    inter = pre;
                    first = false;
                } else {
                    std::vector<int> tmp;

                    std::set_intersection( // 計算量は O(n+m)
                        inter.begin(), inter.end(), // 一つ目の集合の範囲
                        pre.begin(), pre.end(), // 二つ目の集合の範囲
                        std::back_inserter(tmp)); // 書き込み先
                    
                    inter.swap(tmp);

                    if (inter.empty()) { // intersection が空になってしまった場合
                        break;
                    }
                }
            }

            // 新しくランドマークに fact を登録する
            for (int p : inter) {
                if (p < 0 || p >= nfacts) { // fact-id が定義の外にある場合
                    continue;
                }

                if (fact_in_init[p]) { // 初期状態で真の場合
                    continue;
                }

                if (is_landmark_fact[p]) { // 既にランドマークに登録されている場合
                    continue;
                }

                is_landmark_fact[p] = true; // ランドマークの判定用ベクタに登録する

                landmarks.push_back(Landmark{ p, 1.0 }); // landmark facts に登録する
            }
        }
    }

    // 状態 s に対するヒューリスティック値 (未達成 landmark fact 数) を計算する関数
    double compute(const State& s) const {
        double h = 0.0;

        for (const auto& lm : landmarks) {
            int f = lm.fact; // fact-id
            int var = fact_var[f];
            int val = fact_val[f];

            // 状態 s で landmark fact が真であるか判定する
            bool satisfied = false;
            if (0 <= var && var < (int)s.size()) {
                if (s[var] == val) {
                    satisfied = true;
                }
            }

            if (!satisfied) { // true でない landmark fact が存在する場合
                h += lm.weight;
            }
        }
        return h;
    }
};


HeuristicFn hff(const Task& T) {
    // Task ごとに FFData を構築する
    auto data = std::make_shared<FFData>(T);

    return [data](const Task& /*unused*/, const State& s) -> double {
        return data->compute(s);
    };
}

HeuristicFn hlm(const Task& T) {
    // Task ごとに landmark fact に関するデータを生成する
    auto data = std::make_shared<LMData>(T);

    return [data](const Task& /*unused*/, const State& s) -> double {
        return data->compute(s);
    };
}


} // namespace sas
} // namespace planner
