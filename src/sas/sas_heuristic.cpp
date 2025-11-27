#include "sas/sas_heuristic.hpp"
#include <memory>
#include <algorithm>
#include <limits>


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

    // limits::infinity でないか確認するための関数
    bool is_finite(double x) const {
        return x <= std::numeric_limits<double>::infinity() && x >= -std::numeric_limits<double>::infinity();
    }

    // 状態 s に対する h^FF(s) を計算
    double compute(const State& s) const {
        const double INF = std::numeric_limits<double>::infinity();
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

                    if (!is_finite(h[p])) { // 前提条件の h_add のコストが無限大の場合
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
                return INF;
            }

            if (!is_finite(h[g])) { // ゴール条件が到達不可能な場合
                return INF;
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
                return INF;
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
                        return INF;
                    }

                    stack.push_back(p); // スタックに積む
                }
            }
        }

        return cost;
    }
};

} // anonymous namespace


HeuristicFn hff(const Task& T) {
    // Task ごとに FFData を構築する
    auto data = std::make_shared<FFData>(T);

    return [data](const Task& /*unused*/, const State& s) -> double {
        return data->compute(s);
    };
}


} // namespace sas
} // namespace planner
