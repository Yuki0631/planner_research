#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <limits>
#include "lexer.hpp"
#include "parser.hpp"
#include "grounding.hpp"
#include "strips.hpp"

using namespace planner;

// Node Struct
struct Node {
    StripsState s;
    int parent = -1;
    int act_id = -1;
};

int main() {
    // --- 動作確認用：switch ドメイン（1手でON） ---
    const char* dom = R"(
(define (domain switch)
  (:requirements :strips :typing :action-costs)
  (:predicates (switch_is_on) (switch_is_off))
  (:functions (total-cost) - number)
  (:action switch_on
    :parameters ()
    :precondition (switch_is_off)
    :effect (and (switch_is_on)
                 (not (switch_is_off))
                 (increase (total-cost) 1)))
  (:action switch_off
    :parameters ()
    :precondition (switch_is_on)
    :effect (and (switch_is_off)
                 (not (switch_is_on))
                 (increase (total-cost) 1)))
)
)";

    const char* prob = R"(
(define (problem p1)
  (:domain switch)
  (:objects)
  (:init (switch_is_off) (= (total-cost) 0))
  (:goal (switch_is_on))
  (:metric minimize (total-cost))
)
)";

    try {
        // --- Parse ---
        Lexer Ld(dom), Lp(prob);
        Parser Pd(Ld), Pp(Lp);
        Domain D = Pd.parseDomain();
        Problem P = Pp.parseProblem();

        // --- Ground ---
        GroundTask GT = ground(D, P);
        std::cerr << "Grounded actions: " << GT.actions.size() << "\n";

        // --- STRIPS へ ---
        StripsTask ST = compile_to_strips(GT);
        StripsState init = make_init_state(ST);
        std::cerr << "Facts: " << ST.num_facts()
                  << ", Actions: " << ST.actions.size() << "\n";
        std::cerr << "Init: " << state_to_string(ST, init, GT) << "\n";

        // --- 単純BFS の実装 ---
        std::vector<Node> nodes;
        nodes.reserve(1024);
        nodes.push_back({init, -1, -1}); // 初期状態を node に詰め込む

        std::queue<int> q; // node の id を入れる用の FIFO Queue
        q.push(0);

        std::unordered_set<StripsState, StripsStateHash> visited; // 訪れたことのある状態を管理する map
        visited.insert(init);

        int goal_idx = -1;

        while (!q.empty()) {
            // 先頭ノードの取り出し
            int u = q.front();
            q.pop();

            // ゴール条件の判定
            if (is_goal(ST, nodes[u].s)) {
                goal_idx = u;
                break;
            }

            // 各アクションの実行
            for (int ai = 0; ai < (int)ST.actions.size(); ++ai) {
                const auto& act = ST.actions[ai];

                if (!is_applicable(ST, nodes[u].s, act)) continue; // もし適用可能でないならば

                StripsState ns; // 出力先の状態の作成
                apply(ST, nodes[u].s, act, ns);

                if (visited.insert(ns).second) { // insert に成功した場合
                    nodes.push_back({std::move(ns), u, ai}); // 状態, 親の id, action id を保存する
                    q.push((int)nodes.size()-1); // Queue に ノードの id を保存
                }
            }
        }

        if (goal_idx < 0) { // goal が見つからなかった場合
            std::cout << "No plan found.\n";
            return 0;
        }

        // --- プラン復元 ---
        std::vector<std::string> plan;
        for (int v = goal_idx; nodes[v].parent >= 0; v = nodes[v].parent) {
            plan.push_back(ST.actions[nodes[v].act_id].name); // action の名前を plan に順に入れていく
        }
        std::reverse(plan.begin(), plan.end()); // goal -> start となっているので、反転させる

        // --- 出力 ---
        std::cout << "Plan length: " << plan.size() << "\n";
        double cost = 0.0;
        for (auto& an : plan) {
            // 未実装
        }

        // 累積コスト計算（未実装）
        StripsState cur = init;
        double acc = 0.0; // accumulated cost
        for (int i = 0, u = 0; i < (int)plan.size(); ++i) { // plan 中の各 action に対して
            int ai = -1;
            for (int k=0; k<(int)ST.actions.size(); ++k) {
                if (ST.actions[k].name == plan[i]) {
                    ai = k;
                    break;
                }
            }
            if (ai >= 0) {
                acc += ST.actions[ai].cost;
                apply(ST, cur, ST.actions[ai], cur); // cur を更新する
            }
        }

        std::cout << "Plan:\n";
        for (auto& s : plan) std::cout << "  " << s << "\n";
        std::cout << "Reached: " << state_to_string(ST, nodes[goal_idx].s, GT) << "\n";
        std::cout << "Accumulated cost (demo): " << acc << "\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
