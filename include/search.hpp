#pragma once
#include <cmath>
#include "strips.hpp"
#include "heuristic.hpp"
#include <vector>
#include <string>

namespace planner {

// Node のデータ構造
struct Node {
    StripsState s;
    int parent = -1; // 親のノード id
    int act_id = -1; // 適用したアクション id
};

// bucket_pq 用のデータ構造
struct NodeStats {
    int id; // ノード id
    int f; // f 値
};

// 統計情報
struct SearchStats {
    int generated = 0;
    int expanded  = 0;
    int duplicates = 0;
};

// 探索結果の返り値として用いるための struct
struct SearchResult {
    bool solved = false;
    std::vector<int> plan; // アクション id を格納する vector
    double plan_cost = 0.0;
    SearchStats stats;
    std::vector<Node> nodes; // 経路上のノードを格納する用の vector
};

// 探索用のパラメータをまとめた struct
struct SearchParams {
    int   max_expansions = 50000000;
    bool  reopen_closed  = true; // closed をもう一度 open するかどうか
    bool  stop_on_generate_goal = true; // 生成時に goal 条件を満たしていたら停止
};

// 経路復元用の関数
std::vector<int> extract_plan(const std::vector<Node>& nodes, int goal_id);

// plan の cost を計算する関数
double eval_plan_cost(const StripsTask& st, const std::vector<int>& plan);

// plan を文字列として表示する関数
std::string plan_to_string(const StripsTask& st, const std::vector<int>& plan);

// 探索アルゴリズム
SearchResult astar(const StripsTask& st, HeuristicFn h, const SearchParams& p = {}); // A*

// cost がすべて整数かどうかを判定する関数
bool all_action_costs_are_integers(const planner::StripsTask& st, double eps = 1e-9);

// f 値を丸める関数
int rounding(double v);


} // namespace planner
