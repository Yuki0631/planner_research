#pragma once
#include <vector>
#include <string>
#include <functional>
#include <cstdint>
#include "sas/sas_reader.hpp"
#include "sas/sas_heuristic.hpp"

namespace planner { namespace sas { // sas 

// 盤面ノード
struct Node {
    State s;
    int parent; // 親ノード id
    int act_id; // 適用した演算子 id
};

// 統計情報
struct Stats {
    uint64_t expanded = 0;
    uint64_t generated = 0;
    uint64_t evaluated = 0;
    uint64_t duplicates = 0;
};

// 探索結果
struct Result {
    bool solved = false;
    double plan_cost = 0.0;
    std::vector<int> plan; // op index の列
    std::vector<Node> nodes; // 探索ノード（復元用）
    Stats stats;
};

// 検索パラメータ（必要に応じて拡張）
struct Params {
    uint64_t max_expansions = (1ull<<62);
    bool reopen_closed = true;
};

// --- ユーティリティ ---
double eval_plan_cost(const planner::sas::Task& T, const std::vector<int>& plan);
std::string plan_to_string(const planner::sas::Task& T, const std::vector<int>& plan);
std::string plan_to_val(const planner::sas::Task& T, const std::vector<int>& plan);

// A* / GBFS
Result astar   (const planner::sas::Task& T, HeuristicFn h, bool h_is_integer, const Params& p);
Result gbfs    (const planner::sas::Task& T, HeuristicFn h, bool h_is_integer, const Params& p);

}} // namespace planner::sas
