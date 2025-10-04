#pragma once
#include <string>
#include <vector>
#include <tuple>
#include <utility>

namespace planner { namespace sas {
using State = std::vector<int>;

// 変数用の Structure
struct Variable {
    std::string name; // 変数名
    int domain = 0; // domain の個数
};

// 演算子 (FD SAS 書式)
struct Operator {
    std::string name; // 演算氏名
    std::vector<std::pair<int,int>> prevail; // prevail 条件, var_id と domain 値のペア
    // pre_post 効果: 条件 c 個 (var==val)*c の下で var : pre -> post
    // 要素: (conds, var, pre, post)
    using Cond = std::pair<int,int>;
    std::vector<std::tuple<std::vector<Cond>, int, int, int>> pre_posts;
    int cost = 1; // デフォルトコストは 1
};

// 排他グループ
struct MutexGroup {
    std::vector<std::pair<int,int>> lits; // (var==val) のリテラルを積んだベクター
};

// タスク全体
struct Task {
    int version = 3; // SAS ファイルのバージョン
    int metric = 0; // 0 or 1
    std::vector<Variable> vars; // 変数
    std::vector<int> init; // 初期値 (各変数の初期値のベクター)
    std::vector<std::pair<int,int>> goal; // var==val の組
    std::vector<Operator> ops; // 演算子
    std::vector<MutexGroup> mutexes; // 排他グループ      
};

// Mutex Group に反しているかどうか判定する関数
bool violates_mutex(const Task& T, const State& s);

// SASファイルをパースし、タスクを返す関数
Task read_file(const std::string& path);

}} // namespace planner::sas
