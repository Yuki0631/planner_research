#pragma once
#include "functional"
#include "sas/sas_heuristic.hpp" // 既存のヒューリスティック実装を想定
#include "sas/parallel_SOC/node.hpp"

namespace planner { 
namespace sas {

struct Task;
struct State;


} // namespace sas
} // namespace planner 

namespace planner {
namespace sas {
namespace parallel_SOC {

class Heuristic {
    planner::sas::HeuristicFn hfn_;

public:

explicit Heuristic(planner::sas::HeuristicFn fn) : hfn_(std::move(fn)) {} // コンストラクタ

double operator()(const Task& task, State& s) const {
    return hfn_(task,s); // 関数呼出演算子
}

static Heuristic goalcount() { // goalcount() をメンバ変数に持つ Heuristic object を返す関数
    return Heuristic(planner::sas::goalcount());
}

static Heuristic blind() { // blind() をメンバ変数に持つ Heuristic object を返す関数
    return Heuristic(planner::sas::blind());
}

};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
