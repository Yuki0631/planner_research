#include "sas/sas_heuristic.hpp"

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

}}
