#pragma once
#include "sas/sas_reader.hpp"
#include "functional"

namespace planner { namespace sas {
    using HeuristicFn = std::function<double(const planner::sas::Task&, const State&)>;

    HeuristicFn goalcount();
    HeuristicFn blind();


}}