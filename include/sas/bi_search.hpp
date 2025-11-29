#pragma once
#include "sas/sas_reader.hpp"
#include "sas/sas_search.hpp"

namespace planner { namespace sas {

Result bidir_astar(const Task& T, HeuristicFn h, bool h_is_integer, const Params& p);

} // namespace sas
} // namespace planner
