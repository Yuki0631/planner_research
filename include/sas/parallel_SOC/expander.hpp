#pragma once
#include <vector>
#include "sas/sas_reader.hpp"
#include "sas/parallel_SOC/node.hpp"

namespace planner { 
namespace sas { 
namespace parallel_SOC {

// 生成された状態を扱うための Struct
struct Generated {
    sas::State state;
    uint32_t op_id;
    int cost;
};

class Expander {
public:
    static void apply(const sas::Task& T, const sas::State& s, std::vector<Generated>& out);
};

// インプレース化などは後で作成

} // nameplace parallel_SOC
} // namespace sas
} // namespace planner
