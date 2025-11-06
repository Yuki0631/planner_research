#pragma once
#include <vector>
#include <functional>
#include "sas/sas_reader.hpp"
#include "sas/parallel_SOC/node.hpp"
#include "sas/parallel_SOC/parallel_soc_all.hpp"

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
    
    // インプレース化した関数 
    static void for_each_inplace(const sas::Task& T, sas::State& s, const std::function<void(uint32_t /*op_id*/, int /*add_cost*/, const sas::State& /*succ*/)> &cb); // 引数にコールバック関数を入れる
};

// インプレース化などは後で作成

} // nameplace parallel_SOC
} // namespace sas
} // namespace planner
