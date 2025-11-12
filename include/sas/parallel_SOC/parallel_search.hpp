#pragma once
#include <optional>
#include <vector>
#include <cstdint>
#include "sas/sas_reader.hpp"
#include "sas/parallel_SOC/state_store.hpp"
#include "sas/parallel_SOC/id_allocator.hpp"
#include "sas/parallel_SOC/shared_open_list.hpp"
#include "sas/parallel_SOC/closed_table.hpp"
#include "sas/parallel_SOC/expander.hpp"
#include "sas/parallel_SOC/heuristic_adapter.hpp"
#include "sas/parallel_SOC/termination.hpp"
#include "sas/parallel_SOC/stats.hpp"

#include "sas/parallel_SOC/parallel_soc_all.hpp"

namespace planner {
namespace sas {
namespace parallel_SOC {

// 探索を特徴づけするパラメータ
struct SearchParams {
    uint32_t num_threads = 1; // スレッドの数
    SharedOpen::Kind open_kind = SharedOpen::Kind::MultiQueue; // オープンリストの種類
    uint32_t num_queues = 0; // キューの数
    int time_limit_ms = -1; // タイムリミット
    uint32_t num_bucket_shards = 0;
    uint32_t num_k_select = 2;
};

// 探索結果
struct SearchResult {
    bool solved = false;
    int cost = -1;
    std::vector<uint32_t> plan_ops; // 演算子のシーケンス
};

// A* Search
SearchResult astar_soc(const sas::Task& T, const SearchParams& P, planner::sas::soc::GlobalStats* stats_out = nullptr);

}}} // namespace
