#pragma once
#include <vector>
#include <string>
#include <functional>
#include <cstdint>
#include "sas/sas_reader.hpp"
#include "sas/sas_heuristic.hpp"
#include <atomic>
#include <chrono>
#include <ctime>
#include <iostream>

namespace planner { namespace sas { // sas 

// ノードの情報を保管するためのデータ構造
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
    std::vector<uint32_t> plan; // op index の列
    std::vector<Node> nodes; // 探索ノード（復元用）
    Stats stats;
};

// 検索パラメータ（必要に応じて拡張）
struct Params {
    uint64_t max_expansions = (1ull<<62);
    bool reopen_closed = true;
};

// --- ユーティリティ ---
double eval_plan_cost(const planner::sas::Task& T, const std::vector<uint32_t>& plan);
std::string plan_to_string(const planner::sas::Task& T, const std::vector<uint32_t>& plan);
std::string plan_to_val(const planner::sas::Task& T, const std::vector<uint32_t>& plan);

// A* / GBFS
Result astar   (const planner::sas::Task& T, HeuristicFn h, bool h_is_integer, const Params& p);
Result gbfs    (const planner::sas::Task& T, HeuristicFn h, bool h_is_integer, const Params& p);

// Search の際中だけ有効にする CPU 時間の audit
extern std::atomic<bool> g_search_timed_out; // タイムアウトを表すフラグ
extern bool g_cpu_budget_enabled;
extern double g_cpu_limit_sec; // 許容 CPU 時間
extern double g_cpu_start_sec; // search 開始時の CPU 時刻

// 現在のプロセスが CPU 上で実際に動作していた累積時間を、秒単位で返す関数
inline double cpu_seconds() {
    timespec ts{}; // structure for a time value
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// 指定時間を超えているか判定する関数
inline bool time_exceeded_cpu() {
    if (!g_cpu_budget_enabled) {
        return false;
    }
    const double now = cpu_seconds();
    if (now - g_cpu_start_sec >= g_cpu_limit_sec) { // 許容時間を超えていた場合
        g_search_timed_out.store(true, std::memory_order_relaxed);
        return true;
    }
    return false;
}

inline void set_search_cpu_budget(double cpu_limit_sec) {
    if (cpu_limit_sec > 0.0) {
        g_cpu_budget_enabled = true;
        g_cpu_limit_sec = cpu_limit_sec;
        g_cpu_start_sec = cpu_seconds();
        g_search_timed_out.store(false, std::memory_order_relaxed);
    } else {
        g_cpu_budget_enabled = false;
        g_cpu_limit_sec = -1.0;
        g_cpu_start_sec = 0.0;
        g_search_timed_out.store(false, std::memory_order_relaxed);
    }
}

}} // namespace planner::sas
