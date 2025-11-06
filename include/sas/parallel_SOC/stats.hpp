#pragma once
#include <cstdint>
#include <vector>
#include <chrono>
#include "sas/parallel_SOC/parallel_soc_all.hpp"

namespace planner {
namespace sas {
namespace soc {

// キャッシュラインのサイズを設定
#ifndef SOC_CACHELINE_SIZE
#define SOC_CACHELINE_SIZE 64
#endif

// スレッドごとの統計値を保存する Struct
struct alignas(SOC_CACHELINE_SIZE) ThreadStats { // false sharing を防ぐための alignas(64) (64bit 境界に統計用の Struct を配置する)
    // 主要カウンタ
    uint64_t generated = 0;
    uint64_t expanded  = 0;
    uint64_t reopened  = 0;
    uint64_t duplicates_pruned = 0;

    // Open 操作
    uint64_t pushes = 0;
    uint64_t pops   = 0;
    uint64_t steals = 0; // 他キュー/他シャードから盗んだ回数

    // バケットPQ
    uint64_t bucket_window_slides   = 0; // outer window を前進させた回数
    uint64_t bucket_push_collisions = 0; // バケット衝突で再試行した回数
    uint64_t bucket_pop_empty_probes= 0; // 空バケットを引いた回数（近似負荷指標）

    // ヒューリスティック等の計測
    uint64_t relax_eval_ns = 0;

    // 観測された最大 Open サイズ（近似）
    uint64_t max_open_size_seen = 0;

    // 各統計値を 0 に戻す関数
    void reset() {
        generated = expanded = reopened = duplicates_pruned = 0;
        pushes = pops = steals = 0;
        bucket_window_slides = bucket_push_collisions = bucket_pop_empty_probes = 0;
        relax_eval_ns = 0;
        max_open_size_seen = 0;
    }        

    // 統計値を足し合わせる関数
    void add(const ThreadStats& o) {
        generated += o.generated;
        expanded  += o.expanded;
        reopened  += o.reopened;
        duplicates_pruned += o.duplicates_pruned;
        pushes += o.pushes;
        pops   += o.pops;
        steals += o.steals;
        bucket_window_slides += o.bucket_window_slides;
        bucket_push_collisions += o.bucket_push_collisions;
        bucket_pop_empty_probes += o.bucket_pop_empty_probes;
        relax_eval_ns += o.relax_eval_ns;
        if (o.max_open_size_seen > max_open_size_seen) {
            max_open_size_seen = o.max_open_size_seen;
        }
    }
};

// 全体統計（集計は呼び出し側で必要時のみ実施）、基本的にこの GlobalStats を利用する
struct GlobalStats {
    std::vector<ThreadStats> per_thread; 

    void resize(uint32_t n) {
        per_thread.resize(n);
    }

    ThreadStats sum() const {
        ThreadStats s{};
        for (const auto& t : per_thread) {
            s.add(t);
        }
            return s;
    }
};

// 時間計測を ns で返す関数
template <class F>
inline uint64_t measure_ns_and_run(F&& f) {
  const auto t0 = std::chrono::high_resolution_clock::now();
  f();
  const auto t1 = std::chrono::high_resolution_clock::now();
  return (uint64_t) std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
}

} // namespace soc
} // namespace sas
} // namespace planner
