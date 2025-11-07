#pragma once
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include "sas/parallel_SOC/parallel_soc_all.hpp"

namespace planner {
namespace sas {
namespace parallel_SOC {

enum class QueueKind : uint8_t { // Open リストで使用するキューの種類
    MultiQueue = 0,
    BucketPQ   = 1
};

enum class TieBreak : uint8_t { // Tie-Break の手法
    HThenG = 0, // h 値優先でタイブレーク
    GThenH = 1, // g 値優先でタイブレーク      
    FIFO   = 2 // 単純な f 値 FIFO        
};

struct Params {
    // 並列構成
    uint32_t num_threads = 1;
    uint32_t num_queues = 0;
    uint32_t closed_stripes = 0; // closed list の分割数

    // 探索ポリシー
    QueueKind queue_kind = QueueKind::BucketPQ;
    TieBreak tie_break = TieBreak::HThenG;
    float weight = 1.0f;
    bool reopen_closed = true;
    bool early_terminate_on_first_goal = true;

    // ロギング・再現性
    uint32_t random_seed = 634u;
    uint32_t log_interval_ms = 1000u;

    // メモリ目安（ソフト上限）
    std::size_t memory_soft_limit_mb = 0; // 0 => 無効

    // BucketPQ 用パラメータ
    float bucket_delta = 1.0f;
    uint32_t buckets_window = 256;
    uint32_t bucket_shards = 0;
    uint32_t bucket_select_k = 2;
    bool bucket_fifo = true;

    // スレッドアフィニティ（UMA なので既定は false とする）
    bool pin_threads = false;

    // ベリファイの厳しさ
    bool assert_monotone_pop = false;

    // サニタイザー
    void sanitize() {
        if (num_threads == 0) { // スレッド数が 0 の場合
            num_threads = 1;
        }

        if (num_queues == 0) { // キューの数 (Multi Queue の場合) が 0 の時
            num_queues = std::max<uint32_t>(2u * num_threads, 2u);
        }

        if (closed_stripes == 0) { // クローズドリストの分割数が 0 の時
            closed_stripes = std::max<uint32_t>(4u * num_threads, 8u);
        }
        num_queues = std::min<uint32_t>(num_queues, 16u * num_threads); // 極端な多重を制限する

        weight = (weight < 1.0f) ? 1.0f : weight; // weight が 1.0 未満の場合は、すべて 1.0 とする

        bucket_delta = (bucket_delta <= 0.0f) ? 1.0f : bucket_delta;
        buckets_window = std::max<uint32_t>(buckets_window, 32u);
        bucket_shards = (bucket_shards == 0) ? std::max<uint32_t>(num_threads, 2u) : bucket_shards;
        bucket_select_k = std::min<uint32_t>(std::max<uint32_t>(bucket_select_k, 1u), 8u);

        log_interval_ms = std::max<uint32_t>(log_interval_ms, 100u);
    }
};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
