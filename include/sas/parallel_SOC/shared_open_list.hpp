#pragma once
#include <queue>
#include <random>
#include <optional>
#include <atomic>
#include <thread>
#include <vector>
#include <mutex>
#include <map>
#include <deque>
#include <cstdint>

#include "sas/parallel_SOC/node.hpp"
#include "sas/parallel_SOC/id_allocator.hpp"
#include "sas/parallel_SOC/params.hpp"
#include "sas/parallel_SOC/concurrency.hpp"

#define HAVE_BUCKET_PQ 1
#include "bucket_pq.hpp"

#include "sas/parallel_SOC/stats.hpp"

#include "sas/parallel_SOC/parallel_soc_all.hpp"


namespace planner {
namespace sas {
namespace parallel_SOC {

// マルチキュー型のオープンリスト
class MultiQueueOpen {
    planner::sas::soc::GlobalStats* gstats_ = nullptr;
    // priority queue 用の struct
    struct PQ {
        std::priority_queue<Node, std::vector<Node>, NodeLess> q; // Node を要素とし、比較関数は、NodeLess とする (node.hpp)
        planner::sas::soc::SpinLock m; // 軽量ロック、書き込みメイン
        std::atomic<uint64_t> size{0}; // 各優先度付きキューのサイズ
    };

    std::vector<PQ> qs_; // PQ を積んだベクトル
    uint32_t k_choice_; // pop 時にランダムサンプリングするキューの数

    // 乱数生成
    static uint32_t rng_next() {
        thread_local std::mt19937 rng{std::random_device{}()};

        // 乱数の安全性の確保 (uint32_t の最小・最大に合わせてスケーリングを行うオブジェクト)
        thread_local std::uniform_int_distribution<uint32_t> dist(
            std::numeric_limits<uint32_t>::min(), std::numeric_limits<uint32_t>::max()
        );

        return dist(rng);
    }

    // ノードの ID を用いて、どのシャードに挿入するのかを決定する (乗算ハッシュ)
    static uint32_t pick_shard(uint64_t id, uint32_t n) {
        uint64_t x = id * 11400714819323198485ull;
        return (uint32_t)((x >> 32) % n);
    }

public:
    explicit MultiQueueOpen(uint32_t num_queues, uint32_t k_choice = 2) 
    : qs_(num_queues ? num_queues : 1) 
    , k_choice_(k_choice ? k_choice : 2) {} // コンストラクタ、キューの数とランダムサンプリング数を設定する
    void set_stats(planner::sas::soc::GlobalStats* p) {
        gstats_ = p;
    }

    // push 関数
    void push(Node&& n) {
        const uint32_t N = (uint32_t)qs_.size(); // マルチキューに含まれるキューの数
        const uint32_t sid = pick_shard(n.id, N); // シャード ID
        auto& pq = qs_[sid]; // 該当シャードを参照として確保する

        // critical section
        {
            planner::sas::soc::ScopedLock<planner::sas::soc::SpinLock> lg(pq.m); // その priority queue をロックする
            pq.q.push(std::move(n)); // Queue に値を入れる
            pq.size.fetch_add(1, std::memory_order_relaxed); // キューに含まれる要素の数を 1 インクリメントする
        }
        
        if (gstats_) {
            auto tid = planner::sas::soc::current_thread_index(); // 現在のスレッドの ID の取得
            auto& S = gstats_->per_thread[tid];
            S.pushes++; // プッシュした回数を 1 インクリメントする
            auto s = size();
            if (s >S.max_open_size_seen) { // スレッドごとに観測したオープンサイズの最大のサイズを更新する
                S.max_open_size_seen = s;
            }
        }
    }

    // pop 関数
    std::optional<Node> pop() {
        const size_t N = qs_.size(); // マルチキューの総数

        if (N==0) {
            return std::nullopt;
        }

        uint32_t seed = rng_next(); // シード値の設定

        // k-choice サンプリングを行う
        for (uint32_t t = 0; t < k_choice_; ++t) {
            size_t sid = (static_cast<size_t>(seed) + t) % N; // シャード ID を決定する
            auto& pq = qs_[sid]; // 該当のシャード

            planner::sas::soc::ScopedLock<planner::sas::soc::SpinLock> lg(pq.m); // ロックをかける

            if (!pq.q.empty()) { // キューが空でない場合
                Node n = std::move(const_cast<Node&>(pq.q.top()));
                pq.q.pop();
                pq.size.fetch_sub(1, std::memory_order_relaxed);

                if (gstats_) {
                    auto tid = planner::sas::soc::current_thread_index();
                    gstats_->per_thread[tid].pops++;
                }
                return n;
            }
        }

        // k-choice で発見できなかった場合は、すべてのシャードを走査する
        for (size_t t = k_choice_; t < (N + k_choice_); ++t) {
            size_t sid = (static_cast<size_t>(seed) + t) % N;
            auto& pq = qs_[sid];

            planner::sas::soc::ScopedLock<planner::sas::soc::SpinLock> lg(pq.m);

            if (!pq.q.empty()) {
                Node n = std::move(const_cast<Node&>(pq.q.top()));
                pq.q.pop();
                pq.size.fetch_sub(1, std::memory_order_relaxed);

                if (gstats_) {
                    auto tid = planner::sas::soc::current_thread_index();
                    gstats_->per_thread[tid].pops++;
                }
                return n;
            }
        }

        // どのキューも空の場合
        return std::nullopt;
    }

    // すべてのキューが空かどうか確認する関数
    bool empty() const noexcept {
        return size() == 0;
    }

    // 全体のノード数を取得する関数
    uint64_t size() const noexcept {
        uint64_t s = 0;

        for (const auto& pq : qs_) {
            s += pq.size.load(std::memory_order_relaxed); // すべてのキューのサイズを足し合わせる
        }

        return s;
    }

};


// Two-Level Bucket Open
// 複数シャードに分割して、ロック競合を低減する。また、pop 時に k-choice でシャードをランダムサンプリングする
class TwoLevelBucketOpen {
    using BucketPQ = TwoLevelBucketPQ;
    planner::sas::soc::GlobalStats* gstats_ = nullptr;

    struct Shard {
        planner::sas::soc::TicketLock m; // ロック、書き込みメイン、FIFO の軽量ロックを用いる
        BucketPQ pq;
        std::unordered_map<BucketPQ::Value, Node> store; // node 本体を保存するハッシュマップ (bucket_pq.hpp 自体は ID (uint_32t) の保存しかできない)
        std::atomic<uint64_t> size{0};
    };

    std::vector<Shard> shards_; // シャードを積んだベクタ
    uint32_t k_choice_; // pop 時にサンプルするシャードの個数

    // 乱数生成器 (スレッドごとに独立)
    static uint32_t rng_next() {
        thread_local std::mt19937 rng{std::random_device{}()};

        // 乱数の安全性の確保 (uint32_t の最小・最大に合わせてスケーリングを行うオブジェクト)
        thread_local std::uniform_int_distribution<uint32_t> dist(
            std::numeric_limits<uint32_t>::min(), std::numeric_limits<uint32_t>::max()
        );

        return dist(rng);
    }

    // ID をハッシュ値に変換し、どのシャードに入れるのか選ぶ関数
    static inline uint32_t pick_shard(uint64_t id, uint32_t n) {
        uint64_t x = id * 11400714819323198485ull; // 乗算ハッシュ
        return static_cast<uint32_t>((x >> 32) % n); // シャード数で割った余りを用いる
    }

    // (f,h) を UKey にパックする関数（上位16bit: f, 下位16bit: h）
    static inline UKey pack_key(int f, int h = 0) {
        return (static_cast<UKey>(static_cast<uint32_t>(f)) << H_BITS) | (static_cast<UKey>(static_cast<uint32_t>(h)) & H_MASK);
    }

public:
    explicit TwoLevelBucketOpen(uint32_t shards, uint32_t k_choice = 2) // コンストラクタ、k-choice のデフォルト値は 2
        : shards_(shards ? shards : 1) // シャード数が与えられなければ 1 とする
        , k_choice_(k_choice ? k_choice : 2) {}

    void set_stats(planner::sas::soc::GlobalStats* p) {
        gstats_ = p;
        for (auto& sh : shards_) {
            sh.pq.set_stats(p); // 各シャード内の PQ にも統計用のポインタをセットする
        }
    }

    // push 関数
    void push(Node&& n) { // 第一引数の Queue-ID はこの関数では無視する
        const uint32_t num_shards = static_cast<uint32_t>(shards_.size()); // シャードの数
        const uint32_t sid = pick_shard(n.id, num_shards); // どのシャードにそのノードを入れるかのインデックス
        auto& sh = shards_[sid]; // 該当シャード

        const int h = n.h; // h-value
        const UKey key = pack_key(n.g + h, h); // パックする

        // critical section
        {
            planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(sh.m); // 該当シャードをガードする
            auto id = n.id;
            sh.store.emplace(id, std::move(n)); // ハッシュマップに、ID と ノードを入れる
            sh.pq.insert(id, key); // バケットに挿入する
            sh.size.fetch_add(1, std::memory_order_relaxed); // シャードに含まれるノード数を増加させる
        }

        if (gstats_) {
            auto tid = planner::sas::soc::current_thread_index(); // 現在のスレッドの ID の取得
            auto& stats = gstats_->per_thread[tid];
            stats.pushes++; // プッシュ回数を 1 インクリメントする
            auto total = size(); // 二段バケットに含まれる全ノード数を取得
            if (total > stats.max_open_size_seen) { // スレッドごとに観測したオープンリストの最大のサイズを更新する
                stats.max_open_size_seen = total;
            }
        }
    }

    // pop 関数
    std::optional<Node> pop() { // 引数には、スレッドの ID を用いる
        const uint32_t num_shards = static_cast<uint32_t>(shards_.size()); // シャード数

        if (num_shards == 0) { // シャード数が 0 の場合
            return std::nullopt;
        }

        if (size() == 0) { // オープンリストに含まれる全体のノード数が 0 の場合
            return std::nullopt;
        }

        uint32_t seed = rng_next(); // シード値 (thread_local)

        // k-choice sampling
        for (uint32_t t = 0; t < k_choice_; ++t) {
            uint32_t sid = (seed + t) % num_shards; // (thread-id + seed-value + (0~k-1)) をシャード数で割った余り
            auto& sh = shards_[sid]; // 該当シャード

            // critical section
            planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(sh.m); // ロックを掛ける
            if (sh.size.load(std::memory_order_relaxed) == 0 || sh.pq.empty()) { // シャードに含まれるノードが 0 この場合

                if (gstats_) { // 統計値を取っている場合
                    auto tid = planner::sas::soc::current_thread_index(); // 現在のスレッド ID の取得
                    gstats_->per_thread[tid].bucket_pop_empty_probes++; // 空のシャードを走査してしまったので、その統計値を 1 インクリメントする
                }

                continue;
            }

            auto [vid, key] = sh.pq.extract_min();
            auto it = sh.store.find(vid);
            if (it != sh.store.end()) { // 見つかった ID がハッシュマップに存在している場合
                Node out = std::move(it->second); // 該当ノードを取得
                sh.store.erase(it); // ハッシュマップから、そのノードを削除する
                sh.size.fetch_sub(1, std::memory_order_relaxed); // ノードに含まれるノード数を 1 減らす

                if (gstats_) { // 統計値を取っている場合
                    auto tid = planner::sas::soc::current_thread_index(); // 現在のスレッド ID の取得
                    gstats_->per_thread[tid].pops++; // ポップできた回数を 1 インクリメントする
                }

                return out;
            }
        }

        // ID がハッシュマップに登録されていない場合
        for (uint32_t t = k_choice_; t < (num_shards + k_choice_); ++t) { // 他のシャードを探索する
            uint32_t sid = (seed + t) % num_shards;
            auto& sh = shards_[sid];

            // critical section
            planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(sh.m); // ロックを掛ける
            if (sh.size.load(std::memory_order_relaxed) == 0 || sh.pq.empty()) {
                if (gstats_) { // 統計値を取っている場合
                    auto tid = planner::sas::soc::current_thread_index(); // 現在のスレッド ID の取得
                    gstats_->per_thread[tid].bucket_pop_empty_probes++; // 空のシャードを走査してしまったので、その統計値を 1 インクリメントする
                }

                continue;
            }

            // 見つかるまでは、上のコードと同じ操作を繰り返す
            auto [vid, key] = sh.pq.extract_min();
            auto it = sh.store.find(vid);
            if (it != sh.store.end()) {
                Node out = std::move(it->second);
                sh.store.erase(it);
                sh.size.fetch_sub(1, std::memory_order_relaxed);

                if (gstats_) { // 統計値を取っている場合
                    auto tid = planner::sas::soc::current_thread_index(); // 現在のスレッド ID の取得
                    auto &stats = gstats_->per_thread[tid];
                    stats.pops++; // ポップできた回数を 1 インクリメントする
                    if (sid != tid) {
                        stats.steals++;   
                    }
                }
            
                return out;
            }
        }
        return std::nullopt;
    }

    // 二段バケットが空であるか確認する関数
    bool empty() const noexcept {
        return size() == 0;
    }

    // 現在の二段バケットに含まれる全ノード数を確認する関数
    uint64_t size() const noexcept {
        uint64_t total = 0;

        for (const auto& sh : shards_) {
            total += sh.size.load(std::memory_order_relaxed);
        }

        return total;
    }
};

// Shared facade
class SharedOpen {
public:
    enum class Kind { MultiQueue, TwoLevelBucket }; // オープンリストの種類

private:
    Kind kind_; // 種類を保持する変数

    MultiQueueOpen mq_; // マルチキューが都のオープンリスト
    TwoLevelBucketOpen tlb_; // 二段バケット型のオープンリスト

    planner::sas::soc::GlobalStats* gstats_ = nullptr; // 統計保存用のための struct へのポインタ

public:
    // コンストラクタ
    explicit SharedOpen(Kind k, uint32_t num_queues, uint32_t bucket_shards = 0, uint32_t bucket_select_k = 2)
        : kind_(k)
        , mq_(num_queues ? num_queues : 1) // マルチキュー型、シャード数と、k-value は用いない
        , tlb_(bucket_shards ? bucket_shards : std::max(2u, (num_queues?num_queues:1)), bucket_select_k ? bucket_select_k : 2) // 二段バケット型、num_queues は用いない
    {}

    // push 関数
    void push(Node&& n) {
        switch (kind_) {
            case Kind::MultiQueue: mq_.push(std::move(n)); break;
            case Kind::TwoLevelBucket: tlb_.push(std::move(n)); break;
        }
    }

    // pop 関数
    std::optional<Node> pop() {
        switch (kind_) {
            case Kind::MultiQueue: return mq_.pop();
            case Kind::TwoLevelBucket: return tlb_.pop();
        }
        return std::nullopt; // 何も返ってこなかった場合
    }

    // empty 関数
    bool empty() const noexcept {
        switch (kind_) {
            case Kind::MultiQueue: return mq_.empty();
            case Kind::TwoLevelBucket: return tlb_.empty();
        }
        return true; // デフォルトの返し値 (コンパイルエラー・警告防止)
    }

    // size 関数
    uint64_t size() const noexcept {
        switch (kind_) {
            case Kind::MultiQueue: return mq_.size();
            case Kind::TwoLevelBucket: return tlb_.size();
        }
        return 0; // デフォルトの返し値 (コンパイルエラー・警告防止)
    }

    // 統計用の追跡ポインタをセットする関数
    void set_stats(planner::sas::soc::GlobalStats* p) {
        gstats_ = p;
        mq_.set_stats(p);
        tlb_.set_stats(p);
    }
};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner