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
#include "params.hpp"
#include "concurrency.hpp"

#define HAVE_BUCKET_PQ 1
#include "bucket_pq.hpp"


namespace planner {
namespace sas {
namespace parallel_SOC {

// マルチキュー型のオープンリスト
class MultiQueueOpen {
    // priority queue 用の struct
    struct PQ {
        std::priority_queue<Node, std::vector<Node>, NodeLess> q; // Node を要素とし、比較関数は、NodeLess とする (node.hpp)
        planner::sas::soc::SpinLock m; // 軽量ロック、書き込みメイン
    };

    std::vector<PQ> qs_; // PQ を積んだベクトル
    std::atomic<uint64_t> sz_{0}; // 全体のサイズを数え上げるカウンタ

public:
    explicit MultiQueueOpen(uint32_t num_queues) : qs_(num_queues ? num_queues : 1) {} // コンストラクタ、キューの数を num_queues or 1 に設定する

    // push 関数
    void push(uint32_t qid, Node&& n) {
        auto& pq = qs_[qid % qs_.size()]; // priority queue のインデックスは、ID をキューの総数で割った時の余りとする
        // critical section
        {
            planner::sas::soc::ScopedLock<planner::sas::soc::SpinLock> lg(pq.m); // その priority queue をロックする
            pq.q.push(std::move(n)); // Queue に値を入れる
        }
        sz_.fetch_add(1, std::memory_order_relaxed); // 全体のノードの数を 1 だけインクリメントする
    }

    // pop 関数
    std::optional<Node> pop(uint32_t qid) {
        const uint32_t N = qs_.size(); // マルチキューの総数

        // ID が担当しているキューからの取り出し
        {
            auto& pq = qs_[qid % N]; // ID から計算されるインデックスのキュー
            planner::sas::soc::ScopedLock<planner::sas::soc::SpinLock> lg(pq.m); // ロックを行う
            if (!pq.q.empty()) { // キューが空でない場合
                Node n = std::move(const_cast<Node&>(pq.q.top())); // top()
                pq.q.pop(); // pop()
                sz_.fetch_sub(1, std::memory_order_relaxed); // 全体のノードの数を 1 だけデクリメントする
                return n;
            }
        }

        // ID が担当しているキューが空の場合、他のキューから盗み取る
        for (uint32_t t=0; t<N; ++t) {
            uint32_t k = (qid + 1 + t) % N; // 担当以外のキューのインデックスを計算する
            auto& pq = qs_[k];
            std::lock_guard lg(pq.m); // ロックを行う
            if (!pq.q.empty()) { // そのキューがから出ない場合
                Node n = std::move(const_cast<Node&>(pq.q.top())); // top()
                pq.q.pop(); // pop()
                sz_.fetch_sub(1, std::memory_order_relaxed); // 全体のノード数を 1 だけデクリメントする。
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
        return sz_.load(std::memory_order_relaxed);
    }

};


// Two-Level Bucket Open（並列A* 向け、f-value を一次バケット（キー）、同一 f の中のバケットは FIFO（deque）
// 複数シャードに分割して、ロック競合を低減する。また、pop 時に k-choice でシャードをランダムサンプリングする
class TwoLevelBucketOpen {
    using BucketPQ = TwoLevelBucketPQ;

    struct Shard {
        planner::sas::soc::TicketLock m; // ロック、書き込みメイン、FIFO の軽量ロックを用いる
        BucketPQ pq;
        std::unordered_map<BucketPQ::Value, Node> store; // node 本体を保存するハッシュマップ (bucket_pq.hpp 自体は ID (uint_32t) の保存しかできない)
        uint64_t size = 0; // このシャードに入っている Node の個数
    };

    std::vector<Shard> shards_; // シャードを積んだベクタ
    std::atomic<uint64_t> sz_{0}; // 全体のノードの個数
    uint32_t k_choice_; // pop 時にサンプルするシャードの個数

    // 乱数生成器 (スレッドごとに独立)
    static uint32_t rng_next() {
        thread_local std::mt19937 rng{std::random_device{}()};
        return rng();
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

    // push 関数
    void push(uint32_t, Node&& n) { // 第一引数の Queue-ID はこの関数では無視する
        const uint32_t S = static_cast<uint32_t>(shards_.size()); // シャードの数
        const uint32_t sid = pick_shard(n.id, S); // どのシャードにそのノードを入れるかのインデックス
        auto& sh = shards_[sid]; // 該当シャード

        const int h = n.h; // h-value
        const UKey key = pack_key(n.g + h, h); // パックする

        // critical section
        {
            planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(sh.m); // 該当シャードをガードする
            auto id = n.id;
            sh.store.emplace(id, std::move(n)); // ハッシュマップに、ID と ノードを入れる
            sh.pq.insert(id, key); // バケットに挿入する
            ++sh.size; // シャードに含まれるノード数を増加させる
        }
        sz_.fetch_add(1, std::memory_order_relaxed); // 全体のノード数を 1 インクリメントする
    }

    // pop 関数
    std::optional<Node> pop(uint32_t qid) { // 引数には、スレッドの ID を用いる
        const uint32_t S = static_cast<uint32_t>(shards_.size()); // シャード数
        if (sz_.load(std::memory_order_relaxed) == 0) { // オープンリストに含まれる全体のノード数が 0 の場合
            return std::nullopt;
        }

        uint32_t seed = rng_next(); // シード値 (thread_local)

        // k-choice sampling
        for (uint32_t t = 0; t < k_choice_; ++t) {
            uint32_t sid = (qid + seed + t) % S; // (thread-id + seed-value + (0~k-1)) をシャード数で割った余り
            auto& sh = shards_[sid]; // 該当シャード

            // critical section
            planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(sh.m); // ロックを掛ける
            if (sh.size == 0 || sh.pq.empty()) { // シャードに含まれるノードが 0 この場合
                continue;
            }

            auto [vid, key] = sh.pq.extract_min();
            auto it = sh.store.find(vid);
            if (it != sh.store.end()) { // 見つかった ID がハッシュマップに存在している場合
                Node out = std::move(it->second); // 該当ノードを取得
                sh.store.erase(it); // ハッシュマップから、そのノードを削除する
                --sh.size; // 該当シャードに含まれるノード数を 1 減らす
                sz_.fetch_sub(1, std::memory_order_relaxed); // 全体のノード数を 1 だけデクリメントする
                return out;
            }
        }

        // ID がハッシュマップに登録されていない場合
        for (uint32_t sid = 0; sid < S; ++sid) { // 他のシャードを探索する
            auto& sh = shards_[sid];

            // critical section
            planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(sh.m); // ロックを掛ける
            if (sh.size == 0 || sh.pq.empty()) {
                continue;
            }

            // 見つかるまでは、上のコードと同じ操作を繰り返す
            auto [vid, key] = sh.pq.extract_min();
            auto it = sh.store.find(vid);
            if (it != sh.store.end()) {
                Node out = std::move(it->second);
                sh.store.erase(it);
                --sh.size;
                sz_.fetch_sub(1, std::memory_order_relaxed);
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
        return sz_.load(std::memory_order_relaxed);
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

public:
    // コンストラクタ
    explicit SharedOpen(Kind k, uint32_t num_queues, uint32_t bucket_shards = 0, uint32_t bucket_select_k = 2)
        : kind_(k)
        , mq_(num_queues ? num_queues : 1) // マルチキュー型、シャード数と、k-value は用いない
        , tlb_(bucket_shards ? bucket_shards : std::max(2u, (num_queues?num_queues:1)), bucket_select_k ? bucket_select_k : 2) // 二段バケット型、num_queues は用いない
    {}

    // push 関数
    void push(uint32_t qid, Node&& n) {
        switch (kind_) {
            case Kind::MultiQueue: mq_.push(qid, std::move(n)); break;
            case Kind::TwoLevelBucket: tlb_.push(qid, std::move(n)); break;
        }
    }

    // pop 関数
    std::optional<Node> pop(uint32_t qid) {
        switch (kind_) {
            case Kind::MultiQueue: return mq_.pop(qid);
            case Kind::TwoLevelBucket: return tlb_.pop(qid);
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
};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner