#pragma once
#include <atomic>
#include <cstdint>
#include <thread>
#include <type_traits>
#include <utility>
#include <limits>

#if defined(__x86_64__) || defined(_M_X64) // SIMD 命令を行うために、x84_64, x64 architecture で使用可能なヘッダファイルをインクルードする
    #include <immintrin.h>
#endif

namespace planner {
namespace sas {
namespace soc {

// キャッシュラインのサイズを定義
#ifndef SOC_CACHELINE_SIZE
#define SOC_CACHELINE_SIZE 64
#endif

// 少しづつ待ち時間を増やす指数バックオフ (UMA で過剰な yield を避ける)
struct Backoff {
    uint32_t spins = 1;
    void pause() {
    #if defined(__x86_64__) || defined(_M_X64) // もし x86_64, x64 architecture ならば
        for (uint32_t i = 0; i < spins; ++i) {
            _mm_pause();
        }
    #else
        // ARM 等
        for (uint32_t i = 0; i < spins; ++i) {
            /* busy */
        }
    #endif
        if (spins < 1u << 12) {
            spins <<= 1; // 指数的に spins を増加させる
        }
    }
    void yield() { // バックオフ時間が長引いたら他のスレッドに CPU を譲る関数
        std::this_thread::yield(); 
        if (spins < 1u << 10) {
            spins <<= 1;
        }
    }
};

// T-TAS(Test & Test-And-Set) 方式のスピンロック用のクラス
class SpinLock {
    std::atomic_flag flag = ATOMIC_FLAG_INIT; 
public:
    // ロックの安全性のために、コピー等を禁止する
    SpinLock() = default; // デフォルトコンストラクタ
    SpinLock(const SpinLock&) = delete; // コピーコンストラクタ禁止
    SpinLock& operator=(const SpinLock&) = delete; // コピー代入禁止

    void lock() {
        Backoff bk;

        for (;;) {
            // flag.test_and_set が false ならば他スレッドと共有しない
            if (!flag.test_and_set(std::memory_order_acquire)) { // lock なので、memory_order_acquire を使用する
                return;
            }
            // 他スレッドが使用中の場合は、バックオフ時間待機する
            while (flag.test_and_set(std::memory_order_relaxed)) { // 他スレッドが使っている最中の場合
                flag.clear(std::memory_order_relaxed); // flag を true -> false にする
                bk.pause(); // バックオフ時間待つ
            }
        }
    }

    void unlock() {
        flag.clear(std::memory_order_release); // unlock なので、memory_order_release を使用する
    }
};

// FIFO 順にスレッドが lock & unlock を行うためのチケットロック
class TicketLock {
    alignas(SOC_CACHELINE_SIZE) std::atomic<uint32_t> next_{0}; // 次のスレッドの番号
    alignas(SOC_CACHELINE_SIZE) std::atomic<uint32_t> cur_{0}; // 現在のスレッドの番号

public:
    void lock() {
        uint32_t my = next_.fetch_add(1, std::memory_order_acq_rel); // 次のスレッドの番号を受け取り、その値を 1 インクリメントする
        Backoff bk;
        while (cur_.load(std::memory_order_acquire) != my) { // 現在の番号が自分の番号でなければ、バックオフ時間停止する
            bk.pause();
        }
    }

    void unlock() {
        cur_.fetch_add(1, std::memory_order_release); // 現在の番号を 1 インクリメントする
    }
};

// スコープを抜けたら自動的に unlock される自動ロッククラス
template <class Lockable>
class ScopedLock {
    Lockable& lk_;
public:
    explicit ScopedLock(Lockable& l) : lk_(l) { // コンストラクタ、同時に lock を行う
        lk_.lock();
    }
    ~ScopedLock() { // デストラクタ、同時に unlock を行う
        lk_.unlock();
    }
    ScopedLock(const ScopedLock&) = delete; // コピーコンストラクタ禁止
    ScopedLock& operator=(const ScopedLock&) = delete; // コピー代入禁止
};

// Struct や変数をキャッシュライン境界に配置し、false sharing を防ぐための Struct
template <class T>
struct alignas(SOC_CACHELINE_SIZE) cache_aligned {
    static_assert(std::is_trivially_destructible<T>::value, "cache_aligned expects trivially destructible T"); // デストラクタを持たない軽量な型しか入れられない
    T value;
    template <class... Args>
    explicit cache_aligned(Args&&... args) : value(std::forward<Args>(args)...) {} // 完全転送を行う
};

// ハッシュ値を基に、特定の分割領域のインデックスを取得するための関数
inline uint32_t stripe_index(uint64_t hash64, uint32_t stripes) {
    return (uint32_t)(hash64 % stripes);
}

// XorShift法を用いて高速な乱数生成のための Struct
struct XorShift32 {
    uint32_t s;
    explicit XorShift32(uint32_t seed = 1u) : s(seed ? seed : 1u) {} // デフォルトの seed value は 1, 0 だと値が変わらないので、その場合は 1 にする

    uint32_t operator()() {
        uint32_t x = s;
        // ビットシャッフルを行う
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        return s = (x ? x : 1u);
    }

    // [0, n) の一様乱数を生成する関数
    uint32_t uniform(uint32_t n) {
        return (uint32_t)((uint64_t)operator()() * n >> 32); // x/2^32 -> [0, 1.0), x/2^32 * n -> [0, n)
    }
};

// スレッドの ID を登録登録する
inline thread_local uint32_t g_thread_index = std::numeric_limits<uint32_t>::max(); // 初期値は 2^32-1, thread_local で、スレッドごとに独立した変数を持たせる

// スレッドの id を登録する関数
inline void set_current_thread_index(uint32_t i) {
    g_thread_index = i;
}
// 現在のスレッドのインデックスを取得する関数
inline uint32_t current_thread_index() {
    return g_thread_index;
}

// UMA の軽量バリア用のクラスを設定
class SimpleBarrier {
    const uint32_t n_; // 総スレッド数
    std::atomic<uint32_t> count_{0}; // 到達済みスレッド数
    std::atomic<uint32_t> phase_{0}; // フェーズ番号
public:
    explicit SimpleBarrier(uint32_t n) : n_(n) {} // コンストラクタ

    void arrive_and_wait() {
        uint32_t ph = phase_.load(std::memory_order_relaxed); // 現在のフェーズ番号を確認
        if (count_.fetch_add(1, std::memory_order_acq_rel) + 1 == n_) { // 関数が呼ばれるたびに、カウンタを 1 増やし、最後のスレッドが到達したら分岐する
            count_.store(0, std::memory_order_release); // カウンタを 0 に戻す
            phase_.fetch_add(1, std::memory_order_acq_rel); // フェーズを 1 増やす
        } else {
            Backoff bk;
            while (phase_.load(std::memory_order_acquire) == ph) { // フェーズが変わらない場合
                bk.pause(); // バックオフ時間待機する
            }
        }
    }
};

} // namespace soc
} // namespace sas
} // namespace planner
