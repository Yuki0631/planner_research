#pragma once
#include <atomic>
#include <chrono>
#include "sas/parallel_SOC/node.hpp"

namespace planner {
namespace sas {
namespace parallel_SOC {

// 探索時間を制御する Struct
struct Termination {
    std::chrono::steady_clock::time_point t0; // スタート時刻
    int time_limit_ms; // タイムリミット (ミリ秒)
    std::atomic<bool> found{false}; // 解が見つかったかどうか表すフラグ

    Termination(int time_ms = -1) : t0(std::chrono::steady_clock::now()), time_limit_ms(time_ms) {} // コンストラクタ、現在の時間を t0 にし、タイムリミットを設定する

    // タイムアウトしているかどうか判定する関数
    bool timed_out() const {
        if (time_limit_ms <= 0) { // タイムリミットが負の場合
            return false;
        }
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count(); // 経過時刻
        return dt >= time_limit_ms; // 経過時刻がタイムリミットをオーバーしたか
    }
};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
