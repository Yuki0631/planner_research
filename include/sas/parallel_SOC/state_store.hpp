#pragma once
#include <cstdint>
#include <vector>
#include <optional>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include "sas/sas_reader.hpp" // planner::sas::State
#include "sas/parallel_SOC/parallel_soc_all.hpp"

namespace planner { 
namespace sas { 
namespace parallel_SOC {

class StateStore {
    using Map = std::unordered_map<uint64_t, sas::State>; // state と ID を対応付けるハッシュマップ
    const uint32_t stripes_; // ハッシュマップの分割数
    std::vector<Map> maps_; // 部分マップを積んだベクタ
    mutable std::vector<std::shared_mutex> locks_; // 各部分マップで用いる排他ロック

    // state id からどのバケット (部分マップ) に入れるのか決定する関数
    inline uint32_t bucket_of(uint64_t id) const noexcept {
        return static_cast<uint32_t>(id % stripes_); // ストライプ数で ID を割った時の余り
    }

public:
    explicit StateStore(uint32_t stripes = 2048) : stripes_(stripes), maps_(stripes), locks_(stripes) {} // stripes を用いて各メンバ変数を初期化する

    void put(uint64_t id, sas::State s) {
        const uint32_t tmp = bucket_of(id); // index を求める
        std::unique_lock lk(locks_[tmp]); // locks_[tmp] をロックしている最中は、他のスレッドが locks_[tmp] を触れられないため、ハッシュマップに到達できない
        maps_[tmp].emplace(id, std::move(s));
    }

    // コピー取得
    bool get(uint64_t id, sas::State& out) const {
        const uint32_t tmp = bucket_of(id);
        std::shared_lock lk(locks_[tmp]); // ロックを掛ける
        auto it = maps_[tmp].find(id);
        if (it == maps_[tmp].end()) { // もし見つからなかったら
            return false;
        }
        out = it->second; // 状態を書き換える
        return true;
    }

    // 読み取り専用のガード
    struct ReadRef {
        const sas::State* ptr = nullptr;
        std::shared_lock<std::shared_mutex> hold; // shared_mutex を使用する

        const sas::State* operator->() const { // -> 演算子
            return ptr;
        }
        const sas::State& operator*()  const { // * 演算子
            return *ptr;
        }
        explicit operator bool() const { // その参照が有効か確認するための演算子
            return ptr != nullptr;
        }
    };

    // 書き込み専用のガード
    struct WriteRef {
        sas::State* ptr = nullptr;
        std::unique_lock<std::shared_mutex> hold;

        sas::State* operator->() { // -> 演算子
            return ptr;
        }
        sas::State& operator*()  { // * 演算子
            return *ptr;
        }
        explicit operator bool() const { // その参照が有効か確認するための演算子
            return ptr != nullptr;
        }
    };

    std::optional<ReadRef> get_read(uint64_t id) const {
        const uint32_t tmp = bucket_of(id); // index の取得
        std::shared_lock<std::shared_mutex> lk(locks_[tmp]); // ロックを掛ける

        auto it = maps_[tmp].find(id); // 状態をハッシュマップから探す
        if (it == maps_[tmp].end()) { // 見つからなかった場合
            return std::nullopt;
        }
        return std::optional<ReadRef>(std::in_place, ReadRef{&it->second, std::move(lk)}); // ポインタとロックを入れて返す
    }

    std::optional<WriteRef> get_write(uint64_t id) {
        const uint32_t tmp = bucket_of(id); // index の取得
        std::unique_lock<std::shared_mutex> lk(locks_[tmp]); // ロックを掛ける

        auto it = maps_[tmp].find(id); // 状態をハッシュマップから探す
        if (it == maps_[tmp].end()) { // 見つからなかった場合
            return std::nullopt;
        }
        return std::optional<WriteRef>(std::in_place, WriteRef{&it->second, std::move(lk)}); // ポインタとロックを入れて返す
    }

};

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
