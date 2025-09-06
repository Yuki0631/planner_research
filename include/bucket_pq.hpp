#pragma once
#include <vector>
#include <deque>
#include <cstddef>
#include <stdexcept>
#include <limits>

// 二段バケット・プライオリティキュー
// 第一キー: f（小さいほど先）  範囲 [f_min, f_max]
// 第二キー: h（小さいほど先）  範囲 [h_min, h_max]
// 同じ (f,h) では FIFO（先入れ先出し）
//
// 要件：f, h は整数で、事前に範囲がわかっていること

template <class T>
class BucketPriorityQueue {
public:
    struct Entry { // エントリ
        T value;
        int f;
        int h;
    };

    BucketPriorityQueue(int f_min, int f_max, int h_min, int h_max)
        : f_min_(f_min), f_max_(f_max), h_min_(h_min), h_max_(h_max), // 範囲の設定
          F_(f_max - f_min + 1), H_(h_max - h_min + 1), // バケットのサイズ
          buckets_(F_, std::vector<std::deque<Entry>>(H_)), // バケットの初期化
          f_counts_(F_, 0), size_(0), // f レベルごとの非空フラグと全体サイズ
          cur_f_idx_(-1), cur_h_idx_(-1) // カレントインデックス
    {
        // 範囲の妥当性チェック
        if (f_min_ > f_max_ || h_min_ > h_max_) {
            throw std::invalid_argument("invalid bucket ranges");
        }
    }

    bool empty() const noexcept { return size_ == 0; }
    std::size_t size() const noexcept { return size_; }

    // 先頭の要素の取得
    const T& top() {
        ensure_cur();
        const auto& dq = buckets_[cur_f_idx_][cur_h_idx_];
        return dq.front().value;
    }

    // 取り出し（ポップ）
    void pop() {
        ensure_cur();
        auto& dq = buckets_[cur_f_idx_][cur_h_idx_];
        dq.pop_front();
        --size_;
        if (dq.empty()) {
            // この h バケットが空になったので、次の非空位置へ進める
            if (--f_h_nonempty_left_ == 0) {
                // 現在の f レベルが空になった
                f_counts_[cur_f_idx_] = 0;
            }
            advance_to_next_nonempty(cur_f_idx_, cur_h_idx_ + 1);
        }
    }

    // 追加（push）
    void push(T value, int f, int h) {
        if (f < f_min_ || f > f_max_ || h < h_min_ || h > h_max_) {
            throw std::out_of_range("f or h out of configured range");
        }
        const int fi = f - f_min_;
        const int hi = h - h_min_;

        auto& dq = buckets_[fi][hi];
        const bool was_empty_f = (f_counts_[fi] == 0);
        const bool was_empty_cell = dq.empty();

        dq.push_back(Entry{std::move(value), f, h});
        ++size_;

        if (was_empty_f) {
            f_counts_[fi] = 1; // この f レベルに少なくとも1個あることを示す
        } else if (was_empty_cell) {
            // 同一 f 内で新しく非空セルが増えたことをカウント（後述参照）
            // ただし簡略化のためセル数は都度スキャンでもよい。
        }

        // カレント更新（より小さい (f,h) が来たら先頭に）
        if (cur_f_idx_ == -1) {
            // 初要素
            cur_f_idx_ = fi;
            cur_h_idx_ = hi;
            // この f レベルの残り非空セル数を概算：スキャン時に埋める
            f_h_nonempty_left_ = -1; // 未評価フラグ
        } else {
            if (fi < cur_f_idx_ || (fi == cur_f_idx_ && hi < cur_h_idx_)) {
                cur_f_idx_ = fi;
                cur_h_idx_ = hi;
                f_h_nonempty_left_ = -1;
            }
        }
    }

private:
    int f_min_, f_max_, h_min_, h_max_;
    int F_, H_;

    // buckets_[fi][hi] に (f_min_+fi, h_min_+hi) の要素群
    std::vector<std::vector<std::deque<Entry>>> buckets_;
    // f レベルごとの「何か入っているか」のフラグ（>0 で非空）
    std::vector<int> f_counts_;

    std::size_t size_;
    int cur_f_idx_;
    int cur_h_idx_;

    // 最適化用：現在の f レベルで非空の h バケット残数（未評価は -1）
    int f_h_nonempty_left_ = -1;

    void ensure_cur() {
        if (size_ == 0) [[unlikely]] {
            throw std::runtime_error("BucketPriorityQueue::top/pop on empty");
        }
        if (cur_f_idx_ == -1) {
            advance_to_next_nonempty(0, 0);
        }
    }

    // 現在カーソルを、辞書式順序 (f,h) 最小の非空バケットへ進める。
    // h_start は同一 f 内での開始 h 位置。
    void advance_to_next_nonempty(int f_start, int h_start) {
        // まず f を前へ
        for (int fi = std::max(0, f_start); fi < F_; ++fi) {
            if (f_counts_[fi] == 0) continue; // この f は空

            // h の開始位置を決める
            int hi0 = (fi == f_start) ? std::max(0, h_start) : 0;

            // 現 f レベルで非空 h を探す
            for (int hi = hi0; hi < H_; ++hi) {
                if (!buckets_[fi][hi].empty()) {
                    cur_f_idx_ = fi;
                    cur_h_idx_ = hi;

                    // 現 f レベルの非空セル残数を（ざっくり）数え直す
                    // ※ O(H) だが H が小さめなら十分実用。高速化したければ
                    //   各 (fi,hi) の非空→空/空→非空でカウンタを更新する設計に。
                    int cnt = 0;
                    for (int k = 0; k < H_; ++k) {
                        if (!buckets_[fi][k].empty()) ++cnt;
                    }
                    f_h_nonempty_left_ = cnt;
                    return;
                }
            }

            // この f に要素があるはずなのに見つからない？
            // → すべて他の場所に移動して 0 になっている可能性
            // 念のためクリア
            f_counts_[fi] = 0;
        }

        // ここに来るのは size_==0 のときのみのはず
        cur_f_idx_ = -1;
        cur_h_idx_ = -1;
    }
};