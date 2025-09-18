#pragma once
#include <cstddef>
#include <climits>
#include <type_traits>
#include <new>
#include <utility>
#include <cassert>

using UKey = uint32_t; 
static constexpr int  H_BITS = 16; // 32bit の半分の 16bit
static constexpr UKey H_MASK = (UKey(1) << H_BITS) - 1; // 0...01...1 (16bit ずつ)

// f, h 値をパックする関数、h 値は昇順
inline UKey pack_fh_asc(int f, int h) {
    if (f < 0) f = 0;
    if (h < 0) h = 0;
    return (UKey(f) << H_BITS) | (UKey(h) & H_MASK); // 上位 16bit は f 値、下位 16bit は h 値
}

//　f, h 値をパックする関数、h 値は降順、 H_MAX はドメイン依存の h 値の上界
inline UKey pack_fh_desc(int f, int h, int H_MAX) {
    if (f < 0) f = 0;
    if (h < 0) h = 0;
    int h_rev = H_MAX - h; // h_reverse は、 h 値が大きいほど小さくなる
    if (h_rev < 0) h_rev = 0;
    return (UKey(f) << H_BITS) | (UKey(h_rev) & H_MASK);
}

// アンパック関数
inline int unpack_f(UKey key) { return int(key >> H_BITS); } // 16bit だけ右にシフトする (上位 16bit を取り出す)
inline int unpack_h(UKey key) { return int(key & H_MASK); } // 下位 16bit を取り出す


// --- 動的配列のテンプレートクラス ---
template <class T>
class DynamicArray {
public:
    // コンストラクタ
    DynamicArray() noexcept : data_(nullptr), size_(0), cap_(0) {}

    // デストラクタ
    ~DynamicArray() { destroy_all_();
        ::operator delete[](data_); }
    
    // コピーコンストラクタ
    DynamicArray(const DynamicArray& o) : data_(nullptr), size_(0), cap_(0) {
        reserve(o.size_);
        for (u_int32_t i = 0; i < o.size_; ++i) {
            new (data_ + i) T(o.data_[i]); // 直接メモリ上に書き込む
        }
        size_ = o.size_;
    }

    // ムーブコンストラクタ
    DynamicArray(DynamicArray&& o) noexcept : data_(o.data_), size_(o.size_), cap_(o.cap_) {
        o.data_ = nullptr;
        o.size_ = o.cap_ = 0;
    }

    // 代入演算子
    DynamicArray& operator=(DynamicArray o) noexcept(std::is_nothrow_move_constructible<T>::value) {
        swap(o);
        return *this;
    }

    // swap 関数
    void swap(DynamicArray& o) noexcept {
        using std::swap; swap(data_, o.data_);
        swap(size_, o.size_);
        swap(cap_, o.cap_);
    }

    // size() 関数
    u_int32_t size() const noexcept { return size_; }

    // capacity() 関数
    u_int32_t capacity() const noexcept { return cap_; }

    // empty() 関数
    bool empty() const noexcept { return size_ == 0; }

    // インデックス演算子 (読み書き可能)    
    T& operator[](u_int32_t i) {
        assert(i < size_);
        return data_[i];
    }

    // インデックス演算子 (読み込み専用)
    const T& operator[](u_int32_t i) const {
        assert(i < size_);
        return data_[i];
    }

    // clear() 関数
    void clear() noexcept {
        for (u_int32_t i = 0; i < size_; ++i) data_[i].~T();
        size_ = 0;
    }

    // サイズを縮小させる関数
    void shrink_to_fit() { if (cap_ != size_) reallocate_(size_); }

    // 配列のサイズを変更する関数
    void resize(u_int32_t n) {
        if (n < size_) { // サイズが大きい場合
            for (u_int32_t i = n; i < size_; ++i) data_[i].~T();
            size_ = n;
            return;
        }

        if (n > cap_) reallocate_(grow_to_(n)); // capacity が小さい場合

        // サイズが小さい場合
        for (u_int32_t i = size_; i < n; ++i) new (data_ + i) T(); // 空のオブジェクトをメモリ上に配置する
        size_ = n;
    }

    // メモリの確保
    void reserve(u_int32_t n) { if (n > cap_) reallocate_(grow_to_(n)); }

    // emplace_back() 関数
    template <class... Args>
    T& emplace_back(Args&&... args) {
        if (size_ == cap_) reallocate_(next_cap_()); // 容量が満杯の場合
        new (data_ + size_) T(std::forward<Args>(args)...);
        return data_[size_++];
    }

    // push_back 関数 (const と move どちらにも対応)
    void push_back(const T& v) { (void)emplace_back(v); }
    void push_back(T&& v) { (void)emplace_back(std::move(v)); }

    // pop_back() 関数
    void pop_back() {
        assert(size_);
        data_[--size_].~T();
    }

private:
    T* data_;
   u_int32_t size_, cap_;

    // 必要な容量を返す関数
    static u_int32_t grow_to_(u_int32_t need) {
        u_int32_t c = cap_hint_(need);
        return c;
    }

    // 次の容量を返す関数
    u_int32_t next_cap_() const {
        return cap_ ? (cap_ << 1) : u_int32_t(2); // cap_ が存在すれば 2 倍にし、存在しなければ 2 を返す
    }

    // 適切な容量を返す関数
    static u_int32_t cap_hint_(u_int32_t n) {
        u_int32_t c = 1;
        while (c < n) c <<= 1; // n 以上の中で最も小さい 2 のべき乗を取得する
        return c;
    }

    // メモリ上に容量を確保する関数
    void reallocate_(u_int32_t new_cap) {
        T* mem = static_cast<T*>(::operator new[](new_cap * sizeof(T))); // 必要なメモリを計算し、配列のポインタを取得する
        u_int32_t i = 0;
        try {
            if constexpr (std::is_nothrow_move_constructible<T>::value || !std::is_copy_constructible<T>::value) { // ムーブができるまたはコピーでない場合
                for (; i < size_; ++i) new (mem + i) T(std::move(data_[i]));
            } else {
                for (; i < size_; ++i) new (mem + i) T(data_[i]);
            }
        } catch (...) {
            for (u_int32_t j = 0; j < i; ++j) mem[j].~T();
            ::operator delete[](mem);
            throw;
        }
        
        // 元の配列の削除
        for (u_int32_t j = 0; j < size_; ++j) data_[j].~T();
        ::operator delete[](data_);

        // 配列の先頭アドレスと、容量の更新
        data_ = mem;
        cap_ = new_cap;
    }

    // 配列自体を削除する関数
    void destroy_all_() noexcept {
        for (u_int32_t i = 0; i < size_; ++i) data_[i].~T();
        size_ = 0;
        cap_ = 0;
    }

};

// --- バケット Priority Queue ---
class BucketPQ {
public:
    using Value = u_int32_t;
    using Key = UKey;  

    // コンストラクタ
    BucketPQ() : min_key_(UINT32_MAX), count_(0) {}

    // 空かどうか確認する関数
    bool empty() const noexcept { return count_ == 0; }

    // サイズを返す関数
    u_int32_t size() const noexcept { return count_; }

    // insert 関数
    void insert(Value v, Key k) {
        if (k < 0) k = 0;

        // Key と Value がコンテナに入るように調整する
        ensure_buckets_(static_cast<u_int32_t>(k));
        ensure_pos_(v);


        auto &p = pos_[static_cast<u_int32_t>(v)];
        assert(!p.present && "insert: value already present. Use decrease/increase/remove first.");
        auto &b = buckets_[static_cast<u_int32_t>(k)];

        p.key = k;
        p.idx = b.size();
        p.present = true;
        b.push_back(v);

        if (k < min_key_) min_key_ = k;
        ++count_;
    }

    // 最小キー要素を1つ取り出す関数、（ノード id, キー）を返す
    std::pair<Value, Key> extract_min() {
        assert(count_ > 0);
        advance_min_();
        auto &b = buckets_[static_cast<u_int32_t>(min_key_)];

        const u_int32_t last = b.size() - 1;
        Value v = b[last]; // 最小のキーと対応する Value を取得する
        b.pop_back();

        auto &p = pos_[static_cast<u_int32_t>(v)];
        p.present = false;
        --count_;

        // バケットが空になったら次の最小値を前倒して探索する
        if (b.empty()) advance_min_();
        return {v, p.key};
    }

    // キーを小さくする関数
    void decrease_key(Value v, Key new_key) {
        change_key_(v, new_key, /*allow_increase=*/false);
    }

    // キーを大きくする関数
    void increase_key(Value v, Key new_key) {
        change_key_(v, new_key, /*allow_increase=*/true);
    }

    // 値 v が入っているか確認数る関数
    bool contains(Value v) const {
        if (v < 0 || static_cast<u_int32_t>(v) >= pos_.size()) return false;
        return pos_[static_cast<u_int32_t>(v)].present;
    }

    // 値 v を削除する関数
    void remove(Value v) {
        if (!contains(v)) return;

        auto &p = pos_[static_cast<u_int32_t>(v)];
        auto &b = buckets_[static_cast<u_int32_t>(p.key)];

        const u_int32_t last = b.size() - 1;
        if (p.idx != last) {
            Value moved = b[last];
            b[p.idx] = moved;
            b.pop_back();
            pos_[static_cast<u_int32_t>(moved)].idx = p.idx;
        } else {
            b.pop_back();
        }
        p.present = false;
        --count_;
        if (b.empty() && p.key == min_key_) advance_min_();
    }

    // 現在のキーを取得する関数
    Key key_of(Value v) const {
        if (!contains(v)) return -1;
        return pos_[static_cast<u_int32_t>(v)].key;
    }

    // PQ バケットを削除する関数
    void clear() {
        for (u_int32_t i = 0; i < buckets_.size(); ++i) buckets_[i].clear();
        for (u_int32_t i = 0; i < pos_.size(); ++i) pos_[i] = Pos{};
        buckets_.clear();
        pos_.clear();
        min_key_ = UINT32_MAX;
        count_ = 0;
    }

private:
    // Position Structure
    struct Pos {
        Key key = -1;
        u_int32_t idx = 0;
        bool present = false; // value のインデックスが存在しているかどうかのフラグ
    };

    DynamicArray< DynamicArray<Value> > buckets_; // ノード id の動的配列を要素とする動的配列. buckets_[key] = [values...]
    DynamicArray<Pos> pos_; //Pos Structure を要素とする動的配列. pos_[value] = {key, idx, present}
    Key min_key_; // 現在の最小値を追跡する変数
    u_int32_t count_; // 要素の数をカウントする変数

    // バケットのサイズを確保する関数
    void ensure_buckets_(u_int32_t k) {
        if (k >= buckets_.size()) buckets_.resize(k + 1);
    }

    // ノードを管理するベクターのサイズを確保する関数
    void ensure_pos_(u_int32_t v) {
        if (v >= pos_.size()) pos_.resize(v + 1);
    }

    // 最小値の位置を更新する関数
    void advance_min_() {
        if (count_ == 0) { // 要素がない場合
            min_key_ = UINT32_MAX;
            return;
        }

        u_int32_t i = (min_key_ == UINT32_MAX) ? u_int32_t(0) : static_cast<u_int32_t>(min_key_);

        // もしバケットのサイズを超えていた場合
        if (i >= buckets_.size()) i = 0;

        // 次の要素があるところまで走査する
        for (; i < buckets_.size(); ++i) {
            if (!buckets_[i].empty()) {
                min_key_ = static_cast<Key>(i);
                return;
            }
        }
        // 念のため
        min_key_ = UINT32_MAX;
    }

    // Key を更新する関数
    void change_key_(Value v, Key new_key, bool allow_increase) {
        assert(contains(v) && "change_key_: value not present");
        if (new_key < 0) new_key = 0;

        // Value を基に Position Structure を取得する
        auto &p = pos_[static_cast<u_int32_t>(v)];

        // Key の増加が許されていない場合
        if (!allow_increase) { assert(new_key <= p.key && "decrease_key: new_key must be <= old"); }

        // 同一の Key である場合
        if (new_key == p.key) return;

        {
            auto &b = buckets_[static_cast<u_int32_t>(p.key)];
            const u_int32_t last = b.size() - 1;
            if (p.idx != last) { // 削除したい要素が末尾の要素でない場合
                Value moved = b[last];
                b[p.idx] = moved;
                pos_[static_cast<u_int32_t>(moved)].idx = p.idx;
                b.pop_back();
            } else {
                b.pop_back();
            }
        }

        // 新バケットに挿入
        ensure_buckets_(static_cast<u_int32_t>(new_key));
        auto &bn = buckets_[static_cast<u_int32_t>(new_key)];
        p.key = new_key;
        p.idx = bn.size();
        bn.push_back(v);

        // 新しい Key が最小値よりも小さい場合
        if (new_key < min_key_) min_key_ = new_key;
        // min_key_ がない場合
        if (buckets_[static_cast<u_int32_t>(min_key_)].empty()) advance_min_();
    }
};
