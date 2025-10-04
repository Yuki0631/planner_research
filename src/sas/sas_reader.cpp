#include "sas/sas_reader.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cctype>

namespace planner { namespace sas {

// 文字列型を整数型に変換する関数
static int to_int(const std::string& s) {
    try {
        return std::stoi(s);
    } catch (...) {
        throw std::runtime_error("SAS parse error: not an integer: " + s);
    }
}

// 前後の空白を無視して文字列を取り出す関数
static std::string trim(const std::string& s) {
    size_t a=0, b=s.size();
    while (a<b && std::isspace((unsigned char)s[a])) ++a; // 文字列の前の空白分 a をインクリメントする
    while (b>a && std::isspace((unsigned char)s[b-1])) --b; // 文字列の後ろの空白分 b をデクリメントする
    return s.substr(a,b-a);
}

// SAS 形式ファイルを読み取る関数
Task read_file(const std::string& path) {
    std::ifstream fin(path);
    if (!fin) throw std::runtime_error("cannot open SAS file: " + path);

    // 全て読み込んで行ポインタで進める
    std::vector<std::string> L;
    L.reserve(200000);
    for (std::string line; std::getline(fin, line); ) {
        L.push_back(trim(line)); // 空白を取り除いた行を L に積んでいく
    }

    // 予想した文字列と比較する関数
    auto expect = [&](size_t& i, const std::string& key){
        if (i>=L.size() || L[i] != key) {
            throw std::runtime_error("SAS parse error: expect '" + key + "' at line " + std::to_string(i));
        }
        ++i; // 行を一つ進める
    };

    // 最初の行からファイルを読み進めていく
    size_t i=0;
    Task T;
    expect(i, "begin_version"); // 最初はバージョン情報
    if (i>=L.size()) { // もし、ファイルが begin_version の直後で終わってしまっている場合
        throw std::runtime_error("unexpected EOF after begin_version");
    }
    T.version = to_int(L[i++]); // version 情報を整数型に変換し、その後タスク情報に挿入する 3(str) -> 3(int)
    expect(i, "end_version"); // end_version であることを確かめる

    expect(i, "begin_metric");
    if (i>=L.size()) {
        throw std::runtime_error("unexpected EOF after begin_metric");
    }
    T.metric = to_int(L[i++]);
    expect(i, "end_metric");

    // 変数は "N" の後に N 回 "begin_variable ... end_variable" が続く
    if (i>=L.size()) {
        throw std::runtime_error("unexpected EOF reading variable count");
    }
    int nvars = to_int(L[i++]);
    T.vars.reserve(nvars);

    // 変数の読み取り
    for (int v=0; v<nvars; ++v) {

        // structure example
        // begin_variable
        // var0 (variable name)
        // -1 (invalid value)
        // 2 (number of possible domain values)
        // Atom clear(pos-8-6)
        // NegatedAtom clear(pos-8-6)
        // end_variable

        expect(i, "begin_variable");
        if (i>=L.size()) {
            throw std::runtime_error("unexpected EOF in variable block");
        }
        Variable V;
        V.name = L[i++];
        if (i>=L.size()) {
            throw std::runtime_error("unexpected EOF in variable block");
        }
        to_int(L[i++]); // 無効値の箇所を文字列型から整数型へ変換する -1(str) -> -1(int)
        if (i>=L.size()) {
            throw std::runtime_error("unexpected EOF in variable block");
        }
        V.domain = to_int(L[i++]); // ドメインサイズを読み取る

        // ドメイン個数分の Atom/NegatedAtom のラベル行を読み飛ばす
        for (int k=0;k<V.domain;++k) {
            if (i>=L.size()) {
                throw std::runtime_error("unexpected EOF in variable atoms");
            }
            ++i;
        }
        expect(i, "end_variable");
        T.vars.push_back(V);
    }

    // 排他グループの読み取り
    auto parse_mutex_group = [&](size_t& i2){

        // Structure example
        // begin_mutex_group
        // 3 (グループに含まれるリテラルの数)
        // 24 3
        // 25 3 (var_id & domain_value)
        // 7 0
        // end_mutex_group

        expect(i2, "begin_mutex_group");
        if (i2>=L.size()) {
            throw std::runtime_error("unexpected EOF in mutex group");
        }
        int k = to_int(L[i2++]); // グループ内のリテラルの個数
        MutexGroup G;
        G.lits.reserve(k);
        for (int t=0; t<k; ++t) {
            if (i2>=L.size()) {
                throw std::runtime_error("unexpected EOF in mutex rows");
            }
            std::istringstream iss(L[i2++]);
            int var, val;
            if (!(iss>>var>>val)) { // 文字列ストリームから、二つの整数値を抽出できなかった場合
                throw std::runtime_error("bad mutex row");
            }
            G.lits.emplace_back(var,val);
        }
        expect(i2, "end_mutex_group");
        T.mutexes.push_back(std::move(G)); // タスクに排他グループの情報を挿入する
    };

    // 件数が明示されている場合
    if (i < L.size() && L[i] != "begin_state") {
        bool consumed_count = false;
        try {
            int mcount = to_int(L[i]); // 数なら件数とみなす
            ++i;
            consumed_count = true;
            T.mutexes.reserve(mcount);
            for (int m=0; m<mcount; ++m) {
                parse_mutex_group(i);
            }
        } catch (...) { /* 数でなければスルー */ }
        // 件数なしで連続している場合
        while (i < L.size() && L[i] == "begin_mutex_group") {
            parse_mutex_group(i);
        }
    }    

    // 初期状態の読み取り

    // Sturucture example
    // begin_state
    // 1
    // 0       (各変数の初期値)
    // 2
    // :
    //end_state

    expect(i, "begin_state");
    T.init.resize(nvars);
    for (int v=0; v<nvars; ++v) {
        if (i>=L.size()) {
            throw std::runtime_error("unexpected EOF in begin_state");
        }
        T.init[v] = to_int(L[i++]); // 0..domain-1
    }
    expect(i, "end_state");

    // ゴール状態の読み取り

    // Structure example
    // begin_goal
    // 2      (ゴール条件の数)
    // 0 0    (変数 0 が 0)
    // 1 1    (変数 1 が 1) 
    // end_goal

    expect(i, "begin_goal");
    if (i>=L.size()) {
        throw std::runtime_error("unexpected EOF in begin_goal");
    }
    int g = to_int(L[i++]);
    T.goal.reserve(g);
    for (int t=0;t<g;++t) {
        if (i>=L.size()) {
            throw std::runtime_error("unexpected EOF in goal rows");
        }
        std::istringstream iss(L[i++]);
        int var,val;
        if (!(iss>>var>>val)) {
            throw std::runtime_error("bad goal row");
        }
        T.goal.emplace_back(var,val);
    }
    expect(i, "end_goal");

    // 残りの演算子部分の読み取り (ファイル末尾に数字 (0) あり)

    // Structure example
    // begin_operator
    // move A B      (operator name)
    // 0             (prevail 条件数)
    // 3             (効果の数)
    // 1 0 1 0 1 0
    // 1 0 2 0 2 1   (効果の条件数, 条件, 効果)
    // 1 0 3 0 3 2
    // 1             (コスト)
    // end_operator

    while (i < L.size()) {
        if (L[i] == "begin_operator") {
            ++i;
            if (i>=L.size()) {
                throw std::runtime_error("unexpected EOF after begin_operator");
            }
            Operator op;
            op.name = L[i++]; 

            if (i>=L.size()) {
                throw std::runtime_error("unexpected EOF in operator (k)");
            }
            int k = to_int(L[i++]);
            op.prevail.reserve(k);

            // prevail 条件の挿入 (var==val)
            for (int t=0;t<k;++t) {
                if (i>=L.size()) {
                    throw std::runtime_error("unexpected EOF in prevail rows");
                }
                std::istringstream iss(L[i++]);
                int var,val;
                if (!(iss>>var>>val)) {
                    throw std::runtime_error("bad prevail row");
                }
                op.prevail.emplace_back(var,val);
            }

            if (i>=L.size()) {
                throw std::runtime_error("unexpected EOF in operator (l)");
            }
            int l = to_int(L[i++]); 
            op.pre_posts.reserve(l);

            // 効果の挿入 (条件数, 条件, 効果)
            for (int t=0;t<l;++t) {
                if (i>=L.size()) {
                    throw std::runtime_error("unexpected EOF in pre_post row");
                }
                std::istringstream iss(L[i++]);
                int c;
                if (!(iss>>c)) {
                    throw std::runtime_error("bad pre_post row (c)");
                }
                std::vector<Operator::Cond> conds;
                conds.reserve(c);

                // 効果の条件の挿入
                for (int r=0;r<c;++r) {
                    int cv, cval;
                    if (!(iss>>cv>>cval)) {
                        throw std::runtime_error("bad pre_post row (cond pair)");
                    }
                    conds.emplace_back(cv,cval);
                }
                int var, pre, post;

                // 効果の挿入
                if (!(iss>>var>>pre>>post)) {
                    throw std::runtime_error("bad pre_post row (var/pre/post)");
                }
                op.pre_posts.emplace_back(std::move(conds), var, pre, post);
            }

            if (i>=L.size()) {
                throw std::runtime_error("unexpected EOF in operator (cost)");
            }

            op.cost = to_int(L[i++]);

            if (i>=L.size() || L[i] != "end_operator") {
                throw std::runtime_error("SAS parse error: missing end_operator");
            }
            ++i;

            T.ops.push_back(std::move(op));
        } else if (L[i].empty()) { // 空の行の場合
            ++i; // 行を一つ読み飛ばす
        } else { // 余白や末尾の数字などの場合
            ++i;
        }
    }
    return T;
}

}} // namespace planner::sas

namespace planner { namespace sas {

// 排他グループに反しているかどうか判定する関数
bool violates_mutex(const Task& T, const State& s) {
    for (const auto& G : T.mutexes) {
        int cnt = 0; // グループごとにカウンタは初期化する
        for (auto [v,val] : G.lits) {
            if ((int)s.size() <= v) { // 変数 id が状態に含まれる変数よりも大きい場合
                continue;
            }
            if (s[v] == val) { // mutex グループに含まれている変数の値がある場合
                if (++cnt > 1) {
                    return true; // 2 つ以上の排他グループに含まれているリテラルが満たされている場合
                }
            }
        }
    }
    return false;
}
}} // namespace planner::sas
