#pragma once
#include "grounding.hpp" 
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <ostream>

namespace planner {

// --- STRIPS ---

// SRIPPS Actions, id によって precondition および effect を管理
struct StripsAction {
    std::string name; // Action 名
    std::vector<int> pre_pos; // precondition (positive)  
    std::vector<int> pre_neg; // precondition (negative)  
    std::vector<int> add; // add effect     
    std::vector<int> del; // delete effect     
    double cost = 1.0; // action cost (あれば)         
};

// 命題事実の真偽ベクトル
struct StripsState {
    std::vector<std::uint64_t> bits; // 各要素に 64 個の fact の真偽値が入る
};

// STRIPS Task (Ground Task と同様なもの)
struct StripsTask {
    // facts
    std::vector<GroundAtom> facts; // GroundAtom の集合 (predicate id & args ids)
    std::vector<std::string> fact_names; // fact の名前の集合
    std::unordered_map<std::string,int> fid; // 名前と id (整数) の対応

    // operator
    std::vector<StripsAction> actions;

    // init & goal
    std::vector<int> init_true; // init で true の fact-id
    std::vector<int> goal_pos; // ゴールで true の fact-id
    std::vector<int> goal_neg; // ゴールで false の fact-id

    // 事実の数を数え上げる関数
    int num_facts() const {
        return static_cast<int>(facts.size()); 
    }
};


// --- 変換 & 基本演算 ---

// GroundTask -> StripsTask へのコンパイル (引数: GroundTask)
StripsTask compile_to_strips(const GroundTask& gt);

// 初期状態における真偽ベクトルの生成
StripsState make_init_state(const StripsTask& st);

// STRIPS Action が適応可能か判定する関数 (引数: StripsTask, StripsState, StripsAction (適用するもの) )
bool is_applicable(const StripsTask& st, const StripsState& s, const StripsAction& a);

// STRIPS Action を 適用する関数 (引数: StripsTask, StripsState (適用前) , StripsAction (適用するもの) , StripsState (適用後) )
void apply(const StripsTask& st, const StripsState& s, const StripsAction& a, StripsState& out);

// STRIPS State が Goal 状態か判定する関数 (引数: StripsTask, StripsState)
bool is_goal(const StripsTask& st, const StripsState& s);

// --- ユーティリティ関数（デバッグ表示・ハッシュ） ---

// 比較のための operator (状態が一致しているかどうか)
bool operator==(const StripsState& a, const StripsState& b);

// StripsState を ハッシュ値に変換する関数
struct StripsStateHash {
    std::size_t operator()(const StripsState& s) const; // ファンクタは未実装
};

// fact-id と GroundTask を受け取って、事実を文字列として表示する関数
std::string fact_to_string(const StripsTask& st, int fid, const GroundTask& gt);

// 状態を文字列として表示する関数 (max_items: 最大表示数)
std::string state_to_string(const StripsTask& st, const StripsState& s, const GroundTask& gt, int max_items = 32);

// ostream 出力
std::ostream& operator<<(std::ostream& os, const StripsState&);

// --- 差分適用用の関数群 ---
// Undo Structure
struct Undo {
    // 展開中に、ビットが変わった命題の id を入れるようのベクター
    std::vector<int> flipped;
};

// 現在の undo スタック位置をマークする関数
inline std::size_t undo_mark(const Undo& u) {
    return u.flipped.size();
}

// state に action を差分適用し、変化したビットを Undo Structure に入れる関数
void apply_inplace(const StripsTask& st, const StripsAction& a, StripsState& s, Undo& u);

// Undo スタックを mark まで巻き戻す関数 (state を元の状態に戻す)
void undo_to(StripsState& s, Undo& u, std::size_t mark);

} // namespace planner
