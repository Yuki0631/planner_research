#include "search.hpp"
#include "bucket_pq.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <functional>
#include <cassert>
#include <sstream>
#include <iostream>

namespace planner {

// 経路を復元する関数
std::vector<int> extract_plan(const std::vector<Node>& nodes, int goal_id) {
    std::vector<int> acts;
    for (int v = goal_id; v >= 0 && nodes[v].parent >= 0; v = nodes[v].parent) {
        acts.push_back(nodes[v].act_id);
    }
    std::reverse(acts.begin(), acts.end());
    return acts;
}

// plan cost の計算を行う関数
double eval_plan_cost(const StripsTask& st, const std::vector<int>& plan) {
    double c = 0.0;
    for (int a : plan) c += st.actions[a].cost; // action id を基に cost の計算を行う関数
    return c;
}

// plan を文字列にする関数
std::string plan_to_string(const StripsTask& st, const std::vector<int>& plan) {
    std::ostringstream oss;
    for (std::size_t i=0;i<plan.size();++i) {
        if (i) oss << "\n";
        int a = plan[i];
        oss << i << ": " << st.actions[a].name << " [cost=" << st.actions[a].cost << "]";
    }
    return oss.str();
}

std::string plan_to_val(const StripsTask& st, const std::vector<int>& plan) {
    std::ostringstream oss;
    for (size_t i = 0; i < plan.size(); ++i) {
        if (i) oss << "\n"; // 改行
        int a = plan[i];
        oss << st.actions[a].name; // action name を書き込む
    }
    double c = eval_plan_cost(st, plan);
    oss << "\n; cost = " << std::setprecision(17) << c << "\n";
    return oss.str();
}

// cost がすべて整数かどうかを判定する関数
bool all_action_costs_are_integers(const planner::StripsTask& st,
                                          double eps)
{
    for (const auto& a : st.actions) {
        // Nan や inf 対策
        if (!std::isfinite(a.cost)) return false;

        // 端数をチェック
        double nearest = std::round(a.cost);
        if (std::fabs(a.cost - nearest) > eps) return false;
    }
    return true;
}

// f, h 値を丸める関数 all_action_costs_are_integers を通過したものを入れるとする
int rounding(double v) {
    long long k = std::llround(v); // 丸めて整数へ
    if (k < 0) throw std::runtime_error("negative value not supported");
    return static_cast<int>(k);
}


// --- A* Search ---

SearchResult astar(const StripsTask& st, HeuristicFn h, const SearchParams& p) {
    SearchResult R; // 結果を格納する用の R

    // 初期ノード
    StripsState s0 = make_init_state(st);
    R.nodes.push_back(Node{ s0, -1, -1 });

    // 既にゴール
    if (is_goal(st, s0)) { 
        R.solved = true;
        R.plan.clear();
        R.plan_cost = 0.0;
        return R;
    }

    // state -> node_id にするマップ
    std::unordered_map<StripsState,int,StripsStateHash> index_of;

    if (all_action_costs_are_integers(st)) {
        // デバッグ用
        std::cout << "Note: all action costs are integers; using integer A* + BucketPQ." << std::endl;

        // state -> node_id
        index_of.reserve(1 << 10);
        index_of.emplace(R.nodes[0].s, 0);

        // ノードの情報
        struct MetaI { int g; int h; bool closed; };
        std::unordered_map<int, MetaI> meta;
        meta.reserve(1 << 10);

        // OPEN リスト: pack_fh_asc(f,h) をキー、値はノード id
        BucketPQ open;

        // 初期ノードの登録
        const int h0 = rounding(h(st, s0)); 
        meta[0] = MetaI{0, h0, false};
        open.insert(0, pack_fh_asc(h0, h0));

        StripsState succ;

        while (!open.empty()) {
            // 最小値の取り出し
            auto [u32, key] = open.extract_min();
            const int u = static_cast<int>(u32);
            const int fu = unpack_f(key);
            const int hu = unpack_h(key);

            // 古い f, h 値の制御
            const int fu_now = meta[u].g + meta[u].h;
            if (fu != fu_now || hu != meta[u].h) {
                continue;
            }

            StripsState su = R.nodes[u].s;

            // ゴール判定
            if (is_goal(st, su)) {
                R.solved = true;
                R.plan = extract_plan(R.nodes, u);
                R.plan_cost = eval_plan_cost(st, R.plan);
                return R;
            }

            // CLOSE
            meta[u].closed = true;

            ++R.stats.expanded;
            if (R.stats.expanded > p.max_expansions) break;

            // 展開
            for (int a = 0; a < (int)st.actions.size(); ++a) {
                const auto& act = st.actions[a];
                if (!is_applicable(st, su, act)) continue;

                apply(st, su, act, succ); // 状態遷移
                ++R.stats.generated;

                const int w = rounding(act.cost);
                const int tentative_g = meta[u].g + w;

                auto it = index_of.find(succ);
                if (it == index_of.end()) { // 新規ノードの場合
                    const int v = (int)R.nodes.size();
                    R.nodes.push_back(Node{succ, u, a});
                    index_of.emplace(R.nodes[v].s, v);

                    const int hv = rounding(h(st, R.nodes[v].s));
                    meta[v] = MetaI{tentative_g, hv, false};

                    const UKey new_key = pack_fh_asc(tentative_g + hv, hv);
                    open.insert(static_cast<BucketPQ::Value>(v), new_key);
                } else { // 既存ノードの場合
                    const int v = it->second;

                    if (tentative_g < meta[v].g) { // g 値が改善される場合
                        meta[v].g = tentative_g;
                        R.nodes[v].parent = u;
                        R.nodes[v].act_id = a;

                        meta[v].h = rounding(h(st, R.nodes[v].s));
                        const UKey new_key = pack_fh_asc(meta[v].g + meta[v].h, meta[v].h);

                        if (meta[v].closed) { // ノードがクローズドリストに含まれる場合
                            // 再オープンできない場合
                            if (!p.reopen_closed) {
                                ++R.stats.duplicates;
                                continue;
                            }
                            meta[v].closed = false;
                            open.insert(static_cast<BucketPQ::Value>(v), new_key);
                        } else { // ノードがクローズドリストに含まれない場合
                            if (open.contains(static_cast<BucketPQ::Value>(v))) { // ノードがオープンリストに登録されている場合
                                const auto cur_key = open.key_of(static_cast<BucketPQ::Value>(v));
                                if (new_key < cur_key) { // 新しい Key が現在の Key よりも小さい場合
                                    open.decrease_key(static_cast<BucketPQ::Value>(v), new_key);
                                } else if (new_key > cur_key) { // A* では発生しないが、念のために
                                    open.increase_key(static_cast<BucketPQ::Value>(v), new_key);
                                }
                            } else { // ノードがオープンリストに登録されていない場合 (削除された場合などの対策)
                                open.insert(static_cast<BucketPQ::Value>(v), new_key);
                            }
                        }
                    } else { // g 値の改善がない場合
                        ++R.stats.duplicates;
                        if (meta[v].closed && !p.reopen_closed) continue;
                    }
                }
            }
        }
        return R;

    } else { // コストが浮動小数点を含む場合
        // デバッグ用
        std::cout << "Note: action costs are not all integers; using non-integer A* search." << std::endl;
        
        // 初期設定
        index_of.reserve(1024); // あらかじめ 2^10 個を予約しておく
        index_of.emplace(R.nodes[0].s, 0); // 初期状態を value = 0 として登録する

        struct MetaD { double g; double h; bool closed; };
        std::unordered_map<int, MetaD> meta; 
        meta.reserve(1024);
        // open list
        struct QEl { double f; double h; int id; }; // Queue Element の定義
        auto cmp = [](const QEl& a, const QEl& b){
            if (a.f != b.f) return a.f > b.f;   
            return a.h > b.h; // f 値が同じ場合は, h 値が小さいものを選択する           
        };
        std::priority_queue<QEl, std::vector<QEl>, decltype(cmp)> open(cmp);

        // 初期ノード
        meta[0] = MetaD{0.0, h(st, s0), false};
        open.push({ meta[0].g + meta[0].h, meta[0].h, 0 });

        StripsState succ;
        constexpr double EPS = 1e-12; // 誤差項

        while (!open.empty()) {
            // node の展開
            QEl cur = open.top();
            open.pop();
            const int u = cur.id;

            // 古い f 値を持つノードはスキップする
            const double fu_now = meta[u].g + meta[u].h;
            if (std::fabs(cur.f - fu_now) > EPS) continue;

            StripsState su = R.nodes[u].s;

            // ゴール判定
            if (is_goal(st, su)) {
                R.solved = true;
                R.plan = extract_plan(R.nodes, u);
                R.plan_cost = eval_plan_cost(st, R.plan);
                return R;
            }

            // close
            meta[u].closed = true;

            ++R.stats.expanded;
            if (R.stats.expanded > p.max_expansions) break; // ノードの展開上限数を越したら

            for (int a = 0; a < (int)st.actions.size(); ++a) {
                const auto& act = st.actions[a];
                if (!is_applicable(st, su, act)) continue;

                apply(st, su, act, succ);
                ++R.stats.generated;

                const double tentative_g = meta[u].g + act.cost;

                auto it = index_of.find(succ);
                if (it == index_of.end()) { // 新規ノードの場合
                    const int v = (int)R.nodes.size(); // 新しい id の生成
                    R.nodes.push_back(Node{succ, u, a});
                    index_of.emplace(R.nodes[v].s, v);

                    const double hv = h(st, R.nodes[v].s);
                    meta[v] = MetaD{tentative_g, hv, false};
                    open.push({ tentative_g + hv, hv, v });
                } else { // 既存ノードの場合
                    const int v = it->second;

                    // g 値が改善できたら更新する
                    if (tentative_g + EPS < meta[v].g) {
                        meta[v].g = tentative_g;
                        R.nodes[v].parent = u;
                        R.nodes[v].act_id = a;

                        meta[v].h = h(st, R.nodes[v].s);

                        // 再オープン可否
                        if (meta[v].closed && !p.reopen_closed) { // 閉じたままで改善はしない
                            ++R.stats.duplicates;
                            continue;
                        }

                        // open に再投入（古い f は f 値チェックで自然に捨てられる）
                        open.push({ meta[v].g + meta[v].h, meta[v].h, v });
                    } else {
                        // closed かつ再オープン禁止ならスキップ
                        if (meta[v].closed && !p.reopen_closed) {
                            ++R.stats.duplicates;
                            continue;
                        }
                        // 再オープン許可でも改善なしならスキップ
                        ++R.stats.duplicates;
                        }
                    }
            }
        }
        return R;
    }

}

} // namespace planner
