#include "search.hpp"
#include "bucket_pq.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <functional>
#include <cassert>
#include <sstream>

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

// f 値を丸める関数 all_action_costs_are_integers を通過したものを入れるとする (Nan\inf 対策は不要)
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

    // 初期設定
    index_of.reserve(1024); // あらかじめ 2^10 個を予約しておく
    index_of.emplace(R.nodes[0].s, 0); // 初期状態を value = 0 として登録する


    if (all_action_costs_are_integers(st)) { // int だけの場合
        // closed_list
        struct Meta { int g; int h; };
        std::unordered_map<u_int64_t, Meta> meta; // node id で g, h 値を管理する
        meta.reserve(1024);

        int h0 = rounding(h(st, s0));
        meta[0] = {0, h0}; // 初期値の代入

        // OpenList
        BucketPriorityQueue<NodeStats> open(0, 256, 0, 256); // node id を key (value) のように扱う、オープンリスト
        open.push({0, h0}, h0, h0);

        StripsState succ; // 後継ノード

        while (!open.empty()) {
            auto cur = open.top();
            open.pop();
            int u = cur.id; // ノード id
            int fu = cur.f; // f 値

            // f 値チェック
            if (fu != (meta[u].g + meta[u].h)) continue; // 古い f 値を持つ open list に含まれる node はスキップする

            const StripsState& su = R.nodes[u].s;

            // ゴール判定 (pop 時に行う)
            if (is_goal(st, su)) {  
                R.solved = true;
                R.plan  = extract_plan(R.nodes, u);
                R.plan_cost = eval_plan_cost(st, R.plan);
                return R;
            }

            // 展開
            ++R.stats.expanded;
            if (R.stats.expanded > p.max_expansions) break; // 最大展開ノード数を超えた場合

            for (int a = 0; a < (int)st.actions.size(); ++a) {
                const auto& act = st.actions[a];
                if (!is_applicable(st, su, act)) continue; // 現在の state に適用できないなら continue

                apply(st, su, act, succ); // action を適用する
                ++R.stats.generated;

                int tentative_g = meta[u].g + static_cast<int>(act.cost);

                auto it = index_of.find(succ); // 後継ノードの状態のイテレータを探す

                if (it != index_of.end()) { // 同一状態がある場合は、その g 値同士を比較する
                    int succ_id = it -> second;
                    if (tentative_g >= meta[succ_id].g && !p.reopen_closed) { // g 値が大きい かつ closed を reopen しない場合
                        ++R.stats.duplicates;
                        continue;
                    } else { // g 値が小さい または closed を常に reopen する場合
                        int hv = rounding(h(st, succ));
                        int fv = tentative_g + hv;
                        meta[succ_id] = {tentative_g, hv}; // closed の更新
                        R.nodes[succ_id].parent = u;
                        R.nodes[succ_id].act_id = a;
                        open.push({succ_id, fv}, fv, hv);
                    }
                } else { // 新規ノードの場合
                    int v = (int)R.nodes.size();
                    R.nodes.push_back(Node{succ, u, a}); // 後継ノードの状態, 親 id, Action を入れる 
                    index_of.emplace(R.nodes[v].s, v); // map に状態と id の pair を登録する
                    int hv = rounding(h(st, succ));
                    int fv = tentative_g + hv;
                    meta[v] = { tentative_g, hv }; // closed_list の更新
                    open.push({v, fv}, fv, hv);
                }
            }
        }
        return R; 
    } else { // double の場合
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

            const StripsState& su = R.nodes[u].s;

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
