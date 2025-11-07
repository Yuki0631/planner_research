#include "sas/parallel_SOC/parallel_search.hpp"
#include <thread>
#include <atomic>
#include <limits>
#include <unordered_map>

// コンパイラに対するヒントのためのマクロ変数
#define likely(x)   __builtin_expect(!!(x), 1) // よく起こりやすい条件分岐
#define unlikely(x) __builtin_expect(!!(x), 0) // あまり起きない条件分岐

namespace planner {
namespace sas {
namespace parallel_SOC {

// 適用したactionを構築する関数
static std::vector<uint32_t> reconstruct_plan(const std::unordered_map<uint64_t, Node>& nodes, uint64_t goal_id)
{
    std::vector<uint32_t> ops; // 演算子を積むためのベクタ
    uint64_t cur = goal_id;
    while (true) {
        auto it = nodes.find(cur);
        if (it == nodes.end()) { // ハッシュマップで指定の id が見つからなかった場合
            break;
        }
        const Node& n = it->second;
        if (n.parent == UINT64_MAX) { // もし親が規定値ならば (スタートノードならば)
            break;
        }
        ops.push_back(n.op_id); // 演算子をベクタに積む
        cur = n.parent; // 現在見ているノード id を更新する
    }
    std::reverse(ops.begin(), ops.end()); // 最初から最後にするため、std::reverseを用いる
    return ops; // プランを返す
}

// A* 探索の主要部分
SearchResult astar_soc(const sas::Task& T, const SearchParams& P, planner::sas::soc::GlobalStats* stats_out) {
    const uint32_t N = P.num_threads ? P.num_threads : 1; // スレッドの数
    const uint32_t Q = P.num_queues ? P.num_queues : N; // Queue の数、なければスレッド数と一致させる
    const uint32_t Sh = P.num_bucket_shards ? P.num_bucket_shards : N; // 二段バケットのシャードの数、なければスレッド数と一致させる
    const uint32_t K = P.num_k_select ? P.num_k_select : 2; // 二段バケットの k-choice の選択数パラメタ

    IdAllocator ids; // ID 生成器
    Heuristic hfn = Heuristic::goalcount(); // ヒューリスティック関数
    ClosedTable closed(std::max<uint32_t>(1024, N*64)); // クローズドリスト
    SharedOpen open(P.open_kind, Q, Sh, K); // オープンリスト
    Termination term(P.time_limit_ms); // 時間制限

    // 統計値の取得
    planner::sas::soc::GlobalStats GS;
    GS.resize(N); // スレッドの数だけ容量を確保する
    open.set_stats(&GS); // オープンリストに統計情報を書き込むための struct のポインタを渡す

    // スレッド ID の初期化
    planner::sas::soc::set_current_thread_index(0);

    StateStore store(std::max<uint32_t>(2048, N*128)); // ID と状態を対応させるハッシュマップ、2048 と N*128 で大きい方が分割数となる

    // 解の復元のための、遭遇したノードを記録するテーブル
    planner::sas::soc::TicketLock reg_mtx; // テーブルのためのロック (軽量 FIFO ロック)
    std::unordered_map<uint64_t, Node> registry; // ID と ノードを対応させるハッシュマップ
    registry.reserve(1<<24); // あらかじめ 2^24 の要素を確保しておく

    // ルートノード
    Node root;
    root.id = ids.alloc();
    root.g = 0;
    uint64_t first_relax_eval_ns = planner::sas::soc::measure_ns_and_run([&](){
                        root.h = hfn(T, T.init);
                    });
    root.op_id  = std::numeric_limits<uint32_t>::max();
    root.parent = std::numeric_limits<uint64_t>::max();

    store.put(root.id, T.init); // ルートノードの ID と state をマップに挿入する
    closed.prune_or_update(T.init, root.g, root.id);

    // critical section (ノード記録表に登録)
    {
        planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(reg_mtx); // ロックをかける
        registry.emplace(root.id, root);
    }

    open.push(0, std::move(root)); // オープンリストに push する

    std::atomic<bool> done{false}; // 探索が終了しているか表すフラグ、複数のスレッドに共有するので、std::atomic を使用する
    std::atomic<uint64_t> goal_node{UINT64_MAX}; // goal node の ID を記録するための atomic 変数

    auto worker = [&](uint32_t tid){
        planner::sas::soc::set_current_thread_index(tid); // 現在のスレッドの ID を登録する
        sas::State cur_state; // 現在の state
        auto& S = GS.per_thread[tid]; // 各スレッドごとの統計値を取得する

        while (!done.load(std::memory_order_acquire)) { // 探索が終了しない限り
            if (unlikely(term.timed_out())) { // 時間制限を超えてしまった場合
                done.store(true);
                break;
            }

            auto item = open.pop(tid);
            if (unlikely(!item.has_value())) { // オープンリストから取り出したノードが無効値の場合
                if (open.empty()) { // オープンリスト (全体) が空ならば
                    break;
                }
                std::this_thread::yield(); // 他のスレッドに譲る
                continue;
            }

            Node cur = std::move(*item); // 現在のノードを更新する
            S.expanded++; // expansion を 1 インクリメントする

            // ハッシュマップから state を得ることが出来ない場合
            if (unlikely(!store.get(cur.id, cur_state))) { // 通常あり得ないが、コンパイルエラー防止のため
                continue;
            }

            // ゴール判定
            bool is_goal = true; // ゴールかどうか表すフラグ
            for (auto [v,val] : T.goal) { // goal state に含まれるすべての変数 id とその domain-value
                if (cur_state[v] != val) { // 現在の state とゴールの state の変数の domain-value が一致しない場合
                    is_goal=false;
                    break;
                }
            }

            if (unlikely(is_goal)) { // ゴール条件が満たされているのなら
                goal_node.store(cur.id, std::memory_order_release); // ゴールノードに現在の ID を入れる
                done.store(true, std::memory_order_release); // 探索終了を表す atomic flag の値を true にする
                break;
            }

            Expander::for_each_inplace(T, cur_state, 
                [&](uint32_t op_id, int add_cost, const sas::State& succ) { // 参照で、インプレース変換された state を succ として別名で扱う
                    Node nxt; // 後継ノード
                    nxt.id = ids.alloc(); // ノード ID の割り振り
                    nxt.parent = cur.id; // 親ノード ID を繋ぐ
                    nxt.op_id = op_id;
                    nxt.g = cur.g + add_cost;
                    
                    // h-value の計算時間を測定しつつ算出する
                    S.relax_eval_ns += planner::sas::soc::measure_ns_and_run([&](){
                        nxt.h = hfn(T, succ);
                    });

                    // reopen 判定のために事前にクローズリストにノードが含まれているか確認する
                    auto prev = closed.get(succ);

                    if (closed.prune_or_update(succ, nxt.g, nxt.id)) { // g-value が悪化または同じである場合は、枝狩りを行う
                        S.duplicates_pruned++; // 枝狩りできたので、その統計値を 1 インクリメントする
                        return;
                    }

                    // クローズドリストに含まれていたが改善した場合
                    if (prev.has_value()) {
                        S.reopened++;
                    }

                    S.generated++; // 生成ノード数を 1 インクリメントする

                    store.put(nxt.id, succ); // state のコピーの作成

                    {
                        planner::sas::soc::ScopedLock<planner::sas::soc::TicketLock> lg(reg_mtx); // ロックをかける
                        registry.emplace(nxt.id, nxt); // ノードの登録
                    }

                    // オープンリストに挿入
                    open.push(tid, std::move(nxt));
                }
            );
        }
    };

    std::vector<std::thread> th; // 各スレッドを積んだベクタ
    th.reserve(N);

    for (uint32_t t=0; t<N; ++t) {
        th.emplace_back(worker, t); // それぞれのスレッドに仕事を渡す
    }

    for (auto& x : th) {
        x.join(); // 全スレッドを呼出し、それらすべてが終了するまで待機する
    }

    SearchResult R;

    if (stats_out) { // 統計値が書き込まれている場合
        // 初期ノードのヒューリスティック関数の呼び出しを考慮する
        stats_out->per_thread[0].evaluated += 1;
        stats_out->per_thread[0].relax_eval_ns += first_relax_eval_ns;

        *stats_out = GS; // GS を stats_out にコピー代入し、呼び出し側に結果を渡す
    }

    if (goal_node.load() != UINT64_MAX) { // ゴールノードが発見された場合
        R.solved = true;
        auto plan = reconstruct_plan(registry, goal_node.load());
        R.plan_ops = std::move(plan);
        int cost = 0;
        for (auto oi : R.plan_ops) {
            cost += T.ops[oi].cost;
        }
        R.cost = cost;
    }

    return R;
}

} // namespace parallel_SOC
} // namespace sas
} // namespace planner
