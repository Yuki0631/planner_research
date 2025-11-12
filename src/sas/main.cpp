#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <stdexcept>
#include <filesystem>
#include <sstream>
#include <functional>
#include <fstream>
#include <cmath>

#include "sas/sas_reader.hpp"
#include "sas/sas_search.hpp"
#include "sas/sas_heuristic.hpp"

#include "sas/parallel_SOC/parallel_search.hpp"

#include <csignal>
#if defined(__linux__)
   #include <sys/resource.h>
   #include <sys/time.h>
#endif

#include "sas/parallel_SOC/stats.hpp"

namespace fs = std::filesystem;

namespace planner {namespace sas{
    extern int g_mutex_mode;
    enum { MUTEX_AUTO = 0, MUTEX_ON = 1, MUTEX_OFF = 2 };
}}

static std::string shell_quote(const std::string& s) {
    std::ostringstream o;
    o << "'";
    for (char c : s) {
        if (c == '\'') o << "'\\''";
        else o << c;
    }
    o << "'";
    return o.str();
}

int main(int argc, char** argv) {
    // 使い方
    // planner_from_pddl <domain.pddl> <problem.pddl>
    //   [--only-search]
    //   [--algo astar|gbfs|soc_astar]
    //   [--search-cpu-limit int(second)]
    //   [--search-mem-limit-mb int(MB)]
    //   [--fd containers/fast-downward.sif]
    //   [--sas-file sas/output.sas]
    //   [--h goalcount|blind]
    //   [--keep-sas]
    //   [--plan-out plans/plan.val]
    //   [--check-mutex auto|on|off]
    //   [--val /path/to/validate]
    //   [--val-args "-v"]
    if (argc < 3) {
        std::cerr <<
            "usage: planner_sas <domain.pddl> <problem.pddl>\n"
            "       [--only-search]\n"
            "       [--algo astar|gbfs|soc_astar]\n"
            "       [--search-cpu-limit int(second)]\n"
            "       [--search-mem-limit-mb int(MB)]\n"
            "       [--fd   PATH_TO_SIF]\n"
            "       [--sas-file sas/output.sas]\n"
            "       [--h goalcount|blind]\n"
            "       [--keep-sas]\n"
            "       [--plan-out plans/plan.val]\n"
            "       [--check-mutex auto|on|off]\n"
            "       [--val PATH_TO_VAL]\n"
            "       [--val-args \"...\"]\n"
            "       # parallel search (soc_astar) options\n"
            "       [--soc-threads N]\n"
            "       [--soc-open multi|bucket]\n"
            "       [--soc-queues Q]\n"
            "       [--soc-k K]\n";
        return 1;
    }

    std::string domain = argv[1];
    std::string problem = argv[2];

    bool only_search = false;
    std::string algo = "astar";
    std::string fd = "containers/fast-downward.sif";
    std::string sas_path = "sas/output.sas";
    std::string hname = "goalcount";
    bool keep_sas = true;
    std::string plan_out = "plans/plan.val";
    int mutex_mode = planner::sas::MUTEX_AUTO;
    std::string val_bin;
    std::string val_args;

    // cpu-time & memory audit
    double opt_search_cpu_limit_sec = -1.0; // CPU 時間上限 (negative means invalid)
    std::size_t opt_search_mem_limit_mb = 0; // 仮想メモリ上限 (zero means invalid)

    // soc_search options
    int soc_threads = -1; // -1 の場合、hardware_concurrency() を利用する
    std::string soc_open = "bucket";
    int soc_queues = 0; // 0 の場合、スレッド数と同数のキューを用いる
    int soc_k = 2;

    for (int i=3; i<argc; ++i) {
        std::string a = argv[i];
        if (a == "--algo" && i+1 < argc) {
            algo = argv[++i];
        } else if (a == "--only-search" && i+1 < argc) {
            only_search = true;
        } else if (a == "--search-cpu-limit" && i + 1 < argc) {
            opt_search_cpu_limit_sec = std::stod(argv[++i]);
            continue;
        } else if (a == "--search-mem-limit-mb" && i + 1 < argc) {
            opt_search_mem_limit_mb = static_cast<std::size_t>(std::stoull(argv[++i]));
            continue;
        } else if (a == "--fd" && i+1 < argc) {
            fd = argv[++i];
        } else if (a == "--sas-file" && i+1 < argc) {
            sas_path = argv[++i];
        } else if (a == "--keep-sas") {
            keep_sas = true;
        } else if ((a == "--h" || a == "--heuristic") && i+1 < argc) {
            hname = argv[++i];
        } else if (a == "--plan-out" && i+1 < argc) {
            plan_out = argv[++i];
        } else if (a == "--check-mutex" && i+1 < argc) {
            std::string m = argv[++i];
            std::transform(m.begin(), m.end(), m.begin(), [](unsigned char c){ return std::tolower(c); });
            if (m == "auto") {
                mutex_mode = planner::sas::MUTEX_AUTO;
            } else if (m == "on") {
                mutex_mode = planner::sas::MUTEX_ON;
            } else if (m == "off") {
                mutex_mode = planner::sas::MUTEX_OFF;
            } else {
                std::cerr << "warning: unknown --check-mutex value: " << m << " (use auto|on|off)\n";
            }
        } else if (a == "--val" && i+1 < argc) {
            val_bin = argv[++i];
        } else if (a == "--val-args" && i+1 < argc) {
            val_args = argv[++i];
        } else if (a == "--soc-threads" && i+1 < argc) {
            soc_threads = std::stoi(argv[++i]);
        } else if (a == "--soc-open" && i+1 < argc) {
            soc_open = argv[++i];
            std::transform(soc_open.begin(), soc_open.end(), soc_open.begin(), 
            [](unsigned char c){
                return std::tolower(c);
            });
            if (soc_open != "multi" && soc_open != "bucket") {
                std::cerr << "warning: --soc-open must be multi|bucket (got " << soc_open << "), using bucket\n";
                soc_open = "bucket";
            }
        } else if (a == "--soc-queues" && i+1 < argc) {
            soc_queues = std::stoi(argv[++i]);
        } else if (a == "--soc-k" && i+1 < argc) {
            soc_k = std::stoi(argv[++i]);
        } else {
            std::cerr << "warning: unknown arg ignored: " << a << "\n";
        }
    }

    try {
        using clock = std::chrono::steady_clock;
        const auto t_start = clock::now();

        // 出力先ディレクトリを作成
        if (!sas_path.empty()) {
            fs::path p(sas_path);
            if (p.has_parent_path()) {
                fs::create_directories(p.parent_path());
            }
        }

         if (!plan_out.empty()) {
            fs::path pp(plan_out);
            if (pp.has_parent_path()) {
                fs::create_directories(pp.parent_path());
            }
        }

        // FDのtranslatorを呼ぶ
        if (!only_search) {
            {
                std::ostringstream cmd;
                cmd << shell_quote(fd)
                    << " --translate"
                    << " --sas-file " << shell_quote(sas_path)
                    << " " << shell_quote(domain)
                    << " " << shell_quote(problem)
                    << " >/dev/null 2>&1";

                std::cout << "[FD] running translate (output suppressed)\n";
                const auto t_tr_begin = clock::now();
                int rc = std::system(cmd.str().c_str());
                const auto t_tr_end = clock::now();
                if (rc != 0) {
                    std::cerr << "translator failed with exit code " << rc << "\n";
                    return 2;
                }
                const auto tr_s = std::chrono::duration<double>(t_tr_end - t_tr_begin).count();
                std::cout << std::fixed << std::setprecision(3) << "Translate Time: " << tr_s << " s\n";
            }

            // SASファイルが生成されたか確認
            if (!fs::exists(sas_path)) {
                std::cerr << "error: SAS file not found: " << sas_path << "\n";
                return 2;
            }
            if (fs::file_size(sas_path) == 0) {
                std::cerr << "error: SAS file is empty: " << sas_path << "\n";
                return 2;
            }
        }

        // SAS読込 → A*/GBFS
        planner::sas::Task T = planner::sas::read_file(sas_path);

        // タスクのチェックを行う
        {
            const int nvars = (int)T.vars.size();
            if ((int)T.init.size() != nvars) {
                throw std::runtime_error("validate_task: init size mismatch: init="
                    + std::to_string(T.init.size()) + " vars=" + std::to_string(nvars));
            }
            auto chk_var = [&](int v, const char* where){
                if (v < 0 || v >= nvars) {
                    throw std::runtime_error(std::string("validate_task: var OOB at ")
                        + where + ": v=" + std::to_string(v) + " nvars=" + std::to_string(nvars));
                }
            };
            auto chk_val = [&](int v, int val, const char* where){
                chk_var(v, where);
                int dom = T.vars[v].domain;
                if (val < 0 || val >= dom) {
                    throw std::runtime_error(std::string("validate_task: value OOB at ")
                        + where + ": v=" + std::to_string(v)
                        + " val=" + std::to_string(val) + " domain=" + std::to_string(dom));
                }
            };
            // goal
            for (auto [v,val] : T.goal) {
                chk_val(v, val, "goal");
            }
            // operators
            for (const auto& op : T.ops) {
                for (auto [v,val] : op.prevail) {
                    chk_val(v, val, (std::string("op.prevail: ") + op.name).c_str());
                }
                for (const auto& pp : op.pre_posts) {
                    const auto& conds = std::get<0>(pp);
                    int var = std::get<1>(pp);
                    int pre = std::get<2>(pp);
                    int post= std::get<3>(pp);
                    for (auto [cv,cval] : conds) {
                        chk_val(cv, cval, (std::string("op.cond: ") + op.name).c_str());
                    }
                    chk_var(var, (std::string("op.var: ") + op.name).c_str());
                    if (pre  >= 0) chk_val(var, pre,  (std::string("op.pre: ")  + op.name).c_str());
                    chk_val(var, post, (std::string("op.post: ") + op.name).c_str());
                }
            }
        }

        planner::sas::Params P;
        bool h_is_integer = true;

        {
            using namespace planner::sas;
            g_mutex_mode = mutex_mode;
        }

        planner::sas::Result R;
        bool solved = false; // 探索して解を発見できたかどうか
        std::vector<uint32_t> plan_ops_out; // 出力プラン
        int plan_cost_out = -1; // 出力プランにおけるコスト

        // メモリ制限
    #if defined(__linux__)
        rlimit saved_as{}, saved_data{}; // リソース制限の保持、as はアドレス空間 (仮想メモリ空間) 、data はデータセグメント (ヒープ領域)
        bool saved_as_ok = false, saved_data_ok = false; // 取得に成功したかどうか
        if (opt_search_mem_limit_mb > 0) {
            // 現在のプロセスのメモリ制限を取得する
            if (getrlimit(RLIMIT_AS, &saved_as) == 0) {
                saved_as_ok = true;
            }
            if (getrlimit(RLIMIT_DATA, &saved_data) == 0) {
                saved_data_ok = true;
            }
            rlimit rl{}; // 新しいリソース制限値の設定
            rl.rlim_cur = rl.rlim_max = static_cast<rlim_t>(opt_search_mem_limit_mb) * 1024 * 1024;
            // その制限をシステムに反映させる
            (void)setrlimit(RLIMIT_AS, &rl);
            (void)setrlimit(RLIMIT_DATA, &rl);
        }
    #endif
    
        // CPU バジェットの設定
        planner::sas::g_cpu_budget_enabled = true;
        planner::sas::set_search_cpu_budget(opt_search_cpu_limit_sec);

        const auto t_search_begin = clock::now();

        if (algo == "astar") {
            if (hname == "goalcount") {
                R = planner::sas::astar(T, planner::sas::goalcount(), h_is_integer, P);
            } else if (hname == "blind") {
                R = planner::sas::astar(T, planner::sas::blind(), h_is_integer, P);
            } else {
                throw std::runtime_error(hname + std::string(" is not defined."));
            }
            
            solved = R.solved;
            if (solved) {
                plan_ops_out = R.plan;
                plan_cost_out = static_cast<int>(std::lround(planner::sas::eval_plan_cost(T, R.plan)));
            }

        } else if (algo == "gbfs") {
            if (hname == "goalcount") {
                R = planner::sas::gbfs(T, planner::sas::goalcount(), h_is_integer, P);
            } else if (hname == "blind") {
                R = planner::sas::gbfs(T, planner::sas::blind(), h_is_integer, P);
            } else {
                throw std::runtime_error(hname + std::string(" is not defined."));
            }

            solved = R.solved;
            if (solved) {
                plan_ops_out = R.plan;
                plan_cost_out = static_cast<int>(std::lround(planner::sas::eval_plan_cost(T, R.plan)));
            }

        } else if (algo == "soc_astar") {
            using planner::sas::parallel_SOC::SearchParams;
            using planner::sas::parallel_SOC::SharedOpen;
            using planner::sas::soc::GlobalStats;

            SearchParams sp;

            // スレッド数の調整 (負ならば、使用しているハードウェアのスレッド数に合わせる)
            sp.num_threads = (soc_threads > 0) ? (uint32_t)soc_threads : std::max(1u, std::thread::hardware_concurrency());

            // オープンリストの種類の調整
            sp.open_kind = (soc_open == "multi") ? SharedOpen::Kind::MultiQueue : SharedOpen::Kind::TwoLevelBucket;

            // キューの数の調整
            sp.num_queues = (soc_queues > 0) ? (uint32_t)soc_queues : sp.num_threads;

            // CPU リミットの調整 (ただし、並列探索の time_limit_ms に合うように 1000 を掛ける)
            sp.time_limit_ms = (opt_search_cpu_limit_sec > 0) ? (int)std::llround(opt_search_cpu_limit_sec * 1000.0) : -1;

            GlobalStats GS; // 統計保存用の struct

            // 実際の探索の箇所
            auto RS = planner::sas::parallel_SOC::astar_soc(T, sp, &GS);

            solved = RS.solved;
            if (solved) {
                plan_ops_out = RS.plan_ops; 
                plan_cost_out = RS.cost; 
            }

            // 統計値の表示
            auto total = GS.sum();
            std::cout << "===SOC===" << "\n";
            std::cout << "Threads: " << sp.num_threads << "\n";
            std::cout << " Open=" << (sp.open_kind==SharedOpen::Kind::TwoLevelBucket?"TwoLevelBucket":"MultiQueue") << "\n";
            std::cout << " Queues/Shards: " << sp.num_queues << "\n";
            std::cout << "k: " << soc_k << "\n";
            std::cout << "Expanded: " << total.expanded << "\n";
            std::cout << "Generated: " << total.generated << "\n";
            std::cout << "Evaluated: " << total.evaluated << "\n";
            std::cout << "Reopened: " << total.reopened << "\n";
            std::cout << "Pruned: " << total.duplicates_pruned << "\n";
            std::cout << "Pushes: " << total.pushes << "\n";
            std::cout << "Pops: " << total.pops << "\n";
            std::cout << "Steals: " << total.steals << "\n";
            std::cout << "Bucket empty probes: " << total.bucket_pop_empty_probes << "\n";
            {
                using namespace std::chrono;
                const double secs = duration<double>(nanoseconds(total.relax_eval_ns)).count();
                std::cout << "Evaluation time: " << secs << " s\n";
            }
            std::cout << "Max open size: "  << total.max_open_size_seen << "\n";
            std::cout << "\n";
            
            // 各スレッドごとの統計を表示する
            for (size_t i=0; i<GS.per_thread.size(); ++i){
                std::cout << "===thread: " << i << "===" << "\n";
                std::cout << "Expanded: " << GS.per_thread[i].expanded << "\n";
                std::cout << "Generated: " << GS.per_thread[i].generated << "\n";
                std::cout << "Evaluated: " << GS.per_thread[i].evaluated << "\n";
                std::cout << "Reopened: " << GS.per_thread[i].reopened << "\n";
                std::cout << "Pruned: " << GS.per_thread[i].duplicates_pruned << "\n";
                std::cout << "Pushes: " << GS.per_thread[i].pushes << "\n";
                std::cout << "Pops: " << GS.per_thread[i].pops << "\n";
                std::cout << "Steals: " << GS.per_thread[i].steals << "\n";
                std::cout << "Bucket empty probes: " << GS.per_thread[i].bucket_pop_empty_probes << "\n";
                {
                using namespace std::chrono;
                const double secs = duration<double>(nanoseconds(total.relax_eval_ns)).count();
                std::cout << "Evaluation time: " << secs << " s\n";
                }
                std::cout << "Max open size: "  << GS.per_thread[i].max_open_size_seen << "\n";
                std::cout << "\n";
            }
        } else {
            throw std::runtime_error(algo + std::string(" is not defined."));
        }

        const auto t_search_end = clock::now();

        // CPU バジェットの解除
        planner::sas::set_search_cpu_budget(-1.0);

        // メモリ上限の復元
    #if defined(__linux__)
        if (opt_search_mem_limit_mb > 0) {
            if (saved_as_ok) {
                (void)setrlimit(RLIMIT_AS, &saved_as);
            }
            if (saved_data_ok) {
                (void)setrlimit(RLIMIT_DATA, &saved_data);
            }
        }
    #endif

        if (solved) {
            std::cout << "Solution found.\n";
            if (algo != "soc_astar") {
                std::cout << "Expanded: " << R.stats.expanded << " state(s)" << "\n";
                std::cout << "Generated: " << R.stats.generated << " state(s)" << "\n";
                std::cout << "Evaluated: " << R.stats.evaluated << " state(s)" << "\n";
            }

            // VAL形式のテキストを生成
            const std::string plan_txt = planner::sas::plan_to_val(T, plan_ops_out);
            // ファイルに保存
            if (!plan_out.empty()) {
                std::ofstream ofs(plan_out, std::ios::binary);
                if (!ofs) {
                    throw std::runtime_error("failed to open plan file for write: " + plan_out);
                }
                ofs << plan_txt;
                ofs.flush();
                if (!ofs) {
                    throw std::runtime_error("failed to write plan file: " + plan_out);
                }
                std::cout << "[PLAN] wrote: " << plan_out << "\n";
            } else {
                std::cout << plan_txt << std::endl;
            }
            // VALを実行（指定がある場合のみ）
            if (!val_bin.empty() && !plan_out.empty()) {

                std::ostringstream vcmd;

                if (!val_args.empty()) {
                    vcmd << shell_quote(val_bin) << " " << val_args
                         << " " << shell_quote(domain) << " " << shell_quote(problem) << " " << shell_quote(plan_out);
                } else {
                    vcmd << shell_quote(val_bin)
                         << " " << shell_quote(domain) << " " << shell_quote(problem) << " " << shell_quote(plan_out);
                }

                int vrc = system((vcmd.str() + " >/dev/null 2>&1").c_str());

                if (vrc == 0) {
                    const auto cost = (plan_cost_out >= 0) ? plan_cost_out : planner::sas::eval_plan_cost(T, plan_ops_out);
                    std::cout << "[VAL] Plan valid (cost=" << cost << ")\n";
                } else {
                    std::cout << "[VAL] Validation failed (exit=" << vrc << ")\n";
                }
            }
        } else {
            std::cout << "No solution.\n";
        }

        // 時間表示（Search / Total）
        const auto search_ms = std::chrono::duration<double>(t_search_end - t_search_begin).count();
        const auto t_end      = t_search_end;
        const auto total_ms   = std::chrono::duration<double>(t_end - t_start).count();
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Search Time: " << search_ms << " s\n"
                  << "Total Planning Time: " << total_ms << " s\n";

        if (!keep_sas) {
            std::error_code ec;
            fs::remove(sas_path, ec);
        }
        return solved ? 0 : 3;
    } catch (const std::bad_alloc&) {
        std::cerr << "fatal: memory limit exceeded (bad_alloc)\n";
        return 102; // memory limit
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 9;
    }
}
