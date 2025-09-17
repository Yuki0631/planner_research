#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <filesystem>

#include "lexer.hpp"
#include "parser.hpp"
#include "grounding.hpp"
#include "strips.hpp"
#include "heuristic.hpp"
#include "bucket_pq.hpp"
#include "search.hpp"

using namespace planner;

// file を読み取る関数
static std::string slurp(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) { throw std::runtime_error("cannot open: " + path); }
    std::ostringstream oss; 
    oss << ifs.rdbuf();
    return oss.str();
}

// エラー発生時に使用法を表示するための関数
static void print_usage(const char* argv0) {
    std::cerr
      << "Usage:\n"
      << "  " << argv0 << " <domain.pddl> <problem.pddl> [--algo astar] "<< "[--h blind|goalcount|wgoalcount W] [--plan-dir <DIR>]\n"
      << "Examples:\n"
      << "  " << argv0 << " domain.pddl problem.pddl --algo astar --h goalcount --plan-dir directory\n"
      << "  " << argv0 << " domain.pddl problem.pddl --algo astar --h wgoalcount 2.0 --plan-dir directory\n";
}

int main(int argc, char** argv) {
    try {
        if (argc < 3) { // 引数が足りない場合
            print_usage(argv[0]);
            return 1;
        }

        std::string dom_path = argv[1];
        std::string prb_path = argv[2];

        std::string algo = "astar";
        std::string hname = "goalcount";
        double w = 1.0;
        std::string plan_dir; // plan を出力するディレクトリ

        for (int i=3; i<argc; ++i) {
            std::string a = argv[i];
            if (a == "--algo" && i+1 < argc) { // algorithm を指定する引数の場合
                algo = argv[++i];
                continue;
            }
            if (a == "--h" && i+1 < argc) { // heuristic function を指定する引数の場合
                hname = argv[++i];
                if (hname == "wgoalcount") { // w を指定する weighted heuristic function の場合
                    if (i+1 >= (argc - 2)) { // w を指定する引数がない場合 (index の位置が追い越してしまった場合)
                        throw std::runtime_error("--h wgoalcount needs a weight");
                    }
                    w = std::stod(argv[++i]);
                }
                continue;
            }
            if (a == "--plan-dir" && i+1 < argc) {
                plan_dir = argv[++i];
                continue;
            }
            if (a == "--help" || a == "-h") { //　使用法を確認したい場合
                print_usage(argv[0]);
                return 0;
            }
            throw std::runtime_error("unknown option: " + a);
        }

        // --- Parse ---
        double parse_time = 0.0;
        auto t0_parse = std::chrono::steady_clock::now();
        const std::string dom_txt = slurp(dom_path);
        const std::string prb_txt = slurp(prb_path);

        Lexer Ld(dom_txt), Lp(prb_txt);
        Parser Pd(Ld), Pp(Lp);

        Domain  d = Pd.parseDomain();
        Problem p = Pp.parseProblem();
        auto t1_parse = std::chrono::steady_clock::now();
        parse_time = std::chrono::duration<double, std::milli>(t1_parse - t0_parse).count();

        // --- PDDL -> Ground -> STRIPS ---
        // Domain + Problem を Ground化する
        double ground_time = 0.0;
        auto t0_ground = std::chrono::steady_clock::now();
        GroundTask G = ground(d, p);
        auto t1_ground = std::chrono::steady_clock::now();
        ground_time = std::chrono::duration<double, std::milli>(t1_ground - t0_ground).count();

        // GroundTask -> StripsTask
        double strips_time = 0.0;
        auto t0_strips = std::chrono::steady_clock::now();
        StripsTask ST = compile_to_strips(G);
        auto t1_strips = std::chrono::steady_clock::now();
        strips_time = std::chrono::duration<double, std::milli>(t1_strips - t0_strips).count();

        // 簡単に要約を出力する
        std::cout << "=== Summary ===\n"
                  << "Parse Time: " << parse_time << " ms\n"
                  << "Candidates (before pruning): " << G.stats.candidates << "\n"
                  << "  pruned by typing/all-diff : " << G.stats.by_typing_allDiff << "\n"
                  << "  pruned by static preds   : " << G.stats.by_static << "\n"
                  << "  pruned by forward R+     : " << G.stats.by_forward << "\n"
                  << "  pruned by backward rel   : " << G.stats.by_backward << "\n"
                  << "Ground actions (final)     : " << G.actions.size() << "\n"
                  << "Ground Time: " << ground_time << " ms\n"
                  << "Strips Time: " << strips_time << " ms\n"
                  << "Objects: " << G.objects.size()
                  << ", Facts: " << ST.num_facts()
                  << ", Actions: " << ST.actions.size() << "\n";

        // --- Heuristic ---
        HeuristicFn hf;
        if (hname == "blind") hf = make_blind();
        else if (hname == "goalcount") hf = make_goalcount();
        else if (hname == "wgoalcount") hf = make_weighted_goalcount(w);
        else throw std::runtime_error("unknown heuristic: " + hname);

        // --- Search ---
        SearchParams params;
        SearchResult  res;
        double search_time = 0.0;

        if (algo == "astar") {
            auto t0_search = std::chrono::steady_clock::now();
            res = astar(ST, hf, params);
            auto t1_search = std::chrono::steady_clock::now();
            search_time = std::chrono::duration<double, std::milli>(t1_search - t0_search).count();
        } else {
            throw std::runtime_error("unknown algo: " + algo);
        }

        // --- Report (FD-compatible) ---
        std::cout.setf(std::ios::fixed);
        std::cout.precision(3);

        double search_time_s = search_time / 1000.0;
        double total_time_s  = (parse_time + ground_time + strips_time) / 1000.0 + search_time_s;

        if (res.solved) {
            std::cout << "Solution found." << std::endl;
            std::cout << "Plan length: " << res.plan.size() << " step(s)." << std::endl;
            std::cout << "Plan cost: "   << res.plan_cost << "." << std::endl;
        } else {
            std::cout << "Completely explored state space — no solution!" << std::endl;
        }

        // FD 互換の統計行
        std::cout << "Expanded "  << res.stats.expanded  << " state(s)." << std::endl;
        std::cout << "Generated " << res.stats.generated << " state(s)." << std::endl;
        std::cout << "Search time: " << search_time_s << "s" << std::endl;
        std::cout << "Total time: "  << total_time_s  << "s" << std::endl;

        if (res.solved) {
            // 出力先ディレクトリ 指定がなければカレントディレクトリ
            std::filesystem::path outdir = plan_dir.empty() ? std::filesystem::current_path() : std::filesystem::path(plan_dir);
            std::error_code ec;
            std::filesystem::create_directories(outdir, ec);

            // 既存の sas_plan, sas_plan.1, ... を避けて連番にする
            std::filesystem::path plan_path = outdir / "sas_plan";
            int idx = 0;
            while (std::filesystem::exists(plan_path)) {
                ++idx;
                plan_path = outdir / ("sas_plan." + std::to_string(idx));
            }

            std::ofstream ofs(plan_path);
            if (!ofs) {
                std::cerr << "[warn] cannot write plan file: " << plan_path.string() << std::endl;
            } else {
                ofs << plan_to_val(ST, res.plan);
                // 末尾にメタ情報を追加する
                ofs << "; cost = "   << res.plan_cost     << "\n";
                ofs << "; length = " << res.plan.size()   << "\n";
                ofs.close();

                std::cout << "Wrote plan to: " << plan_path.string() << std::endl;
            }
        }
        int exit_code = 0;
        if (!res.solved) {
            exit_code = 1; // 解なし（探索完了）を 1 に
        }
        return exit_code;

    } catch (const LexerError& e) { // PDDL 自体が間違えている場合
        std::cerr << "[lexer] " << e.what() << "\n";
        return 10;
    } catch (const std::exception& e) { // その他例外が起きた場合
        std::cerr << "[error] " << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }
}
