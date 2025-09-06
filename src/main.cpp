#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>

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
      << "  " << argv0 << " <domain.pddl> <problem.pddl> [--algo astar] [--h blind|goalcount|wgoalcount W]\n"
      << "Examples:\n"
      << "  " << argv0 << " domain.pddl problem.pddl --algo astar --h goalcount\n"
      << "  " << argv0 << " domain.pddl problem.pddl --algo astar --h wgoalcount 2.0\n";
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

        for (int i=3; i<argc; ++i) {
            std::string a = argv[i];
            if (a == "--algo" && i+1 < argc) { // algorithm を指定する引数の場合
                algo = argv[++i];
                continue;
            }
            if (a == "--h" && i+1 < argc) { // heuristic function を指定する引数の場合
                hname = argv[++i];
                if (hname == "wgoalcount") { // w を指定する weighted heuristic function の場合
                    if (i+1 >= argc) { // w を指定する引数がない場合 (index が追い付いてしまった場合)
                        throw std::runtime_error("--h wgoalcount needs a weight");
                    }
                    w = std::stod(argv[++i]);
                }
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

        // --- Report ---
        std::cout << "\n== Search Result ==\n";
        std::cout << "Solved: " << (res.solved ? "YES" : "NO") << "\n";
        std::cout << "Search Time: " << search_time << " ms\n";
        std::cout << "Generated: " << res.stats.generated
                  << ", Expanded: " << res.stats.expanded
                  << ", Duplicates: " << res.stats.duplicates << "\n";

        if (res.solved) {
            std::cout << "Plan length: " << res.plan.size()
                      << ", Cost: " << res.plan_cost << "\n";
            std::cout << "\nPlan:\n" << plan_to_string(ST, res.plan) << "\n";
        }
        return res.solved ? 0 : 2;

    } catch (const LexerError& e) { // PDDL 自体が間違えている場合
        std::cerr << "[lexer] " << e.what() << "\n";
        return 10;
    } catch (const std::exception& e) { // その他例外が起きた場合
        std::cerr << "[error] " << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }
}
