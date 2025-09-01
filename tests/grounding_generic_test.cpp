#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "lexer.hpp"
#include "parser.hpp"
#include "grounding.hpp"

using namespace planner;

static std::string slurp_file(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) { std::cerr << "File open failed: " << path << "\n"; std::exit(1); }
    std::ostringstream oss; oss << ifs.rdbuf(); return oss.str();
}

static void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " <domain.pddl> <problem.pddl> [--dump]\n";
}

static void dump_problem_objects(const Problem& Pr) {
    std::cerr << "== Problem Objects ==" << "\n";
    for (const auto& o : Pr.objects) {
        std::cerr << "  " << o.first << " : " << o.second << "\n";
    }
}

int main(int argc, char** argv) {
    if (argc < 3) { print_usage(argv[0]); return 2; }
    const std::string domain_path  = argv[1];
    const std::string problem_path = argv[2];
    const bool dump = (argc >= 4 && std::string(argv[3]) == "--dump");

    try {
        const std::string dom  = slurp_file(domain_path);
        const std::string prob = slurp_file(problem_path);

        // ---- parse domain ----
        Lexer L(dom); Parser P(L); Domain D = P.parseDomain();

        // ---- parse problem ----
        Lexer L2(prob); Parser P2(L2); Problem Pr = P2.parseProblem();

        // トラブル時に見えるよう一旦出す
        dump_problem_objects(Pr);

        // ---- grounding ----
        GroundTask G = ground(D, Pr);

        // ---- basic checks ----
        assert(!G.preds.empty());
        assert(!G.actions.empty());

        std::cout << "[Grounding Generic Test]\n";
        std::cout << "Domain file : " << domain_path  << "\n";
        std::cout << "Problem file: " << problem_path << "\n";
        std::cout << "Objects     : " << G.objects.size() << "\n";
        std::cout << "Predicates  : " << G.preds.size()   << "\n";
        std::cout << "Init facts(+): " << G.init_pos.size() << "\n";
        std::cout << "Goal facts(+): " << G.goal_pos.size() << "\n";
        std::cout << "Ground actions: " << G.actions.size() << "\n";

        if (dump) {
            const std::size_t SHOW = std::min<std::size_t>(G.actions.size(), 5);
            for (std::size_t i = 0; i < SHOW; ++i) {
                const auto& a = G.actions[i];
                std::cout << "  " << a.name << " cost=" << a.cost << "\n";
                for (const auto& pre : a.pre_pos) std::cout << "    pre+ " << to_string(pre, G) << "\n";
                for (const auto& pre : a.pre_neg) std::cout << "    pre- " << to_string(pre, G) << "\n";
                for (const auto& ef  : a.eff_add) std::cout << "    add  " << to_string(ef,  G) << "\n";
                for (const auto& ef  : a.eff_del) std::cout << "    del  " << to_string(ef,  G) << "\n";
            }
        }

        std::cout << "---- Pruning stats ----\n";
        std::cout << "Candidates (before pruning): " << G.stats.candidates << "\n";
        std::cout << "  pruned by typing/all-diff : " << G.stats.by_typing_allDiff << "\n";
        std::cout << "  pruned by static preds   : " << G.stats.by_static << "\n";
        std::cout << "  pruned by forward R+     : " << G.stats.by_forward << "\n";
        std::cout << "  pruned by backward rel   : " << G.stats.by_backward << "\n";
        std::cout << "Ground actions (final)     : " << G.actions.size() << "\n";

        std::cout << "OK\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] " << e.what() << "\n";
        std::cerr << "Hint:\n"
          "  • problem の :objects に、:init/:goal で使っている全ての定数が宣言されているか確認してください。\n"
          "  • domain の型名と problem の型指定が一致しているか（大小・綴り違いを含む）。\n"
          "  • 定数名の大小（lexer が大文字小文字を区別する実装なら特に注意）。\n";
        return 1;
    }
}