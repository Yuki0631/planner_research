#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "lexer.hpp"
#include "parser.hpp"
#include "grounding.hpp"

using namespace planner;

static std::string slurp_file(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "File open failed: " << path << "\n";
        std::exit(1);
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

static void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " <domain.pddl> <problem.pddl> [--dump]\n"
                 "  --dump  : Ground結果の要約と先頭数件のアクションを表示する\n";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 2;
    }
    const std::string domain_path  = argv[1];
    const std::string problem_path = argv[2];
    const bool dump = (argc >= 4 && std::string(argv[3]) == "--dump");

    const std::string dom  = slurp_file(domain_path);
    const std::string prob = slurp_file(problem_path);

    // --- parse domain ---
    Lexer L(dom);
    Parser P(L);
    Domain D = P.parseDomain();

    // --- parse problem ---
    Lexer L2(prob);
    Parser P2(L2);
    Problem Pr = P2.parseProblem();

    // --- grounding ---
    GroundTask G = ground(D, Pr);

    // --- 必須の軽量チェック（ドメインに依存しない範囲）---
    // 少なくとも述語とアクションが1つ以上あること
    assert(!G.preds.empty());
    assert(!G.actions.empty());

    // --- 概要出力 ---
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

    std::cout << "OK\n";
    return 0;
}
