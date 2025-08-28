#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include "lexer.hpp"

using planner::Lexer;
using planner::Token;
using planner::TokenType;

static const char* tyname(TokenType t){
    switch (t){
        case TokenType::LPAR: return "LPAR";
        case TokenType::RPAR: return "RPAR";
        case TokenType::KEYWORD: return "KEYWORD";
        case TokenType::VARIABLE: return "VARIABLE";
        case TokenType::NAME: return "NAME";
        case TokenType::NUMBER: return "NUMBER";
        case TokenType::DASH: return "DASH";
        case TokenType::EOF_TOKEN: return "EOF";
    }
    return "?";
}

static std::string read_all(std::istream& is){
    std::ostringstream oss; oss << is.rdbuf(); return oss.str();
}

struct Summary {
    std::map<TokenType, std::size_t> count;
    std::size_t total = 0;
    int paren_depth_end = 0;
    int paren_depth_min = 0;
    // 先頭の (define (domain|problem ...)) 簡易検出
    bool starts_with_define_domain = false;
    bool starts_with_define_problem = false;
    // よく出るキーワードの有無（参考）
    bool has_kw_requirements = false;
    bool has_kw_types = false;
    bool has_kw_predicates = false;
    bool has_kw_action = false;
    bool has_kw_parameters = false;
    bool has_kw_precondition = false;
    bool has_kw_effect = false;
    bool has_kw_domain = false; // problem 内
    bool has_kw_objects = false;
    bool has_kw_init = false;
    bool has_kw_goal = false;
};

static Summary scan_lex(const std::string& src, bool dump){
    Summary S;
    Lexer lex(src);
    int depth = 0, min_depth = 0;

    std::vector<Token> first8;

    while (true){
        Token t = lex.next();
        if (S.total < 8) first8.push_back(t);
        S.total++;
        S.count[t.type]++;

        if (t.type == TokenType::LPAR) depth++;
        else if (t.type == TokenType::RPAR) depth--;
        if (depth < min_depth) min_depth = depth;

        if (t.type == TokenType::KEYWORD){
            const std::string& k = t.lexeme;
            if (k == "requirements")   S.has_kw_requirements = true;
            else if (k == "types")     S.has_kw_types = true;
            else if (k == "predicates")S.has_kw_predicates = true;
            else if (k == "action")    S.has_kw_action = true;
            else if (k == "parameters")S.has_kw_parameters = true;
            else if (k == "precondition") S.has_kw_precondition = true;
            else if (k == "effect")    S.has_kw_effect = true;
            else if (k == "domain")    S.has_kw_domain = true;
            else if (k == "objects")   S.has_kw_objects = true;
            else if (k == "init")      S.has_kw_init = true;
            else if (k == "goal")      S.has_kw_goal = true;
        }

        if (dump){
            std::cout << tyname(t.type);
            if (!t.lexeme.empty()) std::cout << "  '" << t.lexeme << "'";
            std::cout << "  @" << t.loc.line << ":" << t.loc.col << "\n";
        }

        if (t.type == TokenType::EOF_TOKEN) break;
    }

    S.paren_depth_end = depth;
    S.paren_depth_min = min_depth;

    if (first8.size() >= 4) {
        if (first8[0].type == TokenType::LPAR &&
            first8[1].type == TokenType::NAME && first8[1].lexeme == "define" &&
            first8[2].type == TokenType::LPAR &&
            first8[3].type == TokenType::NAME)
        {
            if (first8[3].lexeme == "domain")  S.starts_with_define_domain  = true;
            if (first8[3].lexeme == "problem") S.starts_with_define_problem = true;
        }
    }

    return S;
}

static int run_one(const std::string& path, bool dump, bool expect_domain){
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "[FAIL] cannot open: " << path << "\n";
        return 2;
    }
    std::string src = read_all(ifs);

    std::cout << "=== LEX TEST: " << path << " ===\n";
    Summary S;
    try {
        S = scan_lex(src, dump);
    } catch (const planner::LexerError& e) {
        std::cerr << "[LexerError] " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << "\n";
        return 1;
    }

    // サマリ表示
    std::cout << "Tokens: " << S.total << "\n";
    for (auto&& kv : S.count){
        std::cout << "  " << tyname(kv.first) << ": " << kv.second << "\n";
    }
    std::cout << "Paren depth end=" << S.paren_depth_end
              << " (min=" << S.paren_depth_min << ")\n";

    bool ok = true;

    // かっこ整合
    if (S.paren_depth_min < 0){
        std::cerr << "[ERROR] more ')' than '(' at some point (min depth "
                  << S.paren_depth_min << ")\n";
        ok = false;
    }
    if (S.paren_depth_end != 0){
        std::cerr << "[ERROR] parentheses not balanced (final depth "
                  << S.paren_depth_end << ")\n";
        ok = false;
    }

    // 期待ファイル種別による軽い検査
    if (expect_domain) {
        if (!S.starts_with_define_domain) {
            std::cerr << "[WARN] does not start with (define (domain ...))\n";
        }
        if (!(S.has_kw_predicates || S.has_kw_action)) {
            std::cerr << "[WARN] no ':predicates' or ':action' detected (domain)\n";
        }
    } else {
        if (!S.starts_with_define_problem) {
            std::cerr << "[WARN] does not start with (define (problem ...))\n";
        }
        if (!S.has_kw_domain) std::cerr << "[WARN] missing ':domain' in problem\n";
        if (!S.has_kw_init)   std::cerr << "[WARN] missing ':init' in problem\n";
        if (!S.has_kw_goal)   std::cerr << "[WARN] missing ':goal' in problem\n";
    }

    if (ok) {
        std::cout << "[PASS] lexical scan OK\n";
        return 0;
    } else {
        std::cout << "[FAIL]\n";
        return 1;
    }
}

int main(int argc, char** argv){
    if (argc < 3){
        std::cerr << "Usage: " << argv[0] << " <test_domain.pddl> <test_problem.pddl> [--dump]\n";
        return 2;
    }
    std::string domain_path = argv[1];
    std::string problem_path = argv[2];
    bool dump = (argc >= 4 && std::string(argv[3]) == "--dump");

    int code1 = run_one(domain_path,  dump, /*expect_domain=*/true);
    int code2 = run_one(problem_path, dump, /*expect_domain=*/false);

    return std::max(code1, code2);
}
