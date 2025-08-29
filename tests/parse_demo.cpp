#include <iostream>
#include <fstream>
#include <sstream>
#include "lexer.hpp"
#include "parser.hpp"

using namespace planner;

static std::string slurp(const std::string& path){
    std::ifstream ifs(path);
    if (!ifs) throw std::runtime_error("cannot open: " + path);
    std::ostringstream oss; oss << ifs.rdbuf();
    return oss.str();
}

int main(int argc, char** argv){
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <test_domain.pddl> <test_problem.pddl>\n";
        return 2;
    }
    try {
        // Domain
        {
            std::string s = slurp(argv[1]);
            Lexer L(s);
            Parser P(L);
            Domain D = P.parseDomain();
            std::cout << "Domain: " << D.name << "\n";
            std::cout << "  requirements: ";
            for (auto& r : D.requirements) std::cout << ":" << r << " ";
            std::cout << "\n  types: ";
            for (auto& t : D.types) std::cout << t << " ";
            std::cout << "\n  predicates:\n";
            for (auto& pr : D.predicates) {
                std::cout << "    " << pr.name << "(";
                for (size_t i=0;i<pr.params.size();++i){
                    if (i) std::cout << ", ";
                    std::cout << pr.params[i].name;
                    if (!pr.params[i].type.empty()) std::cout << " - " << pr.params[i].type;
                }
                std::cout << ")\n";
            }
            std::cout << "  functions:\n";
            for (auto& fn : D.functions) {
                std::cout << "    " << fn.name << "(";
                for (size_t i=0;i<fn.params.size();++i){
                    if (i) std::cout << ", ";
                    std::cout << fn.params[i].name;
                    if (!fn.params[i].type.empty()) std::cout << " - " << fn.params[i].type;
                }
                std::cout << ") : " << fn.rettype << "\n";
            }
            std::cout << "  actions:\n";
            for (auto& a : D.actions) {
                std::cout << "    " << a.name << " params(";
                for (size_t i=0;i<a.params.size();++i){
                    if (i) std::cout << ", ";
                    std::cout << a.params[i].name;
                    if (!a.params[i].type.empty()) std::cout << " - " << a.params[i].type;
                }
                std::cout << ")\n";
                std::cout << "      pre:  " << Parser::to_string(a.precond) << "\n";
                std::cout << "      eff:  " << Parser::to_string(a.effect) << "\n";
            }
        }

        // Problem
        {
            std::string s = slurp(argv[2]);
            Lexer L(s);
            Parser P(L);
            Problem Pr = P.parseProblem();
            std::cout << "Problem: " << Pr.name << "  (domain=" << Pr.domain_name << ")\n";
            std::cout << "  objects: ";
            for (auto& o : Pr.objects) {
                std::cout << o.first;
                if (!o.second.empty()) std::cout << " - " << o.second;
                std::cout << "  ";
            }
            std::cout << "\n  init:\n";
            for (auto& a : Pr.init) std::cout << "    " << Parser::to_string(a) << "\n";
            std::cout << "  goal: " << Parser::to_string(Pr.goal) << "\n";
            if (Pr.metric.present) {
                std::cout << "  metric: "
                          << (Pr.metric.sense == planner::Problem::Metric::MINIMIZE ? "minimize" : "maximize")
                          << " " << Parser::to_string(Pr.metric.expr) << "\n";
            }
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[parse error] " << e.what() << "\n";
        return 1;
    }
}
