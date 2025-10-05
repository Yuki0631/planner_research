#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdexcept>
#include <filesystem>
#include <sstream>
#include <functional>

#include "sas/sas_reader.hpp"
#include "sas/sas_search.hpp"
#include "sas/sas_heuristic.hpp"

namespace fs = std::filesystem;

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
    //   [--algo astar|gbfs]
    //   [--fd containers/fast-downward.sif]
    //   [--sas-file sas/output.sas]
    //   [--h goalcount|blind]
    //   [--keep-sas]
    if (argc < 3) {
        std::cerr <<
            "usage: planner_sas <domain.pddl> <problem.pddl>\n"
            "       [--algo astar|gbfs]\n"
            "       [--fd   PATH_TO_SIF]\n"
            "       [--sas-file sas/output.sas]\n"
            "       [--h goalcount|blind]\n"
            "       [--keep-sas]\n";
        return 1;
    }

    std::string domain = argv[1];
    std::string problem = argv[2];

    std::string algo = "astar";
    std::string fd = "containers/fast-downward.sif";
    std::string sas_path = "sas/output.sas";
    std::string hname = "goalcount";
    bool keep_sas = true; // 既定で残す

    for (int i=3; i<argc; ++i) {
        std::string a = argv[i];
        if (a == "--algo" && i+1 < argc) {
            algo = argv[++i];
        } else if (a == "--fd" && i+1 < argc) {
            fd = argv[++i];
        } else if (a == "--sas-file" && i+1 < argc) {
            sas_path = argv[++i];
        } else if (a == "--keep-sas") {
            keep_sas = true;
        } else if ((a == "--h" || a == "--heuristic") && i+1 < argc) {
            hname = argv[++i];
        } else {
            std::cerr << "warning: unknown arg ignored: " << a << "\n";
        }
    }

    try {
        // 出力先ディレクトリを作成
        if (!sas_path.empty()) {
            fs::path p(sas_path);
            if (p.has_parent_path()) {
                fs::create_directories(p.parent_path());
            }
        }

        // FDのtranslatorを呼ぶ
        {
            std::ostringstream cmd;
            cmd << shell_quote(fd)
                << " --translate"
                << " --sas-file " << shell_quote(sas_path)
                << " " << shell_quote(domain)
                << " " << shell_quote(problem);

            std::cout << "[FD] " << cmd.str() << "\n";
            int rc = std::system(cmd.str().c_str());
            if (rc != 0) {
                std::cerr << "translator failed with exit code " << rc << "\n";
                return 2;
            }
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

        planner::sas::Result R;


        if (algo == "astar") {
            if (hname == "goalcount") {
                R = planner::sas::astar(T, planner::sas::goalcount(), h_is_integer, P);
            } else if (hname == "blind") {
                R = planner::sas::astar(T, planner::sas::blind(), h_is_integer, P);
            } else {
                throw std::runtime_error(hname + std::string(" is not defined."));
            }
        } else if (algo == "gbfs") {
            if (hname == "goalcount") {
                R = planner::sas::gbfs(T, planner::sas::goalcount(), h_is_integer, P);
            } else if (hname == "blind") {
                R = planner::sas::gbfs(T, planner::sas::blind(), h_is_integer, P);
            } else {
                throw std::runtime_error(hname + std::string(" is not defined."));
            }
        } else {
            throw std::runtime_error(algo + std::string(" is not defined."));
        }

        if (R.solved) {
            std::cout << "Solution found.\n";
            std::cout << planner::sas::plan_to_val(T, R.plan) << std::endl;
        } else {
            std::cout << "No solution.\n";
        }

        if (!keep_sas) {
            std::error_code ec;
            fs::remove(sas_path, ec);
        }
        return R.solved ? 0 : 3;
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 9;
    }
}
