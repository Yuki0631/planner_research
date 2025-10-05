#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdexcept>
#include <filesystem>
#include <sstream>

#include "sas/sas_reader.hpp"
#include "sas/sas_search.hpp"

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
    //   [--keep-sas]
    if (argc < 3) {
        std::cerr <<
            "usage: planner_sas <domain.pddl> <problem.pddl>\n"
            "       [--algo astar|gbfs]\n"
            "       [--fd   PATH_TO_SIF]\n"
            "       [--sas-file sas/output.sas]\n"
            "       [--keep-sas]\n";
        return 1;
    }

    std::string domain = argv[1];
    std::string problem = argv[2];

    std::string algo = "astar";
    std::string fd = "containers/fast-downward.sif";
    std::string sas_path = "sas/output.sas";
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

        planner::sas::Params P;
        auto goalcount = [](const planner::sas::Task& TT, const planner::sas::State& s)->double {
            int miss = 0;
            for (auto [v,val] : TT.goal) {
                if (s[v] != val) {
                    ++miss;
                }
            }
            return (double)miss;
        };
        bool h_is_integer = true;

        planner::sas::Result R = (algo == "gbfs")
            ? planner::sas::gbfs(T, goalcount, h_is_integer, P)
            : planner::sas::astar(T, goalcount, h_is_integer, P);

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
