#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdexcept>
#include <cassert>

#include <sas/sas_reader.hpp>

using planner::sas::Task;
using planner::sas::State;
using planner::sas::read_file;
using planner::sas::violates_mutex;

static void die_usage(const char* argv0) {
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " <path/to/output.sas>\n\n"
        << "Runs structural & consistency checks against the given SAS file and\n"
        << "prints a short summary. Returns non-zero on failure.\n";
    std::exit(2);
}

// --- helpers ---

// 要約を表示する関数
static void print_summary(const Task& T) {
    std::cout << "=== SAS Summary ===\n";
    std::cout << "version: " << T.version << "\n";
    std::cout << "metric : " << T.metric  << "\n";
    std::cout << "#vars  : " << T.vars.size() << "\n";
    std::cout << "#mutex : " << T.mutexes.size() << "\n";
    std::cout << "init   : " << T.init.size() << " values\n";
    std::cout << "goal   : " << T.goal.size() << " atoms\n";
    std::cout << "#ops   : " << T.ops.size() << "\n";
    std::cout << "===================\n";
}

// 各解析で理論上起こりえない点が存在しないか走査する関数
static void check_bounds(const Task& T) {
    const int nvars = static_cast<int>(T.vars.size());

    // 変数のドメイン値の数が負または 0 でないか判定する
    for (int v = 0; v < nvars; ++v) {
        if (T.vars[v].domain <= 0) {
            throw std::runtime_error("variable[" + std::to_string(v) + "] has non-positive domain");
        }
    }

    // 初期状態の変数の数が、変数の数と一致するか判定する
    if (static_cast<int>(T.init.size()) != nvars) {
        throw std::runtime_error("init state size mismatch vs #vars");
    }

    // 初期状態の変数のドメイン値が、適切な範囲にあるか判定する
    for (int v = 0; v < nvars; ++v) {
        const int val = T.init[v];
        if (val < 0 || val >= T.vars[v].domain) {
            throw std::runtime_error("init value out of range at var " + std::to_string(v));
        }
    }

    // ゴール状態の変数の id とドメイン値が適切な範囲にあるか判定する
    for (size_t i = 0; i < T.goal.size(); ++i) {
        auto [v, val] = T.goal[i];
        if (v < 0 || v >= nvars) {
            throw std::runtime_error("goal var id out of range at row " + std::to_string(i));
        }
        if (val < 0 || val >= T.vars[v].domain) {
            throw std::runtime_error("goal value out of range at row " + std::to_string(i));
        }
    }

    // 排他グループの変数の id とドメイン値が適切な範囲にあるか判定する
    for (size_t g = 0; g < T.mutexes.size(); ++g) {
        for (auto [v, val] : T.mutexes[g].lits) {
            if (v < 0 || v >= nvars) {
                throw std::runtime_error("mutex[" + std::to_string(g) + "] var id out of range");
            }
            if (val < 0 || val >= T.vars[v].domain) {
                throw std::runtime_error("mutex[" + std::to_string(g) + "] value out of range");
            }
        }
    }

    // 演算子が適切であるかどうかの判定
    for (size_t oi = 0; oi < T.ops.size(); ++oi) {

        const auto& op = T.ops[oi];

        // prevail 条件の変数の id とドメイン値が適切な範囲にあるか判定する
        for (size_t i = 0; i < op.prevail.size(); ++i) {
            auto [v, val] = op.prevail[i];
            if (v < 0 || v >= nvars) {
                throw std::runtime_error("op[" + std::to_string(oi) + "] prevail var oob");
            }
            if (val < 0 || val >= T.vars[v].domain) {
                throw std::runtime_error("op[" + std::to_string(oi) + "] prevail val oob");
            }
        }

        // 効果
        for (size_t i = 0; i < op.pre_posts.size(); ++i) {

            const auto& pp = op.pre_posts[i];

            // 効果の条件の変数の id とドメイン値が適切な範囲にあるか判定する
            for (const auto& c : std::get<0>(pp)) {
                if (c.first < 0 || c.first >= nvars) {
                    throw std::runtime_error("op[" + std::to_string(oi) + "] cond var oob");
                }
                if (c.second < 0 || c.second >= T.vars[c.first].domain) {
                    throw std::runtime_error("op[" + std::to_string(oi) + "] cond val oob");
                }
            }

            // 変数の id が適切な範囲にあるか判定する
            if (std::get<1>(pp) < 0 || std::get<1>(pp) >= nvars) {
                throw std::runtime_error("op[" + std::to_string(oi) + "] effect var oob");
            }

            // 効果の適用前のドメイン値が適切な範囲にあるか判定する
            if (std::get<2>(pp) < -1 || std::get<2>(pp) >= T.vars[std::get<1>(pp)].domain) { // -1 の場合はどの値でもいい時
                throw std::runtime_error("op[" + std::to_string(oi) + "] effect pre out of range (-1 or [0,dom))");
            }

            // 効果の適用後のドメイン値が適切な範囲にあるか判定する
            if (std::get<3>(pp) < 0 || std::get<3>(pp) >= T.vars[std::get<1>(pp)].domain) {
                throw std::runtime_error("op[" + std::to_string(oi) + "] effect post out of range");
            }
        }

        if (op.cost < 0) {
            throw std::runtime_error("op[" + std::to_string(oi) + "] has negative cost");
        }
    }
}

static void check_mutex_invariants_on_init(const Task& T) {
    if (violates_mutex(T, T.init)) {
        throw std::runtime_error("initial state violates a mutex group");
    }
}

// 初期様態に演算子を適用した後に、排他グループに違反しないか判定する
static void spot_check_operators_do_not_introduce_mutex(const Task& T) {
    State s = T.init;
    const int nvars = static_cast<int>(T.vars.size());

    for (size_t oi = 0; oi < T.ops.size(); ++oi) {
        const auto& op = T.ops[oi];

        // 演算子を状態に適応できるかどうか確認する関数
        auto holds = [&](const State& st) {
            for (auto [v, val] : op.prevail) {
                if (st[v] != val) return false;
            }
            for (const auto& pp : op.pre_posts) {
                if (std::get<2>(pp) != -1 && st[std::get<1>(pp)] != std::get<2>(pp)) { // 効果の適用前ドメイン値が適切かどうか
                    return false;
                }
                for (const auto& c : std::get<0>(pp)) { // 効果の条件を満たしているかどうか
                    if (st[c.first] != c.second) {
                        return false;
                    }
                }
            }
            return true;
        };

        if (!holds(s)) { // 初期状態に適応できない場合
            continue;
        }

        State s2 = s;

        // 適応できる場合は、状態の値を変化させる
        for (const auto& pp : op.pre_posts) {
            assert(std::get<1>(pp) >= 0 && std::get<1>(pp) < nvars);
            s2[std::get<1>(pp)] = std::get<3>(pp);
        }
        if (violates_mutex(T, s2)) { // 変化後の状態が排他グループに違反する場合
            throw std::runtime_error(
                "operator '" + op.name + "' introduces a mutex violation when applied to init");
        }
    }
}

// --- main ---

int main(int argc, char** argv) {
    if (argc < 2) {
        die_usage(argv[0]);
    }
    const std::string sas_path = argv[1];

    try {
        Task T = read_file(sas_path);

        print_summary(T);

        // structural/semantic checks
        check_bounds(T);
        check_mutex_invariants_on_init(T);
        spot_check_operators_do_not_introduce_mutex(T);

        std::cout << "[OK] All checks passed.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[FAIL] unknown error\n";
        return 1;
    }
}