#include <cassert>
#include <iostream>
#include <string>
#include <cmath>
#include "lexer.hpp"
#include "parser.hpp"
#include "grounding.hpp"

using namespace planner;

int main() {
    const char* dom = R"(
(define (domain switch)
  (:requirements :strips :typing :action-costs)
  (:predicates (switch_is_on) (switch_is_off))
  (:functions (total-cost) - number)
  (:action switch_on
    :parameters ()
    :precondition (switch_is_off)
    :effect (and (switch_is_on)
                 (not (switch_is_off))
                 (increase (total-cost) 1)))
  (:action switch_off
    :parameters ()
    :precondition (switch_is_on)
    :effect (and (switch_is_off)
                 (not (switch_is_on))
                 (increase (total-cost) 1)))
)
)"
;

    const char* prob = R"(
(define (problem p1)
  (:domain switch)
  (:objects)
  (:init (switch_is_off) (= (total-cost) 0))
  (:goal (switch_is_on))
  (:metric minimize (total-cost))
)
)"
;

    // parse
    {
        Lexer L(dom);
        Parser P(L);
        Domain D = P.parseDomain();

        Lexer L2(prob);
        Parser P2(L2);
        Problem Pr = P2.parseProblem();

        // ground
        GroundTask G = ground(D, Pr);

        // 簡単な検証
        assert(G.objects.size() == 0); // オブジェクトなし
        assert(G.preds.size() == 2);
        assert(G.init_pos.size() == 1);
        assert(G.goal_pos.size() == 1);
        assert(G.actions.size() == 2);

        // アクションコスト1が付いていることを確認
        bool ok_cost = false;
        for (auto& a : G.actions) {
            if (a.name.find("switch_on") != std::string::npos) {
                ok_cost |= (std::fabs(a.cost - 1.0) < 1e-9);
            }
        }
        assert(ok_cost);

        // 画面出力（人間確認用）
        std::cout << "Ground actions: " << G.actions.size() << "\n";
        for (auto& a : G.actions) {
            std::cout << "  " << a.name << " cost=" << a.cost << "\n";
            for (auto& pre : a.pre_pos) std::cout << "    pre+ " << to_string(pre, G) << "\n";
            for (auto& pre : a.pre_neg) std::cout << "    pre- " << to_string(pre, G) << "\n";
            for (auto& ef  : a.eff_add) std::cout << "    add  " << to_string(ef,  G) << "\n";
            for (auto& ef  : a.eff_del) std::cout << "    del  " << to_string(ef,  G) << "\n";
        }
    }

    std::cout << "OK\n";
    return 0;
}
