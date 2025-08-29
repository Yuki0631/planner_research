#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include "parser.hpp" 

namespace planner {

// Ground化された命題 述語ID と引数 ID の組 によって表現する
struct GroundAtom {
    int pred = -1; // predicate id                
    std::vector<int> args; // argument ids        
};

// Ground化されたアクション 作用名 と前提・効果の組 によって表現する
struct GroundAction {
    std::string name;   
    std::vector<GroundAtom> pre_pos; // positive preconditions
    std::vector<GroundAtom> pre_neg; // negative preconditions
    std::vector<GroundAtom> eff_add; // effects to add
    std::vector<GroundAtom> eff_del; // effects to delete

    double cost = 0.0; // アクションコスト（メトリック用）
};

struct GroundTask {
    // オブジェクト、および、その id, type を 管理するデータ構造 (vector, hashmap)
    std::vector<std::string> objects; // object の管理用 vector
    std::unordered_map<std::string,int> obj_id; // name -> id の 写像
    std::unordered_map<std::string,std::string> obj_ty; // name -> type の写像

    // 述語スキーマの管理
    struct PredSchema {
        std::string name;
        std::vector<std::string> types; 
    };
    std::vector<PredSchema> preds; // predicates 管理用 vector
    std::unordered_map<std::string,int> pred_id; // name -> id の写像

    // 関数スキーマの管理
    struct FuncSchema {
        std::string name; std::vector<std::string> types;
    };
    std::vector<FuncSchema> funcs; // functions 管理用 vector
    std::unordered_map<std::string,int> func_id; // name -> id の写像

    // 数値初期化テーブル: "fname(arg1,...)" -> value 名前と引数の組み合わせをキーにして、関数値を保存する
    std::unordered_map<std::string,double> func_values;

    // init
    std::vector<GroundAtom> init_pos;

    // goal
    std::vector<GroundAtom> goal_pos;
    std::vector<GroundAtom> goal_neg;

    // Ground 化されたアクションを管理するデータ構造 (vector)
    std::vector<GroundAction> actions;
};

// Ground 化を行う関数 (pddl を解析した Domain, Problem から GroundTask を生成する)
GroundTask ground(const Domain& d, const Problem& p);

// デバッグ用の表示関数
std::string to_string(const GroundAtom& ga, const GroundTask& gt);

} // namespace planner
